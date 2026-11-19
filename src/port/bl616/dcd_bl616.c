/*
 * TinyUSB device-controller driver (DCD) for the Bouffalo BL616 USB 2.0
 * high-speed device controller.
 *
 * Adapted from the device half of Bouffalo Lab's CherryUSB port,
 * drivers/lhal/src/bflb_usb_v2.c in bouffalo_sdk (bl_mcu_sdk), which is
 * Copyright (c) Bouffalo Lab and licensed under the Apache License, Version 2.0
 * (http://www.apache.org/licenses/LICENSE-2.0). The register sequences (PHY
 * power-up, controller init, FIFO/endpoint mapping, VDMA transfers, interrupt
 * groups) are taken from that file; the CherryUSB event calls are replaced by
 * TinyUSB's dcd_event_* and the API by TinyUSB's dcd_* (src/device/dcd.h).
 * Changes from the original: TinyUSB API, deferred start (below), DMA buffers
 * through the non-cacheable RAM alias, BL616 only.
 *
 * Controller facts this driver relies on (read from the SDK driver):
 *  - Endpoints 1-4 only, besides EP0. Each endpoint NUMBER owns one 512-byte
 *    FIFO (F0-F3), so each number is used in one direction only. The
 *    DualSense descriptor already uses 0x01 OUT, 0x82 IN, 0x03 OUT, 0x84 IN.
 *  - Transfers go through the controller's VDMA, one channel per FIFO plus
 *    one for EP0 (CXF); completion is the VDMA-complete interrupt.
 *
 * Start order: dcd_init() only records state. The controller is powered up by
 * bl616_dcd_start(), called once Bluetooth is up (bt_transport_bl616.cpp), with
 * the USB clock re-applied first: on this board, starting USB before Bluetooth
 * init lost the port during Bluetooth bring-up. GPIO32/33 are never touched
 * (not led out on QFN40); the USB PHY needs no pin mux.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "device/dcd.h"

#include "bflb_core.h"
#include "bflb_clock.h"
#include "bflb_irq.h"
#include "bflb_l1c.h"
#include "bflb_mtimer.h"
#include "bl616_glb.h"
#include "hardware/usb_v2_reg.h"

#ifndef BL616
#error "dcd_bl616.c is for the BL616"
#endif

#if DS5_BL616_DIAG_LOG
/* Debug build: transfer counters, bus events and the SETUP requests that change
 * the device's state, written to the log ring from the interrupt
 * (ports/bl616_haute/diag_log.c, ring-only lines, safe here). */
#include "diag_log.h"
#define DIAG(stmt) do { stmt; } while (0)
#else
#define DIAG(stmt) do { } while (0)
#endif

#define BFLB_USB_BASE           ((uint32_t)0x20072000)
#define BFLB_PDS_BASE           ((uint32_t)0x2000E000)
#define BFLB_USB_IRQ_NUM        37
#define PDS_USB_CTL_OFFSET      (0x500) /* usb_ctl */
#define PDS_USB_PHY_CTRL_OFFSET (0x504) /* usb_phy_ctrl */

/* 0x500 : usb_ctl */
#define PDS_REG_USB_SW_RST_N   (1 << 0U)
#define PDS_REG_USB_EXT_SUSP_N (1 << 1U)
#define PDS_REG_USB_WAKEUP     (1 << 2U)
#define PDS_REG_USB_IDDIG      (1 << 5U)

/* 0x504 : usb_phy_ctrl */
#define PDS_REG_USB_PHY_PONRST       (1 << 0U)
#define PDS_REG_USB_PHY_XTLSEL_SHIFT (2U)
#define PDS_REG_USB_PHY_XTLSEL_MASK  (0x3 << PDS_REG_USB_PHY_XTLSEL_SHIFT)
#define PDS_REG_PU_USB20_PSW         (1 << 6U)

#define USB_SOF_TIMER_MASK_AFTER_RESET_HS (0x44C)

#define USB_FIFO_F0  0
#define USB_FIFO_F1  1
#define USB_FIFO_F2  2
#define USB_FIFO_F3  3
#define USB_FIFO_CXF 0xff

#define USB_FIFO_DIR_BID 2

#define EP_COUNT 5 /* EP0 + EP1-4 */

typedef struct {
    uint16_t mps;
    uint8_t type;
    bool enable;
    volatile bool active;
    uint8_t *buf;
    uint16_t len;
} ep_state_t;

static ep_state_t ep_in[EP_COUNT];
static ep_state_t ep_out[EP_COUNT];

/* The VDMA writes the SETUP packet here; keep it out of the data cache. */
static __attribute__((section(".noncacheable"), aligned(4))) uint8_t setup_buf[8];

static volatile bool g_started;
static volatile bool g_want_connected;
static volatile bool g_int_enabled;
static volatile bool g_remote_wakeup_pending;

/* Every wait on the controller is bounded (wait_reg_clear below). A wait that
 * times out is counted here, in production too, and in the debug build logged
 * with the register it was polling. */
static volatile uint32_t g_wait_timeouts;
/* A SETUP arrived while a control VDMA was still armed (the host abandoned a
 * control transfer); that VDMA was stopped before the SETUP was read. */
static volatile uint32_t g_cx_vdma_preempted;
#if DS5_BL616_DIAG_LOG
static volatile bool g_speed_after_reset_pending;

/* Variant P: what the USB-silence check needs (bl616_dcd_diag_check_silence).
 * "Configured" is the last SET_CONFIGURATION's value being non-zero, cleared by
 * a bus reset; "suspended" is a suspend interrupt not yet followed by a resume,
 * a reset or a SETUP. */
static volatile bool g_diag_configured;
static volatile bool g_diag_suspended;
static volatile uint32_t g_diag_last_irq_ms;   /* last USB device interrupt of any kind */
static volatile uint32_t g_diag_disc_ms;       /* dcd_disconnect() time, 0 while connected */
static volatile bool g_diag_disc_while_configured;
static uint32_t g_diag_sfn_last;
static uint32_t g_diag_sfn_change_ms;
static bool g_diag_sfn_sampled;
static bool g_diag_sfn_moving;                 /* the SOF frame number has been seen to change */
static bool g_diag_suspend_noted;
/* The 200 ms after an audio alt-setting change: every EP1/EP2 transfer and
 * completion is logged, up to DIAG_WIN_MAX_LINES lines, plus the first
 * DIAG_WIN_FIRST completions per endpoint whenever they come. */
#define DIAG_WIN_MS          200u
#define DIAG_WIN_MAX_LINES   64u
#define DIAG_WIN_FIRST       8u
static volatile uint32_t g_diag_win_until_ms;
static volatile uint32_t g_diag_win_lines;
static volatile uint8_t g_diag_win_first[3];
static volatile uint32_t g_diag_arm_us[3];

/* The last USB events, in two rings: bus and control events, and EP1/EP2 data
 * events (EP3/EP4 HID traffic is only counted). Dumped by bl616_dcd_diag_dump(). */
enum {
    EV_SETUP = 1, EV_BUS_RESET, EV_SUSPEND, EV_RESUME, EV_OPEN, EV_CLOSE_ALL, EV_CONNECT, EV_DISCONNECT,
    EV_STALL, EV_SETUP_TIMEOUT, EV_ZLP_RX, EV_ARM, EV_DONE, EV_CLEAR_STALL,
};
struct diag_usb_evt {
    uint32_t t_ms;
    uint8_t kind;
    uint8_t ep;
    uint16_t a;
    uint32_t b;
};
#define DIAG_EVT_N 24u
static struct diag_usb_evt g_ev_ctl[DIAG_EVT_N];
static struct diag_usb_evt g_ev_dat[DIAG_EVT_N];
static volatile uint32_t g_ev_ctl_n, g_ev_dat_n;

extern bool usb_keyboard_only; /* usb.cpp */

static void ev_push(bool data, uint8_t kind, uint8_t ep, uint16_t a, uint32_t b)
{
    uintptr_t const flags = bflb_irq_save();
    struct diag_usb_evt *ring = data ? g_ev_dat : g_ev_ctl;
    volatile uint32_t *n = data ? &g_ev_dat_n : &g_ev_ctl_n;
    struct diag_usb_evt *e = &ring[*n % DIAG_EVT_N];
    e->t_ms = (uint32_t)bflb_mtimer_get_time_ms();
    e->kind = kind;
    e->ep = ep;
    e->a = a;
    e->b = b;
    *n = *n + 1u;
    bflb_irq_restore(flags);
}

static const char *ev_name(uint8_t k)
{
    switch (k) {
        case EV_SETUP: return "SETUP";
        case EV_BUS_RESET: return "BUSRESET";
        case EV_SUSPEND: return "SUSPEND";
        case EV_RESUME: return "RESUME";
        case EV_OPEN: return "OPEN";
        case EV_CLOSE_ALL: return "CLOSEALL";
        case EV_CONNECT: return "CONNECT";
        case EV_DISCONNECT: return "DISCONNECT";
        case EV_STALL: return "STALL";
        case EV_SETUP_TIMEOUT: return "SETUP_TIMEOUT";
        case EV_ZLP_RX: return "ZLP_RX";
        case EV_ARM: return "ARM";
        case EV_DONE: return "DONE";
        case EV_CLEAR_STALL: return "CLRSTALL";
        default: return "?";
    }
}

/* EP1/EP2 transfers and completions near a stream start, as log lines. */
static void diag_iso_trace(bool done, uint8_t ep_addr, uint32_t len, uint32_t armed_len)
{
    uint8_t const num = (uint8_t)(ep_addr & 0x0F);
    if (num != 1 && num != 2) {
        return;
    }
    ev_push(true, done ? EV_DONE : EV_ARM, ep_addr, (uint16_t)len, armed_len);
    uint32_t const now = (uint32_t)bflb_mtimer_get_time_ms();
    uint32_t const us = (uint32_t)bflb_mtimer_get_time_us();
    bool const in_window = (int32_t)(g_diag_win_until_ms - now) > 0;
    if (!done) {
        g_diag_arm_us[num] = us;
    }
    if ((!in_window && !(done && g_diag_win_first[num] != 0)) || g_diag_win_lines >= DIAG_WIN_MAX_LINES) {
        return;
    }
    g_diag_win_lines++;
    if (done) {
        if (g_diag_win_first[num] != 0) {
            g_diag_win_first[num]--;
        }
        diag_logf("[usb] t=%lu.%03lu ep 0x%02x DONE len=%lu of %lu armed, %lu us after it was armed%s\r\n",
                  (unsigned long)(us / 1000u), (unsigned long)(us % 1000u), ep_addr, (unsigned long)len,
                  (unsigned long)armed_len, (unsigned long)(us - g_diag_arm_us[num]),
                  g_diag_win_lines == DIAG_WIN_MAX_LINES ? " (trace line limit reached)" : "");
    } else {
        diag_logf("[usb] t=%lu.%03lu ep 0x%02x ARM len=%lu%s\r\n", (unsigned long)(us / 1000u),
                  (unsigned long)(us % 1000u), ep_addr, (unsigned long)len,
                  g_diag_win_lines == DIAG_WIN_MAX_LINES ? " (trace line limit reached)" : "");
    }
}
#endif

/* The SETUP packet is already in the control FIFO when its interrupt fires, so
 * the 8-byte VDMA read completes in microseconds; 1 ms is far beyond that. */
#define DCD_SETUP_READ_TIMEOUT_US 1000u
/* The controller's soft reset, at start-up only. */
#define DCD_SOFT_RESET_TIMEOUT_US 10000u

/* ---------------------------------------------------------------- helpers */

/* Polls until every bit of mask is clear at addr, for at most timeout_us.
 * Returns true if the bits cleared; false on timeout, counted, and in the debug
 * build logged as "[usb] t=T TIMEOUT waiting for <what> (reg=0x...)". Safe in
 * the interrupt: the machine timer is read directly. */
static bool wait_reg_clear(uint32_t addr, uint32_t mask, uint32_t timeout_us, const char *what)
{
    uint64_t const t0 = bflb_mtimer_get_time_us();
    uint32_t regval;
    while ((regval = getreg32(addr)) & mask) {
        if (bflb_mtimer_get_time_us() - t0 >= timeout_us) {
            g_wait_timeouts++;
            DIAG(diag_logf("[usb] t=%lu TIMEOUT waiting for %s (reg=0x%08lx), %lu timeout(s) so far\r\n",
                           (unsigned long)bflb_mtimer_get_time_ms(), what, (unsigned long)regval,
                           (unsigned long)g_wait_timeouts));
            (void)what;
            return false;
        }
    }
    return true;
}

/* TinyUSB's endpoint buffers are placed in .noncacheable (bl616_tusb_mem.h).
 * Anything else that reaches the VDMA through the cached RAM alias
 * (0x6xxxxxxx) is cleaned before a transmit and invalidated after a receive. */
static inline bool addr_is_cached(const void *p)
{
    return (((uintptr_t)p) & 0xF0000000u) == 0x60000000u;
}

static void reset_fifo(uint8_t fifo)
{
    uint32_t regval;

    if (fifo == USB_FIFO_CXF) {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
        regval |= USB_CX_CLR;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
    } else {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_FIBC0_OFFSET + 4 * fifo);
        regval |= USB_FFRST0_HOV;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_FIBC0_OFFSET + 4 * fifo);
    }
}

static void set_ep_fifomap(uint8_t ep_idx, uint8_t fifo)
{
    uint32_t regaddr = BFLB_USB_BASE + USB_DEV_EPMAP0_OFFSET;
    uint32_t regval = getreg32(regaddr);

    regval &= ~(0xff << ((ep_idx - 1) * 8));
    regval |= (fifo << ((ep_idx - 1) * 8));
    regval |= (fifo << ((ep_idx - 1) * 8 + 4));
    putreg32(regval, regaddr);
}

static void set_fifo_epmap(uint8_t fifo, uint8_t ep_idx, uint8_t dir)
{
    uint32_t regaddr = BFLB_USB_BASE + USB_DEV_FMAP_OFFSET;
    uint32_t regval = getreg32(regaddr);

    regval &= ~(0x3f << (fifo * 8));
    regval |= (ep_idx << (fifo * 8));
    regval |= (dir << (fifo * 8 + 4));
    putreg32(regval, regaddr);
}

static void set_outep_mps(uint8_t ep_idx, uint16_t mps)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
    regval &= ~USB_MAXPS_OEP1_MASK;
    regval |= mps;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
}

static void set_inep_mps(uint8_t ep_idx, uint16_t mps)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
    regval &= ~USB_MAXPS_IEP1_MASK;
    regval |= mps;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
}

static void set_mult(uint8_t ep_idx, uint8_t mult)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
    regval &= ~USB_TX_NUM_HBW_IEP1_MASK;
    regval |= (mult << USB_TX_NUM_HBW_IEP1_SHIFT);
    putreg32(regval, BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
}

static uint8_t get_fifo_ep(uint8_t fifo)
{
    uint32_t regval = (getreg32(BFLB_USB_BASE + USB_DEV_FMAP_OFFSET) & (0xf << (fifo * 8)));
    return (uint8_t)(regval >> (fifo * 8));
}

static void fifo_config(uint8_t fifo, uint8_t ep_type, uint16_t block_size, uint8_t block_num, bool fifo_en)
{
    uint32_t regaddr = BFLB_USB_BASE + USB_DEV_FCFG_OFFSET;
    uint32_t regval = getreg32(regaddr);

    regval &= ~(0x3f << (fifo * 8));
    regval |= (ep_type << (fifo * 8 + 0));
    regval |= ((block_num - 1) << (fifo * 8 + 2));
    if (block_size == 1024) {
        regval |= (1 << (fifo * 8 + 4));
    }
    if (fifo_en) {
        regval |= (1 << (fifo * 8 + 5));
    }
    putreg32(regval, regaddr);
}

static void vdma_regs(uint8_t fifo, uint32_t *ctrl, uint32_t *mem)
{
    if (fifo == USB_FIFO_CXF) {
        *ctrl = BFLB_USB_BASE + USB_VDMA_CXFPS1_OFFSET;
        *mem = BFLB_USB_BASE + USB_VDMA_CXFPS2_OFFSET;
    } else {
        *ctrl = BFLB_USB_BASE + USB_VDMA_F0PS1_OFFSET + fifo * 8;
        *mem = BFLB_USB_BASE + USB_VDMA_F0PS2_OFFSET + fifo * 8;
    }
}

static void vdma_start(uint8_t fifo, const uint8_t *data, uint32_t len, bool to_fifo)
{
    uint32_t ctrl, mem;
    vdma_regs(fifo, &ctrl, &mem);

    uint32_t regval = getreg32(ctrl);
    regval &= ~USB_VDMA_LEN_CXF_MASK;
    regval &= ~USB_VDMA_IO_CXF;
    if (to_fifo) {
        regval |= USB_VDMA_TYPE_CXF;
    } else {
        regval &= ~USB_VDMA_TYPE_CXF;
    }
    regval |= (len << USB_VDMA_LEN_CXF_SHIFT);
    putreg32(regval, ctrl);

    putreg32((uint32_t)(uintptr_t)data, mem);

    regval = getreg32(ctrl);
    regval |= USB_VDMA_START_CXF;
    putreg32(regval, ctrl);
}

/* Drop a VDMA transfer still armed on a FIFO (an OUT read the host never
 * filled), before the endpoint is reconfigured on an alternate-setting or
 * configuration change. */
static void vdma_stop(uint8_t fifo)
{
    uint32_t ctrl, mem;
    vdma_regs(fifo, &ctrl, &mem);
    uint32_t regval = getreg32(ctrl);
    if (regval & USB_VDMA_START_CXF) {
        putreg32(regval & ~USB_VDMA_START_CXF, ctrl);
    }
}

static uint32_t vdma_remain(uint8_t fifo)
{
    uint32_t ctrl, mem;
    vdma_regs(fifo, &ctrl, &mem);
    return (getreg32(ctrl) & USB_VDMA_LEN_CXF_MASK) >> USB_VDMA_LEN_CXF_SHIFT;
}

static inline void control_transfer_done(void)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
    regval |= USB_CX_DONE;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
}

static inline void send_zlp(uint8_t ep_idx)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
    regval |= USB_TX0BYTE_IEP1;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
}

static uint32_t group_intstatus(uint8_t group)
{
    switch (group) {
        case 0:
            return getreg32(BFLB_USB_BASE + USB_DEV_ISG0_OFFSET) & ~getreg32(BFLB_USB_BASE + USB_DEV_MISG0_OFFSET);
        case 2:
            return getreg32(BFLB_USB_BASE + USB_DEV_ISG2_OFFSET) & ~getreg32(BFLB_USB_BASE + USB_DEV_MISG2_OFFSET);
        case 3:
            return getreg32(BFLB_USB_BASE + USB_DEV_ISG3_OFFSET) & ~getreg32(BFLB_USB_BASE + USB_DEV_MISG3_OFFSET);
        default:
            return 0;
    }
}

static void group_int_clear(uint8_t group, uint32_t bits)
{
    switch (group) {
        case 0:
            putreg32(bits, BFLB_USB_BASE + USB_DEV_ISG0_OFFSET);
            break;
        case 2:
            putreg32(bits, BFLB_USB_BASE + USB_DEV_ISG2_OFFSET);
            break;
        case 3:
            putreg32(bits, BFLB_USB_BASE + USB_DEV_ISG3_OFFSET);
            break;
        default:
            break;
    }
}

static tusb_speed_t port_speed(void)
{
    uint32_t speed = (getreg32(BFLB_USB_BASE + USB_OTG_CSR_OFFSET) & USB_SPD_TYP_HOV_POV_MASK) >> USB_SPD_TYP_HOV_POV_SHIFT;
    if (speed == 0) {
        return TUSB_SPEED_FULL;
    } else if (speed == 1) {
        return TUSB_SPEED_LOW;
    }
    return TUSB_SPEED_HIGH;
}

#if DS5_BL616_DIAG_LOG
static const char *speed_name(tusb_speed_t s)
{
    return s == TUSB_SPEED_HIGH ? "high" : (s == TUSB_SPEED_FULL ? "full" : "low");
}
#endif

static void reset_all_fifos(void)
{
    reset_fifo(USB_FIFO_F0);
    reset_fifo(USB_FIFO_F1);
    reset_fifo(USB_FIFO_F2);
    reset_fifo(USB_FIFO_F3);
    reset_fifo(USB_FIFO_CXF);
}

static void ep0_reset_state(void)
{
    memset(ep_in, 0, sizeof(ep_in));
    memset(ep_out, 0, sizeof(ep_out));
    ep_in[0].mps = CFG_TUD_ENDPOINT0_SIZE;
    ep_in[0].enable = true;
    ep_out[0].mps = CFG_TUD_ENDPOINT0_SIZE;
    ep_out[0].enable = true;
}

/* ------------------------------------------------------------- PHY/controller */

static void phy_init(void)
{
    uint32_t regval;

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval &= ~PDS_REG_USB_PHY_XTLSEL_MASK;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval |= PDS_REG_PU_USB20_PSW;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);
    regval |= PDS_REG_USB_PHY_PONRST;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_PHY_CTRL_OFFSET);

    bflb_mtimer_delay_us(1);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval &= ~PDS_REG_USB_SW_RST_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    bflb_mtimer_delay_us(1);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_EXT_SUSP_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    bflb_mtimer_delay_ms(3); /* wait UCLK, 1.2 ms */

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_SW_RST_N;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    bflb_mtimer_delay_ms(2);
}

static void apply_connect(bool connected)
{
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_PHY_TST_OFFSET);
    if (connected) {
        regval &= ~USB_UNPLUG;
    } else {
        regval |= USB_UNPLUG;
    }
    putreg32(regval, BFLB_USB_BASE + USB_PHY_TST_OFFSET);
}

static void usb_isr(int irq, void *arg)
{
    (void)irq;
    (void)arg;
    DIAG(diag_note_usb_isr(1, NULL));
    dcd_int_handler(0);
    DIAG(diag_note_usb_isr(0, NULL));
}

static void controller_init(void)
{
    uint32_t regval;

    phy_init();

    bflb_irq_attach(BFLB_USB_IRQ_NUM, usb_isr, NULL);

    /* disable device-A for host */
    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_IDDIG;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    /* disable global irq */
    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_GLINT_EN_HOV;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);

    apply_connect(false);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_CAP_RMWAKUP;
    regval |= USB_CHIP_EN_HOV;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_FORCE_FS; /* high speed */
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval |= USB_SFRST_HOV;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    if (!wait_reg_clear(BFLB_USB_BASE + USB_DEV_CTL_OFFSET, USB_SFRST_HOV, DCD_SOFT_RESET_TIMEOUT_US,
                        "the controller soft reset (DEV_CTL SFRST)")) {
        /* Carry on: the configuration below is still written, and a controller
         * that never leaves reset simply does not enumerate. */
        printf("[USB] controller soft reset did not complete in %u us\r\n", (unsigned)DCD_SOFT_RESET_TIMEOUT_US);
    }

    regval = getreg32(BFLB_USB_BASE + USB_DEV_ADR_OFFSET);
    regval &= ~USB_AFT_CONF;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_ADR_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_SMT_OFFSET);
    regval &= ~USB_SOFMT_MASK;
    regval |= USB_SOF_TIMER_MASK_AFTER_RESET_HS;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_SMT_OFFSET);

    /* group 0: SETUP only (the others masked), as the SDK driver */
    regval = getreg32(BFLB_USB_BASE + USB_DEV_MISG0_OFFSET);
    regval &= ~USB_MCX_SETUP_INT;
    regval |= USB_MCX_IN_INT;
    regval |= (1 << 3);
    regval |= USB_MCX_OUT_INT;
    regval |= USB_MCX_COMFAIL_INT;
    regval |= USB_MCX_COMABORT_INT;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_MISG0_OFFSET);

    /* group 1: every FIFO interrupt masked (completion is VDMA-driven) */
    putreg32(0xffffffff, BFLB_USB_BASE + USB_DEV_MISG1_OFFSET);

    /* group 2: reset, suspend, resume, zero-length IN/OUT */
    regval = 0xffffffff;
    regval &= ~USB_MUSBRST_INT;
    regval &= ~USB_MSUSP_INT;
    regval &= ~USB_MRESM_INT;
    regval &= ~USB_MTX0BYTE_INT;
    regval &= ~USB_MRX0BYTE_INT;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_MISG2_OFFSET);

    /* group 3: VDMA complete, EP0 and F0-F3 */
    regval = 0xffffffff;
    regval &= ~(USB_MVDMA_CMPLT_CXF | USB_MVDMA_CMPLT_F0 | USB_MVDMA_CMPLT_F1 | USB_MVDMA_CMPLT_F2 | USB_MVDMA_CMPLT_F3);
    putreg32(regval, BFLB_USB_BASE + USB_DEV_MISG3_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_MIGR_OFFSET);
    regval &= ~(USB_MINT_G0 | USB_MINT_G1 | USB_MINT_G2 | USB_MINT_G3 | USB_MINT_G4);
    putreg32(regval, BFLB_USB_BASE + USB_DEV_MIGR_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_GLB_INT_OFFSET);
    regval |= USB_MHC_INT;
    regval |= USB_MOTG_INT;
    regval &= ~USB_MDEV_INT;
    putreg32(regval, BFLB_USB_BASE + USB_GLB_INT_OFFSET);

    group_int_clear(2, 0x3ff);
    group_int_clear(3, 0xffffffff);

    for (uint8_t i = 1; i < 5; i++) {
        set_ep_fifomap(i, 15);
        set_fifo_epmap(i - 1, 15, 0);
    }

    regval = getreg32(BFLB_USB_BASE + USB_VDMA_CTRL_OFFSET);
    regval |= USB_VDMA_EN;
    putreg32(regval, BFLB_USB_BASE + USB_VDMA_CTRL_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_HCMISC_OFFSET);
    regval |= USB_HC_WKP_DET_EN;
    regval |= USB_HC_CONN_DET_EN;
    putreg32(regval, BFLB_USB_BASE + USB_HCMISC_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval |= USB_GLINT_EN_HOV;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
}

/* Called once Bluetooth is up (or by the fallback in bl616_dcd_poll()). Runs
 * in the main loop's task, never from an interrupt. */
void bl616_dcd_start(void)
{
    if (g_started) {
        return;
    }
    /* USB bus clock gate on and the USB PLL divider fed from the Wi-Fi PLL,
     * immediately before the controller starts: board_init() did both once
     * and Bluetooth init runs in between. */
    PERIPHERAL_CLOCK_USB_ENABLE();
    GLB_Set_USB_CLK_From_WIFIPLL(1);

    ep0_reset_state();
    controller_init();
    g_started = true;
    if (g_int_enabled) {
        bflb_irq_enable(BFLB_USB_IRQ_NUM);
    }
    apply_connect(g_want_connected);
    printf("[USB] controller started, %s\r\n", g_want_connected ? "connected" : "waiting to connect");
}

bool bl616_dcd_started(void)
{
    return g_started;
}

/* Main loop (bt_transport_poll): if Bluetooth has not come up 20 s after
 * boot, start USB anyway so a keyboard-only or unpaired adapter still
 * enumerates. Normally bl616_dcd_start() has already run. */
void bl616_dcd_poll(void)
{
    if (!g_started && bflb_mtimer_get_time_ms() >= 20000u) {
        printf("[USB] Bluetooth not up after 20 s; starting USB anyway\r\n");
        bl616_dcd_start();
    }
}

/* ------------------------------------------------------------ controller API */

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init)
{
    (void)rhport;
    (void)rh_init;
    ep0_reset_state();
    g_want_connected = true; /* TinyUSB expects a connected device after init */
    return true;
}

void dcd_int_enable(uint8_t rhport)
{
    (void)rhport;
    g_int_enabled = true;
    if (g_started) {
        bflb_irq_enable(BFLB_USB_IRQ_NUM);
    }
}

void dcd_int_disable(uint8_t rhport)
{
    (void)rhport;
    g_int_enabled = false;
    if (g_started) {
        bflb_irq_disable(BFLB_USB_IRQ_NUM);
    }
}

void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
    (void)rhport;
    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_ADR_OFFSET);
    regval &= ~USB_DEVADR_MASK;
    regval |= dev_addr;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_ADR_OFFSET);
    /* The DCD sends the status stage of SET_ADDRESS itself (dcd.h). */
    control_transfer_done();
}

void dcd_remote_wakeup(uint8_t rhport)
{
    (void)rhport;
    if (!g_started) {
        return;
    }
    uint32_t regval;

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval |= USB_CAP_RMWAKUP;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval |= PDS_REG_USB_WAKEUP;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    bflb_mtimer_delay_ms(10);

    regval = getreg32(BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);
    regval &= ~PDS_REG_USB_WAKEUP;
    putreg32(regval, BFLB_PDS_BASE + PDS_USB_CTL_OFFSET);

    regval = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
    regval &= ~USB_CAP_RMWAKUP;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
}

void dcd_connect(uint8_t rhport)
{
    (void)rhport;
    g_want_connected = true;
    DIAG(g_diag_disc_ms = 0; ev_push(false, EV_CONNECT, 0, 0, diag_cnt.usb_disc_caller));
    if (g_started) {
        apply_connect(true);
    }
}

void dcd_disconnect(uint8_t rhport)
{
    (void)rhport;
    g_want_connected = false;
    DIAG(uint32_t const t = (uint32_t)bflb_mtimer_get_time_ms(); g_diag_disc_ms = t != 0 ? t : 1;
         g_diag_disc_while_configured = g_diag_configured;
         ev_push(false, EV_DISCONNECT, 0, g_diag_configured ? 1 : 0, diag_cnt.usb_disc_caller));
    if (g_started) {
        apply_connect(false);
    }
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
    /* Nothing here needs SOF (no feedback endpoint). */
    (void)rhport;
    (void)en;
}

/* ------------------------------------------------------------- endpoint API */

static bool ep_configure(tusb_desc_endpoint_t const *desc)
{
    uint8_t const ep_addr = desc->bEndpointAddress;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    uint16_t const mps = tu_edpt_packet_size(desc);
    uint8_t const type = desc->bmAttributes.xfer;

    if (ep_idx == 0 || ep_idx >= EP_COUNT || mps > 512) {
        return false; /* this controller has EP1-4 with 512-byte FIFOs */
    }

    ep_state_t *ep = (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) ? &ep_in[ep_idx] : &ep_out[ep_idx];
    ep->mps = mps;
    ep->type = type;
    ep->enable = true;
    ep->active = false;

    if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
        set_inep_mps(ep_idx, mps);
        set_mult(ep_idx, 0);
    } else {
        set_outep_mps(ep_idx, mps);
    }

    /* One 512-byte FIFO per endpoint number, F0-F3 for EP1-4 (the SDK
     * driver's non-ping-pong layout). */
    set_ep_fifomap(1, USB_FIFO_F0);
    set_ep_fifomap(2, USB_FIFO_F1);
    set_ep_fifomap(3, USB_FIFO_F2);
    set_ep_fifomap(4, USB_FIFO_F3);
    set_fifo_epmap(USB_FIFO_F0, 1, USB_FIFO_DIR_BID);
    set_fifo_epmap(USB_FIFO_F1, 2, USB_FIFO_DIR_BID);
    set_fifo_epmap(USB_FIFO_F2, 3, USB_FIFO_DIR_BID);
    set_fifo_epmap(USB_FIFO_F3, 4, USB_FIFO_DIR_BID);
    vdma_stop((uint8_t)(ep_idx - 1));
    reset_fifo((uint8_t)(ep_idx - 1));
    fifo_config((uint8_t)(ep_idx - 1), type, 512, 1, true);

    uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_ADR_OFFSET);
    regval |= USB_AFT_CONF;
    putreg32(regval, BFLB_USB_BASE + USB_DEV_ADR_OFFSET);
#if DS5_BL616_DIAG_LOG
    {
        static const char *const type_names[4] = {"control", "iso", "bulk", "interrupt"};
        bool const in = tu_edpt_dir(ep_addr) == TUSB_DIR_IN;
        uint8_t const fifo = (uint8_t)(ep_idx - 1);
        uint32_t const interval_us = port_speed() == TUSB_SPEED_HIGH ? (125u << (desc->bInterval ? desc->bInterval - 1 : 0))
                                                                    : 1000u * desc->bInterval;
        ev_push(false, EV_OPEN, ep_addr, mps, (uint32_t)type | ((uint32_t)desc->bInterval << 8));
        diag_logf("[usb] t=%lu open ep 0x%02x %s %s mps=%u bInterval=%u (%lu us at %s speed) -> FIFO F%u, 512 B x1 block;"
                  " regs: fcfg=0x%08lx fmap=0x%08lx epmap0=0x%08lx %smps%u=0x%08lx fibc%u=0x%08lx vdma_f%ups1=0x%08lx\r\n",
                  (unsigned long)bflb_mtimer_get_time_ms(), ep_addr, in ? "IN" : "OUT", type_names[type & 3u], mps,
                  desc->bInterval, (unsigned long)interval_us, speed_name(port_speed()), fifo,
                  (unsigned long)getreg32(BFLB_USB_BASE + USB_DEV_FCFG_OFFSET),
                  (unsigned long)getreg32(BFLB_USB_BASE + USB_DEV_FMAP_OFFSET),
                  (unsigned long)getreg32(BFLB_USB_BASE + USB_DEV_EPMAP0_OFFSET), in ? "in" : "out", ep_idx,
                  (unsigned long)getreg32(BFLB_USB_BASE + (in ? USB_DEV_INMPS1_OFFSET : USB_DEV_OUTMPS1_OFFSET) + (ep_idx - 1) * 4),
                  fifo, (unsigned long)getreg32(BFLB_USB_BASE + USB_DEV_FIBC0_OFFSET + 4 * fifo), fifo,
                  (unsigned long)getreg32(BFLB_USB_BASE + USB_VDMA_F0PS1_OFFSET + fifo * 8));
    }
#endif
    return true;
}

bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const *desc_ep)
{
    (void)rhport;
    return ep_configure(desc_ep);
}

bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size)
{
    (void)rhport;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    /* Every endpoint has its own fixed 512-byte FIFO; nothing to allocate. */
    return ep_idx != 0 && ep_idx < EP_COUNT && largest_packet_size <= 512;
}

bool dcd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const *desc_ep)
{
    (void)rhport;
    return ep_configure(desc_ep);
}

void dcd_edpt_close_all(uint8_t rhport)
{
    (void)rhport;
    DIAG(ev_push(false, EV_CLOSE_ALL, 0, 0, 0));
    for (uint8_t i = 1; i < EP_COUNT; i++) {
        ep_in[i].enable = false;
        ep_in[i].active = false;
        ep_out[i].enable = false;
        ep_out[i].active = false;
        if (g_started) {
            vdma_stop((uint8_t)(i - 1));
            reset_fifo((uint8_t)(i - 1));
        }
    }
}

bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes, bool is_isr)
{
    (void)rhport;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    bool const in = tu_edpt_dir(ep_addr) == TUSB_DIR_IN;

    if (!g_started || ep_idx >= EP_COUNT) {
        return false;
    }
    DIAG(if (in) { diag_cnt.usb_armed_in[ep_idx]++; } else { diag_cnt.usb_armed_out[ep_idx]++; });
    ep_state_t *ep = in ? &ep_in[ep_idx] : &ep_out[ep_idx];
    if (!ep->enable) {
        return false;
    }
    if (buffer == NULL && total_bytes != 0) {
        return false;
    }

    if (ep_idx == 0) {
        if (total_bytes == 0) {
            /* Status stage, either direction: the controller acknowledges it
             * when told the control transfer is done. */
            ep->active = false;
            control_transfer_done();
            DIAG(if (in) { diag_cnt.usb_done_in[0]++; } else { diag_cnt.usb_done_out[0]++; });
            dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, is_isr);
            return true;
        }
        uint16_t const n = total_bytes < ep->mps ? total_bytes : ep->mps;
        ep->buf = buffer;
        ep->len = n;
        ep->active = true;
        if (in && addr_is_cached(buffer)) {
            bflb_l1c_dcache_clean_range(buffer, n);
        }
        vdma_start(USB_FIFO_CXF, buffer, n, in);
        return true;
    }

    ep->buf = buffer;
    ep->len = total_bytes;
    ep->active = true;
    DIAG(diag_iso_trace(false, ep_addr, total_bytes, total_bytes));
    if (in) {
        if (total_bytes == 0) {
            send_zlp(ep_idx); /* completes on the TX0BYTE interrupt */
        } else {
            if (addr_is_cached(buffer)) {
                bflb_l1c_dcache_clean_range(buffer, total_bytes);
            }
            vdma_start((uint8_t)(ep_idx - 1), buffer, total_bytes, true);
        }
    } else {
        if (total_bytes == 0) {
            /* Wait for the host's zero-length packet (RX0BYTE). */
            return true;
        }
        if (addr_is_cached(buffer)) {
            bflb_l1c_dcache_clean_invalidate_range(buffer, total_bytes);
        }
        vdma_start((uint8_t)(ep_idx - 1), buffer, total_bytes, false);
    }
    return true;
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    uint32_t regval;

    if (!g_started) {
        return;
    }
    DIAG(ev_push(false, EV_STALL, ep_addr, 0, 0));
    if (ep_idx == 0) {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
        regval |= USB_CX_STL;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
    } else if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
        regval |= USB_STL_OEP1;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
    } else {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
        regval |= USB_STL_IEP1;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
    }
}

void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
    (void)rhport;
    uint8_t const ep_idx = tu_edpt_number(ep_addr);
    uint32_t regval;

    if (!g_started || ep_idx == 0) {
        return;
    }
    DIAG(ev_push(false, EV_CLEAR_STALL, ep_addr, 0, 0));
    if (tu_edpt_dir(ep_addr) == TUSB_DIR_OUT) {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
        regval &= ~USB_STL_OEP1;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_OUTMPS1_OFFSET + (ep_idx - 1) * 4);
    } else {
        regval = getreg32(BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
        regval &= ~USB_STL_IEP1;
        putreg32(regval, BFLB_USB_BASE + USB_DEV_INMPS1_OFFSET + (ep_idx - 1) * 4);
    }
}

#if DS5_BL616_DIAG_LOG
/* The SETUP requests that change the device's state are logged as they come;
 * SET_REPORT is only counted, by report ID. */
static void diag_usb_setup(const uint8_t *s)
{
    const uint32_t t = (uint32_t)bflb_mtimer_get_time_ms();
    const uint8_t type = s[0], req = s[1];
    const uint16_t value = (uint16_t)(s[2] | (s[3] << 8));
    const uint16_t index = (uint16_t)(s[4] | (s[5] << 8));
    diag_cnt.usb_setup++;
    diag_note_usb_isr(1, s);
    g_diag_suspended = false;
    ev_push(false, EV_SETUP, 0, (uint16_t)(type | (req << 8)), (uint32_t)value | ((uint32_t)index << 16));
    if (g_speed_after_reset_pending) {
        /* The bus reset interrupt may come before the high-speed chirp has
         * finished; by the first SETUP the speed is settled. TinyUSB keeps the
         * speed it was given at the reset. */
        g_speed_after_reset_pending = false;
        uint32_t const csr = getreg32(BFLB_USB_BASE + USB_OTG_CSR_OFFSET);
        uint32_t const ctl = getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET);
        diag_logf("[usb] t=%lu first SETUP after the reset: speed now %s (otg_csr=0x%08lx spd_typ=%lu, dev_ctl=0x%08lx hs_en=%lu)\r\n",
                  (unsigned long)t, speed_name(port_speed()), (unsigned long)csr,
                  (unsigned long)((csr & USB_SPD_TYP_HOV_POV_MASK) >> USB_SPD_TYP_HOV_POV_SHIFT),
                  (unsigned long)ctl, (unsigned long)((ctl & USB_HS_EN_HOV) ? 1u : 0u));
    }
    if (type == 0x00 && req == 0x05) {
        diag_logf("[usb] t=%lu SET_ADDRESS %u\r\n", (unsigned long)t, value);
    } else if (type == 0x00 && req == 0x09) {
        diag_logf("[usb] t=%lu SET_CONFIGURATION %u\r\n", (unsigned long)t, value);
        g_diag_configured = value != 0;
        g_diag_last_irq_ms = t;
        diag_cnt.stream_alt[1] = 0;
        diag_cnt.stream_alt[2] = 0;
    } else if (type == 0x01 && req == 0x0B) {
        diag_logf("[usb] t=%lu SET_INTERFACE itf %u alt %u\r\n", (unsigned long)t, index, value);
        if (index == 1 || index == 2) {
            diag_note_stream_alt((uint8_t)index, (uint8_t)value);
            if (value != 0) {
                /* A stream opens: trace EP1/EP2 for the next 200 ms. */
                g_diag_win_until_ms = t + DIAG_WIN_MS;
                g_diag_win_lines = 0;
                g_diag_win_first[1] = DIAG_WIN_FIRST;
                g_diag_win_first[2] = DIAG_WIN_FIRST;
                diag_logf("[usb] t=%lu %s stream opening: tracing ep 0x01/0x82 transfers for %u ms (at most %u lines);"
                          " TinyUSB opens the endpoint when it handles this request\r\n",
                          (unsigned long)t, index == 1 ? "speaker" : "microphone", (unsigned)DIAG_WIN_MS,
                          (unsigned)DIAG_WIN_MAX_LINES);
            }
        }
    } else if (type == 0x21 && req == 0x09) {
        diag_cnt.usb_set_report[value & 0xFF]++;
    } else if ((type & 0x7F) == 0x22) {
        /* Audio class request to an endpoint (UAC1 sampling frequency). */
        diag_logf("[usb] t=%lu class request to endpoint 0x%02x: bmRequestType 0x%02x bRequest 0x%02x wValue 0x%04x wLength %u\r\n",
                  (unsigned long)t, index & 0xFF, type, req, value, (unsigned)(s[6] | (s[7] << 8)));
    } else if ((type & 0x7F) == 0x21 && (index & 0xFF) != 3) {
        /* Audio class request to an interface or entity (mute, volume),
         * rate-limited: the HID interface (3) is counted above instead. */
        static uint32_t window_ms, lines;
        if (t - window_ms >= 5000u) {
            window_ms = t;
            lines = 0;
        }
        if (lines++ < 12u) {
            diag_logf("[usb] t=%lu class request to itf/entity 0x%04x: bmRequestType 0x%02x bRequest 0x%02x wValue 0x%04x wLength %u\r\n",
                      (unsigned long)t, index, type, req, value, (unsigned)(s[6] | (s[7] << 8)));
        }
    }
}

/* Read only: the error bits of the interrupt groups, masked ones included
 * (none of them is enabled, so none is ever cleared). */
void bl616_dcd_diag_status(uint32_t *isg0, uint32_t *isg2, uint32_t *isg3, int *started, int *connected)
{
    *started = g_started ? 1 : 0;
    *connected = g_want_connected ? 1 : 0;
    if (!g_started) {
        *isg0 = *isg2 = *isg3 = 0;
        return;
    }
    *isg0 = getreg32(BFLB_USB_BASE + USB_DEV_ISG0_OFFSET) & (USB_CX_COMFAIL_INT | USB_CX_COMABT_INT);
    *isg2 = getreg32(BFLB_USB_BASE + USB_DEV_ISG2_OFFSET) & (USB_ISO_SEQ_ERR_INT | USB_ISO_SEQ_ABORT_INT | USB_DMA_ERROR_HOV);
    *isg3 = getreg32(BFLB_USB_BASE + USB_DEV_ISG3_OFFSET) &
            (USB_VDMA_ERROR_CXF | USB_VDMA_ERROR_F0 | USB_VDMA_ERROR_F1 | USB_VDMA_ERROR_F2 | USB_VDMA_ERROR_F3);
}

/* Totals since boot: bounded waits that timed out, and SETUPs that found a
 * control VDMA still armed. */
void bl616_dcd_diag_waits(uint32_t *timeouts, uint32_t *cx_preempted)
{
    *timeouts = g_wait_timeouts;
    *cx_preempted = g_cx_vdma_preempted;
}

/* Variant P. Called by the monitor task every 250 ms. The host is "talking"
 * while USB device interrupts keep coming (EP4's input reports alone complete
 * about 250 times a second at high speed) or the SOF frame number keeps
 * changing. The frame number counts only if it has been seen to change at least
 * once since the configuration: a register that never moves would otherwise
 * read as silence. Returns 0, or the reset reason with why filled in. */
#define DIAG_SILENT_MS 2000u

uint32_t bl616_dcd_diag_check_silence(uint32_t now, char *why, uint32_t cap)
{
    if (!g_started) {
        return 0;
    }
    if (!g_want_connected) {
        /* The firmware took the device off the bus itself (tud_disconnect). */
        uint32_t const since = g_diag_disc_ms;
        if (since != 0 && g_diag_disc_while_configured && (uint32_t)(now - since) >= DIAG_SILENT_MS) {
            snprintf(why, cap,
                     "the firmware disconnected USB itself (D+ pull-up off) %lu ms ago while configured, caller 0x%08lx,"
                     " and has not reconnected; audio alts itf1=%u itf2=%u, last alt change %lu ms ago",
                     (unsigned long)(now - since), (unsigned long)diag_cnt.usb_disc_caller,
                     (unsigned)diag_cnt.stream_alt[1], (unsigned)diag_cnt.stream_alt[2],
                     (unsigned long)(diag_cnt.stream_alt_ms ? now - diag_cnt.stream_alt_ms : 0));
            return DIAG_RESET_USB_SILENT;
        }
        return 0;
    }
    if (!g_diag_configured) {
        g_diag_sfn_sampled = false;
        g_diag_sfn_moving = false;
        g_diag_suspend_noted = false;
        return 0;
    }
    uint32_t const sfn = getreg32(BFLB_USB_BASE + USB_DEV_SFN_OFFSET) & (USB_SOFN_MASK | USB_USOFN_MASK);
    if (!g_diag_sfn_sampled) {
        g_diag_sfn_sampled = true;
        g_diag_sfn_last = sfn;
        g_diag_sfn_change_ms = now;
    } else if (sfn != g_diag_sfn_last) {
        g_diag_sfn_last = sfn;
        g_diag_sfn_change_ms = now;
        g_diag_sfn_moving = true;
    }
    uint32_t const irq_age = now - g_diag_last_irq_ms;
    uint32_t const sof_age = now - g_diag_sfn_change_ms;
    bool silent;
    const char *basis;
    if (g_diag_sfn_moving) {
        silent = irq_age >= DIAG_SILENT_MS && sof_age >= DIAG_SILENT_MS;
        basis = "no USB interrupt and no SOF frame-number change";
    } else {
        /* The frame number has not been seen moving (could not look): the
         * interrupts alone decide, except in keyboard-only mode, where nothing
         * completes until a key is sent. */
        silent = irq_age >= DIAG_SILENT_MS && !usb_keyboard_only;
        basis = "no USB interrupt (the SOF frame number was never seen moving, so it was not used)";
    }
    if (!silent) {
        g_diag_suspend_noted = false;
        return 0;
    }
    bool const stream_open = diag_cnt.stream_alt[1] != 0 || diag_cnt.stream_alt[2] != 0;
    if (g_diag_suspended && !stream_open) {
        if (!g_diag_suspend_noted) {
            g_diag_suspend_noted = true;
            diag_logf("[usb] t=%lu silent %lu ms while suspended with no audio stream open: a host suspend, not reset\r\n",
                      (unsigned long)now, (unsigned long)irq_age);
        }
        return 0;
    }
    snprintf(why, cap,
             "%s for %lu ms (last interrupt %lu ms ago, frame number 0x%04lx unchanged %lu ms) while configured%s;"
             " audio alts itf1=%u itf2=%u, last alt change %lu ms ago",
             basis, (unsigned long)(irq_age < sof_age || !g_diag_sfn_moving ? irq_age : sof_age),
             (unsigned long)irq_age, (unsigned long)sfn, (unsigned long)sof_age,
             g_diag_suspended ? ", suspended WITH a stream open" : "", (unsigned)diag_cnt.stream_alt[1],
             (unsigned)diag_cnt.stream_alt[2],
             (unsigned long)(diag_cnt.stream_alt_ms ? now - diag_cnt.stream_alt_ms : 0));
    return DIAG_RESET_USB_SILENT;
}

static void dump_events(const char *title, const struct diag_usb_evt *ring, uint32_t count)
{
    uint32_t const n = count < DIAG_EVT_N ? count : DIAG_EVT_N;
    diag_logf("[usbdump] %s: last %lu of %lu\r\n", title, (unsigned long)n, (unsigned long)count);
    char line[200];
    size_t len = 0;
    line[0] = '\0';
    for (uint32_t i = count - n; i != count; i++) {
        const struct diag_usb_evt *e = &ring[i % DIAG_EVT_N];
        char tok[64];
        int k;
        if (e->kind == EV_SETUP) {
            k = snprintf(tok, sizeof(tok), " %lu:SETUP %02x/%02x v%04lx i%04lx", (unsigned long)e->t_ms,
                         e->a & 0xFF, e->a >> 8, (unsigned long)(e->b & 0xFFFF), (unsigned long)(e->b >> 16));
        } else {
            k = snprintf(tok, sizeof(tok), " %lu:%s ep%02x a=%u b=0x%lx", (unsigned long)e->t_ms, ev_name(e->kind),
                         e->ep, e->a, (unsigned long)e->b);
        }
        if (k <= 0) {
            continue;
        }
        if (len + (size_t)k + 3 >= sizeof(line)) {
            diag_logf("[usbdump]%s\r\n", line);
            len = 0;
            line[0] = '\0';
        }
        memcpy(line + len, tok, (size_t)k + 1);
        len += (size_t)k;
    }
    if (len != 0) {
        diag_logf("[usbdump]%s\r\n", line);
    }
}

#define R(off) ((unsigned long)getreg32(BFLB_USB_BASE + (off)))

void bl616_dcd_diag_dump(const char *why)
{
    uint32_t const now = (uint32_t)bflb_mtimer_get_time_ms();
    diag_logf("[usbdump] t=%lu USB state (%s)\r\n", (unsigned long)now, why != NULL ? why : "");
    diag_logf("[usbdump] started=%d want_connected=%d int_enabled=%d configured=%d suspended=%d speed=%s"
              " last_irq=%lu ms ago disc_at=%lu ms (while configured %d) disc_caller=0x%08lx\r\n",
              g_started, g_want_connected, g_int_enabled, g_diag_configured, g_diag_suspended,
              g_started ? speed_name(port_speed()) : "-", (unsigned long)(now - g_diag_last_irq_ms),
              (unsigned long)g_diag_disc_ms, g_diag_disc_while_configured, (unsigned long)diag_cnt.usb_disc_caller);
    diag_logf("[usbdump] sof_moving=%d sfn_last=0x%04lx changed %lu ms ago wait_timeouts=%lu setup_preempted=%lu\r\n",
              g_diag_sfn_moving, (unsigned long)g_diag_sfn_last, (unsigned long)(now - g_diag_sfn_change_ms),
              (unsigned long)g_wait_timeouts, (unsigned long)g_cx_vdma_preempted);
    for (int i = 0; i < EP_COUNT; i++) {
        diag_logf("[usbdump] ep%d in: mps=%u type=%u enable=%d active=%d len=%u buf=%p | out: mps=%u type=%u enable=%d"
                  " active=%d len=%u buf=%p\r\n",
                  i, ep_in[i].mps, ep_in[i].type, ep_in[i].enable, ep_in[i].active, ep_in[i].len, (void *)ep_in[i].buf,
                  ep_out[i].mps, ep_out[i].type, ep_out[i].enable, ep_out[i].active, ep_out[i].len,
                  (void *)ep_out[i].buf);
    }
    if (!g_started) {
        diag_logf("[usbdump] controller not started: registers not read\r\n");
    } else {
        diag_logf("[usbdump] otg_csr=0x%08lx otg_isr=0x%08lx glb_isr=0x%08lx glb_int=0x%08lx dev_ctl=0x%08lx dev_adr=0x%08lx"
                  " dev_tst=0x%08lx dev_sfn=0x%08lx dev_smt=0x%08lx phy_tst=0x%08lx (unplug=%lu) hcmisc=0x%08lx\r\n",
                  R(USB_OTG_CSR_OFFSET), R(USB_OTG_ISR_OFFSET), R(USB_GLB_ISR_OFFSET), R(USB_GLB_INT_OFFSET),
                  R(USB_DEV_CTL_OFFSET), R(USB_DEV_ADR_OFFSET), R(USB_DEV_TST_OFFSET), R(USB_DEV_SFN_OFFSET),
                  R(USB_DEV_SMT_OFFSET), R(USB_PHY_TST_OFFSET), R(USB_PHY_TST_OFFSET) & USB_UNPLUG,
                  R(USB_HCMISC_OFFSET));
        diag_logf("[usbdump] igr=0x%08lx migr=0x%08lx isg0=0x%08lx/m0x%08lx isg1=0x%08lx/m0x%08lx isg2=0x%08lx/m0x%08lx\r\n",
                  R(USB_DEV_IGR_OFFSET), R(USB_DEV_MIGR_OFFSET), R(USB_DEV_ISG0_OFFSET), R(USB_DEV_MISG0_OFFSET),
                  R(USB_DEV_ISG1_OFFSET), R(USB_DEV_MISG1_OFFSET), R(USB_DEV_ISG2_OFFSET), R(USB_DEV_MISG2_OFFSET));
        diag_logf("[usbdump] isg3=0x%08lx/m0x%08lx isg4=0x%08lx rxz=0x%08lx txz=0x%08lx ise=0x%08lx cxcfe=0x%08lx\r\n",
                  R(USB_DEV_ISG3_OFFSET), R(USB_DEV_MISG3_OFFSET), R(USB_DEV_ISG4_OFFSET), R(USB_DEV_RXZ_OFFSET),
                  R(USB_DEV_TXZ_OFFSET), R(USB_DEV_ISE_OFFSET), R(USB_DEV_CXCFE_OFFSET));
        diag_logf("[usbdump] epmap0=0x%08lx fmap=0x%08lx fcfg=0x%08lx fibc0-3=0x%08lx 0x%08lx 0x%08lx 0x%08lx\r\n",
                  R(USB_DEV_EPMAP0_OFFSET), R(USB_DEV_FMAP_OFFSET), R(USB_DEV_FCFG_OFFSET), R(USB_DEV_FIBC0_OFFSET),
                  R(USB_DEV_FIBC1_OFFSET), R(USB_DEV_FIBC2_OFFSET), R(USB_DEV_FIBC3_OFFSET));
        diag_logf("[usbdump] inmps1-4=0x%08lx 0x%08lx 0x%08lx 0x%08lx outmps1-4=0x%08lx 0x%08lx 0x%08lx 0x%08lx\r\n",
                  R(USB_DEV_INMPS1_OFFSET), R(USB_DEV_INMPS2_OFFSET), R(USB_DEV_INMPS3_OFFSET),
                  R(USB_DEV_INMPS4_OFFSET), R(USB_DEV_OUTMPS1_OFFSET), R(USB_DEV_OUTMPS2_OFFSET),
                  R(USB_DEV_OUTMPS3_OFFSET), R(USB_DEV_OUTMPS4_OFFSET));
        diag_logf("[usbdump] vdma_ctrl=0x%08lx cxf ps1=0x%08lx ps2=0x%08lx | f0 0x%08lx 0x%08lx | f1 0x%08lx 0x%08lx"
                  " | f2 0x%08lx 0x%08lx | f3 0x%08lx 0x%08lx\r\n",
                  R(USB_VDMA_CTRL_OFFSET), R(USB_VDMA_CXFPS1_OFFSET), R(USB_VDMA_CXFPS2_OFFSET),
                  R(USB_VDMA_F0PS1_OFFSET), R(USB_VDMA_F0PS2_OFFSET), R(USB_VDMA_F1PS1_OFFSET),
                  R(USB_VDMA_F1PS2_OFFSET), R(USB_VDMA_F2PS1_OFFSET), R(USB_VDMA_F2PS2_OFFSET),
                  R(USB_VDMA_F3PS1_OFFSET), R(USB_VDMA_F3PS2_OFFSET));
    }
    diag_logf("[usbdump] totals: setup=%lu resets=%lu suspends=%lu resumes=%lu done/armed in0 %lu/%lu out0 %lu/%lu"
              " out1 %lu/%lu in2 %lu/%lu out3 %lu/%lu in4 %lu/%lu iso_out short=%lu zero=%lu\r\n",
              (unsigned long)diag_cnt.usb_setup, (unsigned long)diag_cnt.usb_bus_reset,
              (unsigned long)diag_cnt.usb_suspend, (unsigned long)diag_cnt.usb_resume,
              (unsigned long)diag_cnt.usb_done_in[0], (unsigned long)diag_cnt.usb_armed_in[0],
              (unsigned long)diag_cnt.usb_done_out[0], (unsigned long)diag_cnt.usb_armed_out[0],
              (unsigned long)diag_cnt.usb_done_out[1], (unsigned long)diag_cnt.usb_armed_out[1],
              (unsigned long)diag_cnt.usb_done_in[2], (unsigned long)diag_cnt.usb_armed_in[2],
              (unsigned long)diag_cnt.usb_done_out[3], (unsigned long)diag_cnt.usb_armed_out[3],
              (unsigned long)diag_cnt.usb_done_in[4], (unsigned long)diag_cnt.usb_armed_in[4],
              (unsigned long)diag_cnt.usb_iso_out_short, (unsigned long)diag_cnt.usb_iso_out_zero);
    /* Copies, so the rings are not read while the interrupt writes them. */
    static struct diag_usb_evt copy[DIAG_EVT_N];
    uintptr_t flags = bflb_irq_save();
    uint32_t const nc = g_ev_ctl_n;
    memcpy(copy, g_ev_ctl, sizeof(copy));
    bflb_irq_restore(flags);
    dump_events("bus/control events (time ms:event)", copy, nc);
    flags = bflb_irq_save();
    uint32_t const nd = g_ev_dat_n;
    memcpy(copy, g_ev_dat, sizeof(copy));
    bflb_irq_restore(flags);
    dump_events("ep1/ep2 data events (ARM a=len, DONE a=len b=armed len)", copy, nd);
}

#undef R
#endif

/* ---------------------------------------------------------------- interrupt */

static void complete_out(uint8_t ep_addr, ep_state_t *ep, uint32_t len)
{
    ep->active = false;
    if (len != 0 && addr_is_cached(ep->buf)) {
        bflb_l1c_dcache_invalidate_range(ep->buf, len);
    }
    DIAG(diag_cnt.usb_done_out[ep_addr & 0x0F]++;
         if ((ep_addr & 0x0F) == 3 && len != 0 && ep->buf != NULL) { diag_cnt.usb_out_report[ep->buf[0]]++; }
         if ((ep_addr & 0x0F) == 1) {
             if (len < ep->len) { diag_cnt.usb_iso_out_short++; }
             if (len < diag_cnt.usb_iso_out_min) { diag_cnt.usb_iso_out_min = len; }
             if (len > diag_cnt.usb_iso_out_max) { diag_cnt.usb_iso_out_max = len; }
         }
         diag_iso_trace(true, ep_addr, len, ep->len));
    dcd_event_xfer_complete(0, ep_addr, len, XFER_RESULT_SUCCESS, true);
}

void dcd_int_handler(uint8_t rhport)
{
    (void)rhport;
    uint32_t const glb_intstatus = getreg32(BFLB_USB_BASE + USB_GLB_ISR_OFFSET);
    if (!(glb_intstatus & USB_DEV_INT)) {
        return;
    }
    uint32_t const dev_intstatus = getreg32(BFLB_USB_BASE + USB_DEV_IGR_OFFSET);
    uint32_t status;
    DIAG(g_diag_last_irq_ms = (uint32_t)bflb_mtimer_get_time_ms());

    if (dev_intstatus & USB_INT_G0) {
        status = group_intstatus(0);
        if (status & USB_CX_SETUP_INT) {
            /* A new SETUP supersedes whatever EP0 was doing. A data-stage
             * VDMA the host abandoned may still be armed on the control
             * channel: stop it before the channel is reprogrammed for the
             * SETUP read, rather than rewriting a running channel. */
            ep_in[0].active = false;
            ep_out[0].active = false;
            if (getreg32(BFLB_USB_BASE + USB_VDMA_CXFPS1_OFFSET) & USB_VDMA_START_CXF) {
                g_cx_vdma_preempted++;
                vdma_stop(USB_FIFO_CXF);
                DIAG(diag_logf("[usb] t=%lu SETUP while a control VDMA was armed (cxfps1=0x%08lx); stopped it, %lu so far\r\n",
                               (unsigned long)bflb_mtimer_get_time_ms(),
                               (unsigned long)getreg32(BFLB_USB_BASE + USB_VDMA_CXFPS1_OFFSET),
                               (unsigned long)g_cx_vdma_preempted));
            }
            vdma_start(USB_FIFO_CXF, setup_buf, 8, false);
            if (wait_reg_clear(BFLB_USB_BASE + USB_VDMA_CXFPS1_OFFSET, USB_VDMA_START_CXF,
                               DCD_SETUP_READ_TIMEOUT_US, "the SETUP read (VDMA_CXFPS1 START)")) {
                group_int_clear(3, USB_VDMA_CMPLT_CXF);
                DIAG(diag_usb_setup(setup_buf));
                dcd_event_setup_received(0, setup_buf, true);
            } else {
                /* The SETUP was not read. Stop the VDMA, empty the control
                 * FIFO and stall EP0: the host retries the request, which
                 * TinyUSB then sees as a fresh SETUP. */
                vdma_stop(USB_FIFO_CXF);
                reset_fifo(USB_FIFO_CXF);
                group_int_clear(3, USB_VDMA_CMPLT_CXF);
                DIAG(ev_push(false, EV_SETUP_TIMEOUT, 0, 0, g_wait_timeouts));
                uint32_t const cx = getreg32(BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
                putreg32(cx | USB_CX_STL, BFLB_USB_BASE + USB_DEV_CXCFE_OFFSET);
            }
        }
    }

    if (dev_intstatus & USB_INT_G2) {
        status = group_intstatus(2);

        if (status & USB_SUSP_INT) {
            group_int_clear(2, USB_SUSP_INT);
            DIAG(diag_cnt.usb_suspend++; g_diag_suspended = true; ev_push(false, EV_SUSPEND, 0, 0, 0);
                 diag_logf("[usb] t=%lu suspend\r\n", (unsigned long)bflb_mtimer_get_time_ms()));
            /* Unlike the SDK driver, the FIFOs are NOT reset here: TinyUSB
             * keeps an endpoint busy until its transfer completes, so a report
             * dropped on suspend would leave that endpoint stuck after resume.
             * A transfer in flight simply finishes once the host resumes. */
            dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
        }
        if (status & USB_RESM_INT) {
            group_int_clear(2, USB_RESM_INT);
            DIAG(diag_cnt.usb_resume++; g_diag_suspended = false; ev_push(false, EV_RESUME, 0, 0, 0);
                 diag_logf("[usb] t=%lu resume\r\n", (unsigned long)bflb_mtimer_get_time_ms()));
            dcd_event_bus_signal(0, DCD_EVENT_RESUME, true);
        }
        if (status & USB_TX0BYTE_INT) {
            uint32_t const txz = getreg32(BFLB_USB_BASE + USB_DEV_TXZ_OFFSET);
            for (uint8_t i = 1; i < EP_COUNT; i++) {
                if (txz & (1u << (i - 1))) {
                    putreg32((1u << (i - 1)), BFLB_USB_BASE + USB_DEV_TXZ_OFFSET);
                    ep_in[i].active = false;
                    DIAG(diag_cnt.usb_done_in[i]++; diag_iso_trace(true, (uint8_t)(i | TUSB_DIR_IN_MASK), 0, 0));
                    dcd_event_xfer_complete(0, (uint8_t)(i | TUSB_DIR_IN_MASK), 0, XFER_RESULT_SUCCESS, true);
                }
            }
            group_int_clear(2, USB_TX0BYTE_INT);
        }
        if (status & USB_RX0BYTE_INT) {
            uint32_t const rxz = getreg32(BFLB_USB_BASE + USB_DEV_RXZ_OFFSET);
            for (uint8_t i = 1; i < EP_COUNT; i++) {
                if (rxz & (1u << (i - 1))) {
                    putreg32((1u << (i - 1)), BFLB_USB_BASE + USB_DEV_RXZ_OFFSET);
                    DIAG(ev_push(true, EV_ZLP_RX, i, ep_out[i].active ? 1 : 0, ep_out[i].type);
                         if (i == 1) { diag_cnt.usb_iso_out_zero++; });
                    if (!ep_out[i].active) {
                        continue;
                    }
                    if (ep_out[i].type == TUSB_XFER_ISOCHRONOUS) {
                        /* An empty isochronous packet carries no samples. Keep
                         * the transfer armed for the next packet: completing it
                         * with 0 bytes made TinyUSB's audio driver stop
                         * re-arming the endpoint (audiod_rx_xfer_isr returns
                         * before its next usbd_edpt_xfer when it writes 0
                         * bytes), and left this VDMA running underneath a
                         * transfer TinyUSB considered finished. */
                        continue;
                    }
                    /* Stop the VDMA armed for this transfer before it is
                     * reported done, so TinyUSB's next transfer does not
                     * reprogram a channel that is still running. */
                    vdma_stop((uint8_t)(i - 1));
                    complete_out(i, &ep_out[i], 0);
                }
            }
            group_int_clear(2, USB_RX0BYTE_INT);
        }
        if (status & USBRST_INT) {
            group_int_clear(2, USBRST_INT);
            for (uint8_t fifo = 0; fifo < 4; fifo++) {
                vdma_stop(fifo);
            }
            reset_all_fifos();

            uint32_t regval = getreg32(BFLB_USB_BASE + USB_DEV_SMT_OFFSET);
            regval &= ~USB_SOFMT_MASK;
            regval |= USB_SOF_TIMER_MASK_AFTER_RESET_HS;
            putreg32(regval, BFLB_USB_BASE + USB_DEV_SMT_OFFSET);

            ep0_reset_state();
            tusb_speed_t const speed = port_speed();
            DIAG(diag_cnt.usb_bus_reset++; g_speed_after_reset_pending = true; g_diag_configured = false;
                 g_diag_suspended = false; diag_cnt.stream_alt[1] = 0; diag_cnt.stream_alt[2] = 0;
                 ev_push(false, EV_BUS_RESET, 0, (uint16_t)speed, 0);
                 diag_logf("[usb] t=%lu bus reset, %s speed told to TinyUSB (otg_csr=0x%08lx spd_typ=%lu, dev_ctl=0x%08lx hs_en=%lu force_fs=%lu)\r\n",
                           (unsigned long)bflb_mtimer_get_time_ms(), speed_name(speed),
                           (unsigned long)getreg32(BFLB_USB_BASE + USB_OTG_CSR_OFFSET),
                           (unsigned long)((getreg32(BFLB_USB_BASE + USB_OTG_CSR_OFFSET) & USB_SPD_TYP_HOV_POV_MASK) >> USB_SPD_TYP_HOV_POV_SHIFT),
                           (unsigned long)getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET),
                           (unsigned long)((getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET) & USB_HS_EN_HOV) ? 1u : 0u),
                           (unsigned long)((getreg32(BFLB_USB_BASE + USB_DEV_CTL_OFFSET) & USB_FORCE_FS) ? 1u : 0u)));
            dcd_event_bus_reset(0, speed, true);
        }
    }

    if (dev_intstatus & USB_INT_G3) {
        status = group_intstatus(3);
        group_int_clear(3, status);

        if (status & USB_VDMA_CMPLT_CXF) {
            if (ep_in[0].active) {
                ep_in[0].active = false;
                uint32_t const n = ep_in[0].len - vdma_remain(USB_FIFO_CXF);
                DIAG(diag_cnt.usb_done_in[0]++);
                dcd_event_xfer_complete(0, TUSB_DIR_IN_MASK, n, XFER_RESULT_SUCCESS, true);
            } else if (ep_out[0].active) {
                uint32_t const n = ep_out[0].len - vdma_remain(USB_FIFO_CXF);
                complete_out(0x00, &ep_out[0], n);
            }
        }

        for (uint8_t fifo = 0; fifo < 4; fifo++) {
            if (status & (1u << (fifo + 1))) {
                uint8_t const ep_idx = get_fifo_ep(fifo);
                if (ep_idx == 0 || ep_idx >= EP_COUNT) {
                    continue;
                }
                if (ep_in[ep_idx].active) {
                    ep_in[ep_idx].active = false;
                    uint32_t const n = ep_in[ep_idx].len - vdma_remain(fifo);
                    DIAG(diag_cnt.usb_done_in[ep_idx]++;
                         diag_iso_trace(true, (uint8_t)(ep_idx | TUSB_DIR_IN_MASK), n, ep_in[ep_idx].len));
                    dcd_event_xfer_complete(0, (uint8_t)(ep_idx | TUSB_DIR_IN_MASK), n, XFER_RESULT_SUCCESS, true);
                } else if (ep_out[ep_idx].active) {
                    uint32_t const n = ep_out[ep_idx].len - vdma_remain(fifo);
                    complete_out(ep_idx, &ep_out[ep_idx], n);
                }
            }
        }
    }
}

#endif /* CFG_TUD_ENABLED */
