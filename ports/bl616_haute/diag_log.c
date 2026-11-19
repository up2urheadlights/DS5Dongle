/*
 * Debug-build log for the BL616 Haute target, for a board whose UART pins are
 * not reachable. None of this is in the production image (CMakeLists.txt adds
 * the file only with DS5_BL616_DIAG_LOG=1).
 *
 * Every console byte is copied into a RAM ring by wrapping the SDK's UART
 * output calls at link time (-Wl,--wrap, see CMakeLists.txt), so printf from
 * any task -- the SDK banner before main() included -- lands in the ring. The
 * diagnostics added for the stage 2 hang write to the ring only (diag_logf),
 * which is much faster than the 115200-baud console and safe in interrupts.
 *
 * The ring is read over USB without an endpoint of its own: the BL616 has
 * four, and the DualSense uses all of them. The debug build adds a vendor
 * collection with a 64-byte feature report (DIAG_LOG_REPORT_ID) to the
 * DualSense HID interface, and each GET_REPORT of it returns the next bytes
 * (src/port/bl616/usb_hid_bl616.cpp). A host tool polls it
 * (capture-hid-log.ps1 on the share).
 *
 * THE LOG SURVIVES A RESET. The ring, its write position and a few crumbs
 * (where the main loop was, the last Bluetooth packet in and out, whether the
 * USB interrupt was running, why the firmware reset itself) live in
 * .noinit_data, a section the SDK's linker script places in ram_noinit, which
 * start_load() never zeroes (it zeroes only ram_bss and wifi_bss:
 * drivers/soc/bl616/std/startup/start_load.c). A watchdog or software reset
 * keeps the RAM powered, so the next boot finds the previous log. A header
 * magic, its complement, the ring size and the write position's complement
 * must all match before the old content is trusted; anything else (power-on,
 * a different firmware) starts an empty ring. Bytes that are not printable
 * text are replaced by '?' at boot, so a partly overwritten ring still reads.
 *
 * The ring is accessed through the non-cacheable alias of its address
 * (0x6xxxxxxx -> 0x2xxxxxxx, the same RAM): a write-back data cache would
 * otherwise hold the newest bytes in lines that a reset discards.
 *
 * What else could overwrite it between boots is boot2, which runs from flash
 * but keeps its own data in RAM. Its linker script in this SDK
 * (examples/boot2_isp/port/bl616/bflb_sp_boot2_iap_flash.ld) DECLARES
 * 0x62FC0000-0x62FF0000, and this section starts below 0x62FF0000. But the
 * shipped boot2 (boot2_bl616_isp_release_v8.1.8.bin) forms no address above
 * page 0x62FE7000 (every lui in it was scanned), so it touches nothing past
 * 0x62FE8800 unless one of its buffers runs past its own page; hence
 * BOOT2_RAM_TOUCHED, which the boot line checks. Aligning the section above
 * 0x62FF0000 instead was tried: the linker then aligns all of ram_noinit
 * and about 80 KB of heap is lost. The ROM's own RAM use is not documented:
 * the header check covers it.
 *
 * A Bluetooth link key is never logged. bt.cpp prints the stored key on a
 * Link Key Request ("[HCI] Link key: <hex>"); the console copy into the ring
 * replaces those hex digits with '*' (the UART itself is not reachable on
 * this board). The HCI event log below prints only the code of Link Key
 * Request / Notification and Return Link Keys events.
 */
#include "diag_log.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bflb_irq.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "bl616_glb.h"
#include "FreeRTOS.h"
#include "mm.h"
#include "task.h"

struct diag_counters diag_cnt;

/* ------------------------------------------------------------ persistence */

#define DIAG_RING_SIZE  65536U /* power of two */
#define DIAG_RING_MASK  (DIAG_RING_SIZE - 1U)
#define DIAG_MAGIC      0x44474C32U /* "DGL2" */
#define BOOT2_RAM_TOUCHED 0x62FE8800U /* see the file comment */

struct diag_crumbs {
    uint32_t main_where;
    uint32_t main_passes;
    uint32_t main_last_ms;
    uint32_t monitor_ms;
    uint32_t usb_isr_inside;
    uint32_t usb_isr_count;
    uint32_t usb_last_setup[2];
    uint32_t bt_last_rx; /* H4 type << 8 | event code (ACL: 0) */
    uint32_t bt_last_tx; /* H4 type << 16 | opcode (command) or handle (ACL) */
    uint32_t reset_reason;
    uint32_t reset_ms;
};
#define DIAG_CRUMB_WORDS (sizeof(struct diag_crumbs) / sizeof(uint32_t))

struct diag_persist {
    uint32_t magic;
    uint32_t magic_inv;
    uint32_t ring_size;
    uint32_t boot_count;
    uint32_t head;     /* total bytes ever written (wraps at 2^32) */
    uint32_t head_inv; /* ~head, written after head */
    struct diag_crumbs crumbs;
    uint8_t ring[DIAG_RING_SIZE];
};

/* Lands right after the 4 KB startup stack at the start of ram_noinit
 * (0x62FEB840 in the build this was written for). See BOOT2_RAM_TOUCHED. */
static struct diag_persist g_persist_storage __attribute__((section(".noinit_data.ds5_diag"), aligned(32)));

/* Both set by diag_init_locked() on the first log byte of a boot (.bss, so
 * zero until then). */
static volatile struct diag_persist *g_p;
static bool g_inited;
static uint32_t g_tail; /* next byte the host reads */

static inline uint32_t now_ms(void)
{
    return (uint32_t)bflb_mtimer_get_time_ms();
}

static void put_raw_locked(const uint8_t *data, uint32_t len)
{
    volatile struct diag_persist *p = g_p;
    uint32_t head = p->head;
    for (uint32_t i = 0; i < len; i++) {
        p->ring[(head + i) & DIAG_RING_MASK] = data[i];
    }
    head += len;
    p->head = head;
    p->head_inv = ~head;
}

static bool printable(uint8_t c)
{
    return c == '\r' || c == '\n' || c == '\t' || c == 0x1B || (c >= 0x20 && c < 0x7F);
}

static const char *reset_reason_name(uint32_t r)
{
    switch (r & 0xFFu) {
        case DIAG_RESET_NONE:   return "none recorded (watchdog, power or external reset)";
        case DIAG_RESET_FAULT:  return "CPU exception (see the FAULT line above)";
        case DIAG_RESET_STACK:  return "FreeRTOS stack overflow";
        case DIAG_RESET_ASSERT: return "FreeRTOS assert";
        default:                return "unknown code";
    }
}

static void diag_init_locked(void)
{
    g_inited = true;
    const uintptr_t addr = (uintptr_t)&g_persist_storage;
    /* Non-cacheable alias of the same RAM (see the file comment). */
    g_p = ((addr >> 28) == 0x6U) ? (volatile struct diag_persist *)(addr & 0x2FFFFFFFU)
                                  : (volatile struct diag_persist *)addr;
    volatile struct diag_persist *p = g_p;

    GLB_RESET_RECORD_Type rec;
    memset(&rec, 0, sizeof(rec));
    GLB_Get_Reset_Reason(&rec);

    char line[768]; /* runs on the 4 KB startup stack (board_init's first printf) */
    const uint32_t magic_seen = p->magic;
    const bool valid = magic_seen == DIAG_MAGIC && p->magic_inv == ~DIAG_MAGIC &&
                       p->ring_size == DIAG_RING_SIZE && p->head_inv == ~p->head;
    uint32_t boot;
    int n;
    if (valid) {
        const uint32_t head = p->head;
        const uint32_t held = head < DIAG_RING_SIZE ? head : DIAG_RING_SIZE;
        uint32_t fixed = 0;
        for (uint32_t i = head - held; i != head; i++) {
            if (!printable(p->ring[i & DIAG_RING_MASK])) {
                p->ring[i & DIAG_RING_MASK] = '?';
                fixed++;
            }
        }
        struct diag_crumbs c;
        volatile uint32_t *src = (volatile uint32_t *)&p->crumbs;
        uint32_t *dst = (uint32_t *)&c;
        for (uint32_t i = 0; i < DIAG_CRUMB_WORDS; i++) {
            dst[i] = src[i];
        }
        boot = p->boot_count + 1U;
        n = snprintf(line, sizeof(line),
                     "\r\n=== previous boot ended here; reset record: por=%u ext=%u pds=%u wdt=%u cpu_por=%u sys=%u cpu_sys=%u;"
                     " previous boot #%lu: main loop last at %s (0x%02lx) in pass %lu at %lu ms, monitor last ran at %lu ms;"
                     " software reset: %s%s at %lu ms; usb isr %s (%lu interrupts, last setup %08lx %08lx);"
                     " last bt rx type %lu code 0x%02lx; last bt tx type %lu op/handle 0x%04lx;"
                     " %lu unreadable byte(s) shown as '?' ===\r\n",
                     rec.reset_recorder_ana_powb, rec.reset_recorder_ext_rst_n, rec.reset_recorder_pds_reset,
                     rec.reset_recorder_wdt_rst_n, rec.reset_recorder_cpu_porst_n,
                     rec.reset_recorder_sys_reset_n, rec.reset_recorder_cpu_sys_rstreq_n,
                     (unsigned long)(boot - 1U), diag_where_name(c.main_where), (unsigned long)c.main_where,
                     (unsigned long)c.main_passes, (unsigned long)c.main_last_ms, (unsigned long)c.monitor_ms,
                     reset_reason_name(c.reset_reason),
                     (c.reset_reason & 0x100U) ? ", then a software reset because the watchdog did not fire" : "",
                     (unsigned long)c.reset_ms,
                     c.usb_isr_inside ? "WAS RUNNING" : "idle", (unsigned long)c.usb_isr_count,
                     (unsigned long)c.usb_last_setup[0], (unsigned long)c.usb_last_setup[1],
                     (unsigned long)(c.bt_last_rx >> 8), (unsigned long)(c.bt_last_rx & 0xFFU),
                     (unsigned long)(c.bt_last_tx >> 16), (unsigned long)(c.bt_last_tx & 0xFFFFU),
                     (unsigned long)fixed);
    } else {
        boot = 1;
        p->magic = DIAG_MAGIC;
        p->magic_inv = ~DIAG_MAGIC;
        p->ring_size = DIAG_RING_SIZE;
        p->head = 0;
        p->head_inv = ~0U;
        n = snprintf(line, sizeof(line),
                     "\r\n=== no previous log in RAM (power-on, or the RAM was not kept: header magic %08lx);"
                     " reset record: por=%u ext=%u pds=%u wdt=%u cpu_por=%u sys=%u cpu_sys=%u ===\r\n",
                     (unsigned long)magic_seen,
                     rec.reset_recorder_ana_powb, rec.reset_recorder_ext_rst_n, rec.reset_recorder_pds_reset,
                     rec.reset_recorder_wdt_rst_n, rec.reset_recorder_cpu_porst_n,
                     rec.reset_recorder_sys_reset_n, rec.reset_recorder_cpu_sys_rstreq_n);
    }
    p->boot_count = boot;
    volatile uint32_t *cw = (volatile uint32_t *)&p->crumbs;
    for (uint32_t i = 0; i < DIAG_CRUMB_WORDS; i++) {
        cw[i] = 0;
    }
    if (n > 0) {
        put_raw_locked((const uint8_t *)line, (uint32_t)n < sizeof(line) ? (uint32_t)n : sizeof(line) - 1U);
    }
    n = snprintf(line, sizeof(line), "=== boot #%lu, log ring %lu bytes at 0x%08lx%s ===\r\n",
                 (unsigned long)boot, (unsigned long)DIAG_RING_SIZE, (unsigned long)addr,
                 addr < BOOT2_RAM_TOUCHED ? ", WARNING: below 0x62FE8800, in RAM boot2 uses: the previous-boot log may not survive" : "");
    if (n > 0) {
        put_raw_locked((const uint8_t *)line, (uint32_t)n < sizeof(line) ? (uint32_t)n : sizeof(line) - 1U);
    }
}

static inline void ensure_init_locked(void)
{
    if (!g_inited) {
        diag_init_locked();
    }
}

/* ------------------------------------------------------- console copy */

/* "[HCI] Link key: " is followed by the key in hex (bt.cpp); those digits are
 * replaced. The match runs on the byte stream, one printf character at a time. */
static const char k_key_marker[] = "Link key: ";
static uint8_t g_key_match;
static bool g_key_redact;

static uint8_t redact(uint8_t c)
{
    if (g_key_redact) {
        if (c == '\n' || c == '\r') {
            g_key_redact = false;
            return c;
        }
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
            return '*';
        }
        return c;
    }
    if (c == (uint8_t)k_key_marker[g_key_match]) {
        g_key_match++;
        if (k_key_marker[g_key_match] == '\0') {
            g_key_match = 0;
            g_key_redact = true;
        }
    } else {
        g_key_match = (c == (uint8_t)k_key_marker[0]) ? 1U : 0U;
    }
    return c;
}

static void diag_console_put(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    uintptr_t flags = bflb_irq_save();
    ensure_init_locked();
    for (uint32_t i = 0; i < len; i++) {
        const uint8_t c = redact(data[i]);
        put_raw_locked(&c, 1U);
    }
    bflb_irq_restore(flags);
}

int __real_bflb_uart_putchar(struct bflb_device_s *dev, int ch);
int __real_bflb_uart_put(struct bflb_device_s *dev, uint8_t *data, uint32_t len);
int __real_bflb_uart_put_block(struct bflb_device_s *dev, uint8_t *data, uint32_t len);

int __wrap_bflb_uart_putchar(struct bflb_device_s *dev, int ch)
{
    uint8_t c = (uint8_t)ch;
    diag_console_put(&c, 1U);
    return __real_bflb_uart_putchar(dev, ch);
}

int __wrap_bflb_uart_put(struct bflb_device_s *dev, uint8_t *data, uint32_t len)
{
    diag_console_put(data, len);
    return __real_bflb_uart_put(dev, data, len);
}

int __wrap_bflb_uart_put_block(struct bflb_device_s *dev, uint8_t *data, uint32_t len)
{
    diag_console_put(data, len);
    return __real_bflb_uart_put_block(dev, data, len);
}

/* ------------------------------------------------------- ring-only lines */

static void put_line(const char *buf, int n, size_t cap)
{
    if (n <= 0) {
        return;
    }
    uint32_t len = (uint32_t)n;
    if (len >= cap) {
        len = (uint32_t)cap - 1U; /* truncated: still end the line */
    }
    uintptr_t flags = bflb_irq_save();
    ensure_init_locked();
    put_raw_locked((const uint8_t *)buf, len);
    if (len == (uint32_t)cap - 1U && (uint32_t)n >= cap) {
        put_raw_locked((const uint8_t *)"\r\n", 2U);
    }
    bflb_irq_restore(flags);
}

void diag_logf(const char *fmt, ...)
{
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    put_line(buf, n, sizeof(buf));
}

/* ------------------------------------------------------------ HID reader */

uint16_t diag_log_fill_report(uint8_t *buffer, uint16_t reqlen)
{
    if (buffer == NULL || reqlen < 1U) {
        return 0;
    }
    uint32_t room = (uint32_t)reqlen - 1U;
    if (room > DIAG_LOG_REPORT_SIZE - 1U) {
        room = DIAG_LOG_REPORT_SIZE - 1U;
    }

    uintptr_t flags = bflb_irq_save();
    ensure_init_locked();
    volatile struct diag_persist *p = g_p;
    uint32_t head = p->head;
    if ((uint32_t)(head - g_tail) > DIAG_RING_SIZE) {
        g_tail = head - DIAG_RING_SIZE; /* overrun (or first read): oldest held byte */
    }
    uint32_t n = head - g_tail;
    if (n > room) {
        n = room;
    }
    for (uint32_t i = 0; i < n; i++) {
        buffer[1 + i] = p->ring[(g_tail + i) & DIAG_RING_MASK];
    }
    g_tail += n;
    bflb_irq_restore(flags);

    buffer[0] = (uint8_t)n;
    /* Always the full report: the host's feature read is fixed-size. */
    memset(buffer + 1 + n, 0, (size_t)(DIAG_LOG_REPORT_SIZE - 1U - n));
    return DIAG_LOG_REPORT_SIZE <= reqlen ? DIAG_LOG_REPORT_SIZE : reqlen;
}

/* -------------------------------------------------------------- crumbs */

const char *diag_where_name(uint32_t code)
{
    const bool returned = (code & DIAG_WHERE_RETURNED) != 0U;
    switch (code & 0x7FU) {
        case 0:                         return "not started";
        case DIAG_WHERE_LOOP_TOP:       return returned ? "after watchdog_update" : "watchdog_update (loop top)";
        case DIAG_WHERE_BT_POLL:        return returned ? "after bt_transport_poll" : "bt_transport_poll (draining HCI)";
        case DIAG_WHERE_BT_TASK:        return returned ? "after bt_task" : "bt_task";
        case DIAG_WHERE_USB_RECONNECT:  return returned ? "after usb_reconnect_task" : "usb_reconnect_task";
        case DIAG_WHERE_TUD_TASK:       return returned ? "after tud_task" : "tud_task";
        case DIAG_WHERE_WAKE_TASK:      return returned ? "after wake_task" : "wake_task";
        case DIAG_WHERE_SHORTCUT_TASK:  return returned ? "after button_shortcut_task" : "button_shortcut_task";
        case DIAG_WHERE_AUDIO_LOOP:     return returned ? "after audio_loop (in interrupt_loop)" : "audio_loop";
        case DIAG_WHERE_BATT_LED:       return returned ? "after battery_led_tick" : "battery_led_tick";
        case DIAG_WHERE_BUTTON_CHECK:   return returned ? "after button_check" : "button_check";
        case DIAG_WHERE_INQUIRY_LED:    return returned ? "after bt_inquiring_led" : "bt_inquiring_led";
        case DIAG_WHERE_BT_DISPATCH:    return "bt_transport_poll: in BTstack's packet handler";
        case DIAG_WHERE_BT_DCD_POLL:    return "bt_transport_poll: bl616_dcd_poll";
        case DIAG_WHERE_BT_RUN_LOOP:    return "bt_transport_poll: BTstack run loop";
        case DIAG_WHERE_BT_YIELD:       return "bt_transport_poll: taskYIELD";
        default:                        return "unknown";
    }
}

void diag_set_where(uint32_t code)
{
    volatile struct diag_persist *p = g_p;
    if (p != NULL) {
        p->crumbs.main_where = code;
    }
}

void diag_main_pass(void)
{
    volatile struct diag_persist *p = g_p;
    if (p == NULL) {
        return;
    }
    const uint32_t now = now_ms();
    const uint32_t passes = p->crumbs.main_passes;
    if (passes != 0U) {
        const uint32_t gap = now - p->crumbs.main_last_ms;
        if (gap > diag_cnt.main_max_gap_ms) {
            diag_cnt.main_max_gap_ms = gap;
        }
    }
    p->crumbs.main_last_ms = now;
    p->crumbs.main_passes = passes + 1U;
}

void diag_note_bt_rx(uint8_t type, uint8_t code)
{
    volatile struct diag_persist *p = g_p;
    if (p != NULL) {
        p->crumbs.bt_last_rx = ((uint32_t)type << 8) | code;
    }
}

void diag_note_bt_tx(uint8_t type, uint16_t opcode)
{
    volatile struct diag_persist *p = g_p;
    if (p != NULL) {
        p->crumbs.bt_last_tx = ((uint32_t)type << 16) | opcode;
    }
}

void diag_note_usb_isr(int inside, const uint8_t *setup)
{
    volatile struct diag_persist *p = g_p;
    if (p == NULL) {
        return;
    }
    if (setup != NULL) {
        p->crumbs.usb_last_setup[0] = (uint32_t)setup[0] << 24 | (uint32_t)setup[1] << 16 |
                                      (uint32_t)setup[3] << 8 | setup[2];
        p->crumbs.usb_last_setup[1] = (uint32_t)setup[5] << 24 | (uint32_t)setup[4] << 16 |
                                      (uint32_t)setup[7] << 8 | setup[6];
    }
    p->crumbs.usb_isr_inside = inside ? 1U : 0U;
    if (inside && setup == NULL) { /* counted on entry, not again for its SETUP */
        p->crumbs.usb_isr_count++;
    }
}

/* ------------------------------------------------------- fatal stops */

static void record_reset_reason(uint32_t reason)
{
    volatile struct diag_persist *p = g_p;
    if (p != NULL) {
        p->crumbs.reset_reason = reason;
        p->crumbs.reset_ms = now_ms();
    }
}

static const char *task_name_now(void)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return "(before the scheduler)";
    }
    TaskHandle_t t = xTaskGetCurrentTaskHandle();
    return t != NULL ? pcTaskGetName(t) : "(none)";
}

void diag_fatal(uint32_t reason, const char *fmt, ...)
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "[diag] t=%lu FATAL: ", (unsigned long)now_ms());
    if (n < 0) {
        n = 0;
    }
    if ((size_t)n < sizeof(buf)) {
        va_list ap;
        va_start(ap, fmt);
        int m = vsnprintf(buf + n, sizeof(buf) - (size_t)n, fmt, ap);
        va_end(ap);
        n += m > 0 ? m : 0;
    }
    put_line(buf, n, sizeof(buf));
    diag_logf("[diag] main loop was at %s; masking interrupts and waiting for the watchdog\r\n",
              diag_where_name(g_p != NULL ? g_p->crumbs.main_where : 0U));
    record_reset_reason(reason);
    (void)bflb_irq_save();
    const uint32_t t0 = now_ms();
    while ((uint32_t)(now_ms() - t0) < 1500U) {
    }
    record_reset_reason(reason | 0x100U);
    GLB_SW_System_Reset();
    while (1) {
    }
}

/* CPU exceptions. The SDK's exception_entry() prints to the console and then
 * spins in its core dump loop (CONFIG_COREDUMP) until the watchdog -- if it is
 * armed -- resets the chip. Wrapped (CMakeLists.txt) so the debug build writes
 * the cause into the persistent ring and resets at once.
 *
 * Two trap paths reach it. Before the scheduler starts, mtvec is the SDK's
 * default_trap_handler (vector.S), which passes the saved registers in a0.
 * Once FreeRTOS runs, mtvec is freertos_risc_v_trap_handler (portASM.S), which
 * passes mcause in a0 and has saved the context on the trapping stack with
 * that stack pointer stored in pxCurrentTCB->pxTopOfStack (portContext.h):
 * mstatus at word 1, ra at word 2 of the integer frame, with 33 FPU words
 * below it when the FPU state was dirty; the trapping sp is the integer frame
 * plus 31 words. A trap taken inside an interrupt handler saves its frame the
 * same way, so ra/sp are then the interrupt handler's. */
extern void freertos_risc_v_trap_handler(void);
extern void *volatile pxCurrentTCB;      /* tasks.c */
extern const BaseType_t *pTrapNetCounter; /* port.c: trap nesting, 1 = from a task */
void __real_exception_entry(uintptr_t *regs);

static bool ram_word_addr(uintptr_t a)
{
    if ((a & 3U) != 0U) {
        return false;
    }
    return (a >= 0x62FC0000U && a < 0x63038000U) || (a >= 0x22FC0000U && a < 0x23038000U);
}

static const char *mcause_name(uint32_t code)
{
    static const char *const names[] = {
        "instruction address misaligned", "instruction access fault", "illegal instruction",
        "breakpoint", "load address misaligned", "load access fault",
        "store address misaligned", "store access fault", "ecall from U-mode",
        "ecall from S-mode", "reserved", "ecall from M-mode",
        "instruction page fault", "load page fault", "reserved", "store page fault",
    };
    return code < sizeof(names) / sizeof(names[0]) ? names[code] : "other";
}

void __wrap_exception_entry(uintptr_t *regs)
{
    static volatile uint32_t in_fault;
    if (in_fault) {
        GLB_SW_System_Reset(); /* a fault while reporting one */
        while (1) {
        }
    }
    in_fault = 1;

    uint32_t mcause, mepc, mtval, mtvec;
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));
    __asm__ volatile("csrr %0, mepc" : "=r"(mepc));
    __asm__ volatile("csrr %0, mtval" : "=r"(mtval));
    __asm__ volatile("csrr %0, mtvec" : "=r"(mtvec));

    uint32_t ra = 0, sp = 0;
    long nest = -1;
    const char *path;
    /* Low bits of mtvec are the mode (3 = CLIC here); both handlers are at
     * least 64-byte aligned. */
    if ((mtvec & ~0x3FU) == (uint32_t)(uintptr_t)freertos_risc_v_trap_handler) {
        path = "freertos";
        void *tcb = pxCurrentTCB;
        if (tcb != NULL && ram_word_addr((uintptr_t)tcb)) {
            const uint32_t *frame = *(uint32_t *const *)tcb;
            if (ram_word_addr((uintptr_t)frame)) {
                const uint32_t mstatus = frame[1];
                const uint32_t *base = (((mstatus >> 13) & 3U) == 3U) ? frame + 33 : frame;
                if (ram_word_addr((uintptr_t)base)) {
                    ra = base[2];
                    sp = (uint32_t)(uintptr_t)(base + 31);
                }
            }
        }
        if (pTrapNetCounter != NULL) {
            nest = (long)*pTrapNetCounter;
        }
    } else {
        path = "default";
        if (regs != NULL && ram_word_addr((uintptr_t)regs)) {
            ra = (uint32_t)regs[1];
            sp = (uint32_t)regs[2];
        }
    }

    diag_logf("[diag] t=%lu FAULT mcause=0x%08lx (%s) mepc=0x%08lx mtval=0x%08lx ra=0x%08lx sp=0x%08lx"
              " task=%s trap=%s nest=%ld main_at=%s; resetting\r\n",
              (unsigned long)now_ms(), (unsigned long)mcause,
              (mcause & 0x80000000U) ? "interrupt?" : mcause_name(mcause & 0x3FFU),
              (unsigned long)mepc, (unsigned long)mtval, (unsigned long)ra, (unsigned long)sp,
              task_name_now(), path, nest,
              diag_where_name(g_p != NULL ? g_p->crumbs.main_where : 0U));
    record_reset_reason(DIAG_RESET_FAULT);
    GLB_SW_System_Reset();
    while (1) {
    }
}

/* ------------------------------------------------------------- monitor */

#define DIAG_TASK_STACK_WORDS 1024U
#define DIAG_TICK_MS          250U
#define DIAG_COUNTER_MS       5000U
#define DIAG_HEALTH_MS        30000U
#define DIAG_STALL_MS         500U
#define DIAG_MAX_TASKS        20U

static StackType_t diag_task_stack[DIAG_TASK_STACK_WORDS];
static StaticTask_t diag_task_tcb;
static TaskStatus_t g_task_status[DIAG_MAX_TASKS];

static const char *task_state_name(eTaskState s)
{
    switch (s) {
        case eRunning:   return "running";
        case eReady:     return "ready";
        case eBlocked:   return "blocked";
        case eSuspended: return "suspended";
        case eDeleted:   return "deleted";
        default:         return "invalid";
    }
}

static const char *named_task_state(const char *name, TaskHandle_t *cache)
{
    if (*cache == NULL) {
        *cache = xTaskGetHandle(name);
    }
    return *cache != NULL ? task_state_name(eTaskGetState(*cache)) : "not found";
}

struct usb_snapshot {
    uint32_t setup, reset, susp, resm;
    uint32_t armed_in[5], armed_out[5], done_in[5], done_out[5];
    uint16_t set_report[256], out_report[256];
};

static struct usb_snapshot g_usb_last;
static struct {
    uint32_t acl_in, evt_in, acl_out, acl_out_bytes, cmd_out, send_fail, drops, evt_unlogged, passes;
} g_bt_last;

static void print_bt_line(uint32_t now)
{
    const uint32_t acl_in = diag_cnt.bt_acl_in, evt_in = diag_cnt.bt_evt_in;
    const uint32_t acl_out = diag_cnt.bt_acl_out, bytes = diag_cnt.bt_acl_out_bytes;
    const uint32_t cmd_out = diag_cnt.bt_cmd_out, fail = diag_cnt.bt_send_fail;
    const uint32_t drops = diag_cnt.bt_rx_drops, unlogged = diag_cnt.bt_evt_unlogged;
    const uint32_t passes = g_p != NULL ? g_p->crumbs.main_passes : 0U;
    const uint32_t peak = __atomic_exchange_n(&diag_cnt.bt_ring_peak, 0U, __ATOMIC_SEQ_CST);
    const uint32_t gap = __atomic_exchange_n(&diag_cnt.main_max_gap_ms, 0U, __ATOMIC_SEQ_CST);
    const int32_t free_now = diag_cnt.bt_acl_free_now;
    const int32_t free_min = __atomic_exchange_n(&diag_cnt.bt_acl_free_min, -1, __ATOMIC_SEQ_CST);
    char slots[40];
    if (free_now < 0 && free_min < 0) {
        snprintf(slots, sizeof(slots), "n/a (no connection)");
    } else {
        snprintf(slots, sizeof(slots), "%ld (min %ld)", (long)free_now, (long)free_min);
    }
    diag_logf("[diag] t=%lu bt: acl_in=%lu evt_in=%lu acl_out=%lu (%lu B) cmd_out=%lu send_fail=%lu"
              " ring_peak=%lu/%lu drops=%lu evt_unlogged=%lu acl_free=%s | main: passes=%lu max_gap=%lu ms\r\n",
              (unsigned long)now, (unsigned long)(acl_in - g_bt_last.acl_in),
              (unsigned long)(evt_in - g_bt_last.evt_in), (unsigned long)(acl_out - g_bt_last.acl_out),
              (unsigned long)(bytes - g_bt_last.acl_out_bytes), (unsigned long)(cmd_out - g_bt_last.cmd_out),
              (unsigned long)(fail - g_bt_last.send_fail), (unsigned long)peak,
              (unsigned long)diag_cnt.bt_ring_size, (unsigned long)(drops - g_bt_last.drops),
              (unsigned long)(unlogged - g_bt_last.evt_unlogged), slots,
              (unsigned long)(passes - g_bt_last.passes), (unsigned long)gap);
    g_bt_last.acl_in = acl_in;
    g_bt_last.evt_in = evt_in;
    g_bt_last.acl_out = acl_out;
    g_bt_last.acl_out_bytes = bytes;
    g_bt_last.cmd_out = cmd_out;
    g_bt_last.send_fail = fail;
    g_bt_last.drops = drops;
    g_bt_last.evt_unlogged = unlogged;
    g_bt_last.passes = passes;
}

static uint32_t delta_and_keep(volatile uint32_t *now, uint32_t *last)
{
    const uint32_t v = *now;
    const uint32_t d = v - *last;
    *last = v;
    return d;
}

static void append_ids(char *buf, size_t cap, volatile uint16_t *now, uint16_t *last)
{
    size_t len = strlen(buf);
    bool any = false;
    for (uint32_t id = 0; id < 256U; id++) {
        const uint16_t v = now[id];
        const uint16_t d = (uint16_t)(v - last[id]);
        last[id] = v;
        if (d == 0U || len + 12U >= cap) {
            continue;
        }
        int n = snprintf(buf + len, cap - len, " 0x%02lx:%u", (unsigned long)id, (unsigned int)d);
        if (n > 0) {
            len += (size_t)n;
        }
        any = true;
    }
    if (!any && len + 6U < cap) {
        strcat(buf, " none");
    }
}

static void print_usb_lines(uint32_t now)
{
    struct usb_snapshot *l = &g_usb_last;
    uint32_t ai[5], ao[5], di[5], dout[5];
    for (int i = 0; i < 5; i++) {
        ai[i] = delta_and_keep(&diag_cnt.usb_armed_in[i], &l->armed_in[i]);
        ao[i] = delta_and_keep(&diag_cnt.usb_armed_out[i], &l->armed_out[i]);
        di[i] = delta_and_keep(&diag_cnt.usb_done_in[i], &l->done_in[i]);
        dout[i] = delta_and_keep(&diag_cnt.usb_done_out[i], &l->done_out[i]);
    }
    uint32_t isg0 = 0, isg2 = 0, isg3 = 0;
    int started = 0, connected = 0;
    bl616_dcd_diag_status(&isg0, &isg2, &isg3, &started, &connected);
    diag_logf("[diag] t=%lu usb: started=%d connected=%d setup=%lu resets=%lu suspends=%lu resumes=%lu"
              " done/armed ep0 in %lu/%lu out %lu/%lu | ep1 out %lu/%lu | ep2 in %lu/%lu | ep3 out %lu/%lu | ep4 in %lu/%lu"
              " | err isg0=0x%02lx isg2=0x%03lx isg3=0x%06lx\r\n",
              (unsigned long)now, started, connected,
              (unsigned long)delta_and_keep(&diag_cnt.usb_setup, &l->setup),
              (unsigned long)delta_and_keep(&diag_cnt.usb_bus_reset, &l->reset),
              (unsigned long)delta_and_keep(&diag_cnt.usb_suspend, &l->susp),
              (unsigned long)delta_and_keep(&diag_cnt.usb_resume, &l->resm),
              (unsigned long)di[0], (unsigned long)ai[0], (unsigned long)dout[0], (unsigned long)ao[0],
              (unsigned long)dout[1], (unsigned long)ao[1], (unsigned long)di[2], (unsigned long)ai[2],
              (unsigned long)dout[3], (unsigned long)ao[3], (unsigned long)di[4], (unsigned long)ai[4],
              (unsigned long)isg0, (unsigned long)isg2, (unsigned long)isg3);
    char buf[256];
    snprintf(buf, sizeof(buf), "[diag] t=%lu usb ids: set_report", (unsigned long)now);
    append_ids(buf, sizeof(buf) - 24U, diag_cnt.usb_set_report, l->set_report);
    strcat(buf, " | ep3 out");
    append_ids(buf, sizeof(buf) - 4U, diag_cnt.usb_out_report, l->out_report);
    strcat(buf, "\r\n");
    diag_logf("%s", buf);
}

static void print_task_health(uint32_t now)
{
    const UBaseType_t total = uxTaskGetNumberOfTasks();
    const UBaseType_t n = uxTaskGetSystemState(g_task_status, DIAG_MAX_TASKS, NULL);
    if (n == 0U) {
        diag_logf("[diag] t=%lu tasks: %lu, more than the %u this report holds\r\n",
                  (unsigned long)now, (unsigned long)total, (unsigned int)DIAG_MAX_TASKS);
        return;
    }
    diag_logf("[diag] t=%lu tasks: %lu; heap_free=%u\r\n", (unsigned long)now, (unsigned long)n,
              (unsigned int)kfree_size(0));
    for (UBaseType_t i = 0; i < n; i++) {
        const TaskStatus_t *t = &g_task_status[i];
        diag_logf("[diag]   task %-16s prio %2lu %-9s stack never-used %lu B\r\n", t->pcTaskName,
                  (unsigned long)t->uxCurrentPriority, task_state_name(t->eCurrentState),
                  (unsigned long)t->usStackHighWaterMark * sizeof(StackType_t));
    }
}

static void diag_task(void *arg)
{
    (void)arg;
    TaskHandle_t main_task = NULL, worker_task = NULL;
    uint32_t last_passes = 0;
    bool stall_reported = false;
    uint32_t next_stall_report = 0;
    uint32_t next_counters = now_ms() + DIAG_COUNTER_MS;
    uint32_t next_health = now_ms() + DIAG_HEALTH_MS;
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(DIAG_TICK_MS));
        const uint32_t now = now_ms();
        if (g_p != NULL) {
            g_p->crumbs.monitor_ms = now;
        }

        /* Main-loop stall: the pass counter has not moved. The hardware
         * watchdog (1 s, fed at the loop top) is the one that resets. */
        const uint32_t passes = g_p != NULL ? g_p->crumbs.main_passes : 0U;
        if (passes != last_passes) {
            if (stall_reported) {
                diag_logf("[diag] t=%lu main loop running again (%lu passes since)\r\n",
                          (unsigned long)now, (unsigned long)(passes - last_passes));
            }
            stall_reported = false;
            last_passes = passes;
        } else if (passes != 0U) {
            const uint32_t age = now - g_p->crumbs.main_last_ms;
            if (age >= DIAG_STALL_MS && (!stall_reported || age >= next_stall_report)) {
                const uint32_t where = g_p->crumbs.main_where;
                diag_logf("[diag] t=%lu watchdog: main loop stalled %lu ms at %s (0x%02lx); ds5_main %s,"
                          " audio_worker %s; last bt rx type %lu code 0x%02lx; usb isr %s\r\n",
                          (unsigned long)now, (unsigned long)age, diag_where_name(where), (unsigned long)where,
                          named_task_state("ds5_main", &main_task), named_task_state("audio_worker", &worker_task),
                          (unsigned long)(g_p->crumbs.bt_last_rx >> 8), (unsigned long)(g_p->crumbs.bt_last_rx & 0xFFU),
                          g_p->crumbs.usb_isr_inside ? "running" : "idle");
                stall_reported = true;
                next_stall_report = age + 1000U;
            }
        }

        if ((int32_t)(now - next_counters) >= 0) {
            next_counters += DIAG_COUNTER_MS;
            diag_logf("[diag] heartbeat uptime_ms=%lu heap_free=%u logged=%lu\r\n", (unsigned long)now,
                      (unsigned int)kfree_size(0), (unsigned long)(g_p != NULL ? g_p->head : 0U));
            print_bt_line(now);
            print_usb_lines(now);
        }
        if ((int32_t)(now - next_health) >= 0) {
            next_health += DIAG_HEALTH_MS;
            print_task_health(now);
        }
    }
}

void diag_log_start_heartbeat(unsigned long priority)
{
    diag_cnt.bt_acl_free_now = -1;
    diag_cnt.bt_acl_free_min = -1;
    if (xTaskCreateStatic(diag_task, "diag_mon", DIAG_TASK_STACK_WORDS, NULL, (UBaseType_t)priority,
                          diag_task_stack, &diag_task_tcb) == NULL) {
        diag_logf("[diag] monitor task create FAILED\r\n");
    }
}
