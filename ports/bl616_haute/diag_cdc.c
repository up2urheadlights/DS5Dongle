/*
 * Debug-build log port for the BL616 Haute target: the UART0 console, mirrored
 * to a USB CDC-ACM serial port, for boards whose UART pins are not reachable.
 *
 * Every console byte is copied into a RAM ring by wrapping the SDK's UART
 * output calls at link time (-Wl,--wrap, see CMakeLists.txt), so printf from
 * any task -- the SDK banner before main() included -- lands in the ring. When
 * the host opens the port (DTR), the ring is replayed from its oldest byte.
 *
 * USB is started only once Bluetooth is up, and with the USB clock recovered
 * first: on this chip, starting the controller earlier lost the port partway
 * through Bluetooth init. That order is the one other firmware on this board
 * uses for its own USB device. GPIO32/33 are never touched (not led out on
 * QFN40; the SDK pad check traps on them), and the USB PHY needs no pin mux.
 *
 * This is a stage-1 instrument: the DualSense USB device (TinyUSB, a BL616
 * device-controller driver) replaces it, and the debug build then uses the
 * firmware's own ENABLE_SERIAL CDC like the other targets.
 *
 * The log can contain a Bluetooth link key (bt.cpp prints the stored key on a
 * Link Key Request). Treat a saved log as sensitive.
 */
#include "diag_cdc.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bflb_clock.h"
#include "bflb_core.h"
#include "bflb_irq.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "bl616_glb.h"
#include "FreeRTOS.h"
#include "mm.h"
#include "task.h"
#include "usbd_cdc_acm.h"
#include "usbd_core.h"

/* ------------------------------------------------------------------ ring */

#define DIAG_RING_SIZE 32768U /* power of two */
#define DIAG_RING_MASK (DIAG_RING_SIZE - 1U)

static uint8_t diag_ring[DIAG_RING_SIZE];
static volatile uint32_t diag_head; /* total bytes ever written */

static void diag_ring_put(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }
    uintptr_t flags = bflb_irq_save();
    uint32_t head = diag_head;
    for (uint32_t i = 0; i < len; i++) {
        diag_ring[(head + i) & DIAG_RING_MASK] = data[i];
    }
    diag_head = head + len;
    bflb_irq_restore(flags);
}

int __real_bflb_uart_putchar(struct bflb_device_s *dev, int ch);
int __real_bflb_uart_put(struct bflb_device_s *dev, uint8_t *data, uint32_t len);
int __real_bflb_uart_put_block(struct bflb_device_s *dev, uint8_t *data, uint32_t len);

int __wrap_bflb_uart_putchar(struct bflb_device_s *dev, int ch)
{
    uint8_t c = (uint8_t)ch;
    diag_ring_put(&c, 1U);
    return __real_bflb_uart_putchar(dev, ch);
}

int __wrap_bflb_uart_put(struct bflb_device_s *dev, uint8_t *data, uint32_t len)
{
    diag_ring_put(data, len);
    return __real_bflb_uart_put(dev, data, len);
}

int __wrap_bflb_uart_put_block(struct bflb_device_s *dev, uint8_t *data, uint32_t len)
{
    diag_ring_put(data, len);
    return __real_bflb_uart_put_block(dev, data, len);
}

/* ------------------------------------------------------------ CDC device */

#define DIAG_CDC_IN_EP  0x83
#define DIAG_CDC_OUT_EP 0x04
#define DIAG_CDC_INT_EP 0x85
#ifdef CONFIG_USB_HS
#define DIAG_CDC_MPS 512U
#else
#define DIAG_CDC_MPS 64U
#endif

/* 0xFFFF is the CherryUSB demo vendor id, assigned to no one, so this debug
 * port cannot be mistaken for a product. Windows binds usbser.sys by the
 * CDC-ACM class code. */
#define DIAG_USBD_VID 0xFFFF
#define DIAG_USBD_PID 0xD012

#define DIAG_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)

static const uint8_t diag_device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, DIAG_USBD_VID, DIAG_USBD_PID, 0x0100, 0x01)
};

static const uint8_t diag_config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(DIAG_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, 100),
    CDC_ACM_DESCRIPTOR_INIT(0x00, DIAG_CDC_INT_EP, DIAG_CDC_OUT_EP, DIAG_CDC_IN_EP, DIAG_CDC_MPS, 0x02)
};

static const uint8_t diag_device_quality_descriptor[] = {
    0x0a, USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00,
};

static const char *diag_string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    "DS5Dongle",
    "DS5Dongle BL616 debug log",
    "BL616-DEBUG",
};

static const uint8_t *diag_device_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return diag_device_descriptor;
}

static const uint8_t *diag_config_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return diag_config_descriptor;
}

static const uint8_t *diag_device_quality_descriptor_cb(uint8_t speed)
{
    (void)speed;
    return diag_device_quality_descriptor;
}

static const char *diag_string_descriptor_cb(uint8_t speed, uint8_t index)
{
    (void)speed;
    if (index >= (sizeof(diag_string_descriptors) / sizeof(diag_string_descriptors[0]))) {
        return NULL;
    }
    return diag_string_descriptors[index];
}

static const struct usb_descriptor diag_cdc_descriptor = {
    .device_descriptor_callback = diag_device_descriptor_cb,
    .config_descriptor_callback = diag_config_descriptor_cb,
    .device_quality_descriptor_callback = diag_device_quality_descriptor_cb,
    .string_descriptor_callback = diag_string_descriptor_cb,
};

#define DIAG_TX_CHUNK 512U

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t diag_tx_buf[DIAG_TX_CHUNK];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t diag_rx_buf[DIAG_CDC_MPS];

static volatile bool diag_configured;
static volatile bool diag_dtr;
static volatile bool diag_tx_busy;
static volatile bool diag_replay_req;
static volatile bool diag_usb_request;
static volatile uint32_t diag_evt_reset;
static int diag_usb_init_rc = -1000;
static TaskHandle_t diag_task_handle;

static void diag_kick(void)
{
    if (diag_task_handle == NULL) {
        return;
    }
    if (xPortIsInsideInterrupt()) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(diag_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    } else {
        xTaskNotifyGive(diag_task_handle);
    }
}

static void diag_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    (void)nbytes; /* host input is discarded */
    usbd_ep_start_read(busid, DIAG_CDC_OUT_EP, diag_rx_buf, sizeof(diag_rx_buf));
}

static void diag_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;
    if (nbytes != 0U && (nbytes % DIAG_CDC_MPS) == 0U) {
        usbd_ep_start_write(busid, DIAG_CDC_IN_EP, NULL, 0); /* ZLP */
        return;
    }
    diag_tx_busy = false;
    diag_kick();
}

static struct usbd_endpoint diag_out_ep = {
    .ep_addr = DIAG_CDC_OUT_EP,
    .ep_cb = diag_bulk_out,
};

static struct usbd_endpoint diag_in_ep = {
    .ep_addr = DIAG_CDC_IN_EP,
    .ep_cb = diag_bulk_in,
};

static struct usbd_interface diag_intf0;
static struct usbd_interface diag_intf1;

static void diag_usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            diag_evt_reset++;
            diag_configured = false;
            diag_dtr = false;
            diag_tx_busy = false;
            break;
        case USBD_EVENT_DISCONNECTED:
            diag_configured = false;
            diag_dtr = false;
            diag_tx_busy = false;
            break;
        case USBD_EVENT_CONFIGURED:
            diag_tx_busy = false;
            diag_configured = true;
            usbd_ep_start_read(busid, DIAG_CDC_OUT_EP, diag_rx_buf, sizeof(diag_rx_buf));
            break;
        default:
            break;
    }
    diag_kick();
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    (void)busid;
    (void)intf;
    if (dtr && !diag_dtr) {
        diag_replay_req = true;
    }
    diag_dtr = dtr;
    diag_kick();
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{
    (void)busid;
    (void)intf;
    (void)rts;
}

static struct cdc_line_coding diag_line_coding = {
    .dwDTERate = 115200,
    .bCharFormat = 0,
    .bParityType = 0,
    .bDataBits = 8,
};

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;
    diag_line_coding = *line_coding; /* any rate works; it is USB */
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    (void)busid;
    (void)intf;
    *line_coding = diag_line_coding;
}

static int diag_usb_init(void)
{
    /* USB bus clock gate on and the USB PLL divider fed from the Wi-Fi PLL,
     * immediately before the controller starts. board_init() did both once;
     * Bluetooth init runs in between and this re-applies them. */
    PERIPHERAL_CLOCK_USB_ENABLE();
    GLB_Set_USB_CLK_From_WIFIPLL(1);
    usbd_desc_register(0, &diag_cdc_descriptor);
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &diag_intf0));
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &diag_intf1));
    usbd_add_endpoint(0, &diag_out_ep);
    usbd_add_endpoint(0, &diag_in_ep);
    diag_usb_init_rc = usbd_initialize(0, 0, diag_usbd_event_handler);
    return diag_usb_init_rc;
}

void diag_cdc_request_usb(void)
{
    diag_usb_request = true;
    diag_kick();
}

/* ------------------------------------------------------------ drain task */

#define DIAG_TASK_STACK_WORDS 768U
#define DIAG_HEARTBEAT_MS     5000U
#define DIAG_USB_FALLBACK_MS  20000U

static StackType_t diag_task_stack[DIAG_TASK_STACK_WORDS];
static StaticTask_t diag_task_tcb;

static void diag_task(void *arg)
{
    uint32_t tx_pos = 0;
    bool seen_configured = false;
    bool seen_dtr = false;

    (void)arg;
    {
        uint32_t t0 = (uint32_t)bflb_mtimer_get_time_ms();
        /* Everything logged meanwhile stays in the ring for the replay. */
        while (!diag_usb_request && (uint32_t)((uint32_t)bflb_mtimer_get_time_ms() - t0) < DIAG_USB_FALLBACK_MS) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        }
        printf("[diag] starting USB log port: %s\r\n",
               diag_usb_request ? "Bluetooth is up" : "20 s passed without Bluetooth coming up");
    }
    diag_usb_init();
    printf("[diag] usbd_initialize rc=%d\r\n", diag_usb_init_rc);

    uint32_t last_hb = (uint32_t)bflb_mtimer_get_time_ms();
    while (1) {
        uint32_t now = (uint32_t)bflb_mtimer_get_time_ms();

        if (diag_configured != seen_configured) {
            seen_configured = diag_configured;
            printf("[diag] USB %s (resets=%lu)\r\n", seen_configured ? "configured by host" : "not configured",
                   (unsigned long)diag_evt_reset);
        }
        if (diag_dtr != seen_dtr) {
            seen_dtr = diag_dtr;
            printf("[diag] host %s the port\r\n", seen_dtr ? "opened" : "closed");
        }
        if ((uint32_t)(now - last_hb) >= DIAG_HEARTBEAT_MS) {
            last_hb = now;
            printf("[diag] heartbeat uptime_ms=%lu heap_free=%u logged=%lu\r\n",
                   (unsigned long)now, (unsigned int)kfree_size(0), (unsigned long)diag_head);
        }

        if (diag_replay_req) {
            uint32_t head = diag_head;
            diag_replay_req = false;
            tx_pos = (head > DIAG_RING_SIZE) ? (head - DIAG_RING_SIZE) : 0U;
        }

        if (diag_configured && diag_dtr && !diag_tx_busy) {
            uintptr_t flags = bflb_irq_save();
            uint32_t head = diag_head;
            if ((uint32_t)(head - tx_pos) > DIAG_RING_SIZE) {
                tx_pos = head - DIAG_RING_SIZE; /* overrun: skip the oldest */
            }
            uint32_t n = head - tx_pos;
            if (n > DIAG_TX_CHUNK) {
                n = DIAG_TX_CHUNK;
            }
            for (uint32_t i = 0; i < n; i++) {
                diag_tx_buf[i] = diag_ring[(tx_pos + i) & DIAG_RING_MASK];
            }
            bflb_irq_restore(flags);

            if (n != 0U) {
                diag_tx_busy = true;
                if (usbd_ep_start_write(0, DIAG_CDC_IN_EP, diag_tx_buf, n) != 0) {
                    diag_tx_busy = false;
                } else {
                    tx_pos += n;
                }
            }
        }

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    }
}

void diag_cdc_start_task(UBaseType_t priority)
{
    diag_task_handle = xTaskCreateStatic(diag_task, "diag_cdc", DIAG_TASK_STACK_WORDS, NULL, priority,
                                         diag_task_stack, &diag_task_tcb);
    if (diag_task_handle == NULL) {
        printf("[diag] log task create FAILED\r\n");
    }
}
