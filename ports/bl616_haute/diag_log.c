/*
 * Debug-build log for the BL616 Haute target, for a board whose UART pins are
 * not reachable.
 *
 * Every console byte is copied into a RAM ring by wrapping the SDK's UART
 * output calls at link time (-Wl,--wrap, see CMakeLists.txt), so printf from
 * any task -- the SDK banner before main() included -- lands in the ring.
 *
 * The ring is read over USB without an endpoint of its own: the BL616 has
 * four, and the DualSense uses all of them. The debug build adds a vendor
 * collection with a 64-byte feature report (DIAG_LOG_REPORT_ID) to the
 * DualSense HID interface, and each GET_REPORT of it returns the next bytes
 * (src/port/bl616/usb_hid_bl616.cpp). So the debug build presents the same
 * device as the production build -- audio included -- plus that one report.
 * A host tool polls it (capture-h-log.ps1 -Hid on the share).
 *
 * The log can contain a Bluetooth link key (bt.cpp prints the stored key on a
 * Link Key Request). Treat a saved log as sensitive.
 */
#include "diag_log.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bflb_irq.h"
#include "bflb_mtimer.h"
#include "bflb_uart.h"
#include "FreeRTOS.h"
#include "mm.h"
#include "task.h"

/* ------------------------------------------------------------------ ring */

#define DIAG_RING_SIZE 32768U /* power of two */
#define DIAG_RING_MASK (DIAG_RING_SIZE - 1U)

static uint8_t diag_ring[DIAG_RING_SIZE];
static volatile uint32_t diag_head; /* total bytes ever written */
static uint32_t diag_tail;          /* next byte the host reads */

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
    uint32_t head = diag_head;
    if ((uint32_t)(head - diag_tail) > DIAG_RING_SIZE) {
        diag_tail = head - DIAG_RING_SIZE; /* overrun (or first read): oldest held byte */
    }
    uint32_t n = head - diag_tail;
    if (n > room) {
        n = room;
    }
    for (uint32_t i = 0; i < n; i++) {
        buffer[1 + i] = diag_ring[(diag_tail + i) & DIAG_RING_MASK];
    }
    diag_tail += n;
    bflb_irq_restore(flags);

    buffer[0] = (uint8_t)n;
    /* Always the full report: the host's feature read is fixed-size. */
    memset(buffer + 1 + n, 0, (size_t)(DIAG_LOG_REPORT_SIZE - 1U - n));
    return DIAG_LOG_REPORT_SIZE <= reqlen ? DIAG_LOG_REPORT_SIZE : reqlen;
}

/* ------------------------------------------------------------- heartbeat */

#define DIAG_TASK_STACK_WORDS 512U
#define DIAG_HEARTBEAT_MS     5000U

static StackType_t diag_task_stack[DIAG_TASK_STACK_WORDS];
static StaticTask_t diag_task_tcb;

static void diag_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_HEARTBEAT_MS));
        printf("[diag] heartbeat uptime_ms=%lu heap_free=%u logged=%lu\r\n",
               (unsigned long)bflb_mtimer_get_time_ms(), (unsigned int)kfree_size(0),
               (unsigned long)diag_head);
    }
}

void diag_log_start_heartbeat(unsigned long priority)
{
    if (xTaskCreateStatic(diag_task, "diag_hb", DIAG_TASK_STACK_WORDS, NULL, (UBaseType_t)priority,
                          diag_task_stack, &diag_task_tcb) == NULL) {
        printf("[diag] heartbeat task create FAILED\r\n");
    }
}
