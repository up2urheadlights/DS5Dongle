/* Debug-build log for the BL616 Haute target; see diag_log.c. Nothing here is
 * compiled into the production image. */
#ifndef DS5_BL616_DIAG_LOG_H
#define DS5_BL616_DIAG_LOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Feature report that carries the log, in a vendor collection the debug build
 * adds to the DualSense HID interface (src/port/bl616/usb_hid_bl616.cpp). No
 * DualSense report uses this ID. Payload after the ID: one count byte, then up
 * to DIAG_LOG_REPORT_SIZE - 1 log bytes. */
#define DIAG_LOG_REPORT_ID   0x4F
#define DIAG_LOG_REPORT_SIZE 63

/* Fills a GET_REPORT(feature, DIAG_LOG_REPORT_ID) payload with the next log
 * bytes (the first read after boot starts at the oldest byte still held, which
 * after a reset is the previous boot's tail). Returns the payload length. */
uint16_t diag_log_fill_report(uint8_t *buffer, uint16_t reqlen);

/* Starts the monitor task: heartbeat and counter lines every 5 s, task health
 * every 30 s, and the main-loop stall check every 250 ms. */
void diag_log_start_heartbeat(unsigned long priority);

/* One line into the log ring only (not the UART), written in one piece.
 * Safe from tasks, interrupts and the trap handler. Add the line end. */
void diag_logf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Logs a fatal condition (assert, stack overflow), records it as the reset
 * reason, masks interrupts and waits for the watchdog; if the watchdog has not
 * reset the chip 1.5 s later (it is armed only once the main loop starts) a
 * software reset follows. Never returns. */
void diag_fatal(uint32_t reason, const char *fmt, ...) __attribute__((format(printf, 2, 3), noreturn));

#define DIAG_RESET_NONE          0u
#define DIAG_RESET_FAULT         1u /* CPU exception */
#define DIAG_RESET_STACK         2u /* FreeRTOS stack overflow hook */
#define DIAG_RESET_ASSERT        3u /* configASSERT */
#define DIAG_RESET_FALLBACK      4u /* diag_fatal: the watchdog did not fire */
#define DIAG_RESET_STALL         5u /* monitor task: main loop stalled 3 s, the watchdog had not reset */
#define DIAG_RESET_HWTIMER       6u /* timer interrupt: the monitor task itself stopped for 3 s */
#define DIAG_RESET_USB_SILENT    7u /* variant P: configured, then the host went silent for 2 s */
#define DIAG_RESET_BT_DROP       8u /* variant P: Bluetooth link dropped within 5 s of an audio alt-setting change */

/* Variant P: asks the monitor task to reset the chip once delay_ms has passed
 * (so the lines that follow the cause still reach the ring), after a full USB
 * state dump. The first request wins; later ones are logged and ignored. Safe
 * from tasks and interrupts. */
void diag_request_reset(uint32_t reason, uint32_t delay_ms, const char *why);
/* Variant P: crumbs for the previous-boot line. */
void diag_note_bt_disconnect(uint8_t reason);
void diag_note_stream_alt(uint8_t itf, uint8_t alt);

/* --------------------------------------------------- main-loop position */

/* Where the main loop is, kept in the RAM that survives a reset so the next
 * boot can say where the previous one stopped. Codes: see diag_where_name()
 * in diag_log.c. A code with DIAG_WHERE_RETURNED set means "returned from". */
#define DIAG_WHERE_RETURNED        0x80u
#define DIAG_WHERE_LOOP_TOP        0x01u /* port::watchdog_update(), first call of a pass */
#define DIAG_WHERE_BT_POLL         0x02u /* port::bt_transport_poll() */
#define DIAG_WHERE_BT_TASK         0x03u
#define DIAG_WHERE_USB_RECONNECT   0x04u
#define DIAG_WHERE_TUD_TASK        0x05u
#define DIAG_WHERE_WAKE_TASK       0x06u
#define DIAG_WHERE_SHORTCUT_TASK   0x07u
#define DIAG_WHERE_AUDIO_LOOP      0x08u /* returned from it = in main.cpp's interrupt_loop() */
#define DIAG_WHERE_BATT_LED        0x09u
#define DIAG_WHERE_BUTTON_CHECK    0x0Au
#define DIAG_WHERE_INQUIRY_LED     0x0Bu
#define DIAG_WHERE_BT_DISPATCH     0x21u /* bt_transport_poll: inside BTstack's packet handler */
#define DIAG_WHERE_BT_DCD_POLL     0x22u /* bt_transport_poll: bl616_dcd_poll() */
#define DIAG_WHERE_BT_RUN_LOOP     0x23u /* bt_transport_poll: BTstack run loop (timers) */
#define DIAG_WHERE_BT_YIELD        0x24u /* bt_transport_poll: taskYIELD() */

void diag_set_where(uint32_t code);
const char *diag_where_name(uint32_t code);
/* Called once per main-loop pass (from the loop-top wrapper). */
void diag_main_pass(void);

/* ------------------------------------------------------------ counters */

/* Written by the Bluetooth transport and the USB driver, read and printed by
 * the monitor every 5 s. Plain counters only grow; the monitor prints deltas. */
struct diag_counters {
    /* Bluetooth transport */
    volatile uint32_t bt_acl_in;        /* ACL packets handed to BTstack */
    volatile uint32_t bt_evt_in;        /* HCI events handed to BTstack */
    volatile uint32_t bt_acl_out;       /* ACL packets given to the controller */
    volatile uint32_t bt_acl_out_bytes; /* their payload bytes */
    volatile uint32_t bt_cmd_out;       /* HCI commands given to the controller */
    volatile uint32_t bt_send_fail;     /* bt_onchiphci_send() != 0 */
    volatile uint32_t bt_rx_drops;      /* receive ring full, packet dropped */
    volatile uint32_t bt_ring_peak;     /* receive ring bytes used, peak since the last report */
    volatile uint32_t bt_ring_size;
    volatile int32_t bt_acl_free_now;   /* BTstack free outgoing ACL slots, -1 = no connection */
    volatile int32_t bt_acl_free_min;   /* lowest since the last report, -1 = none seen */
    volatile uint32_t bt_evt_unlogged;  /* events the event log skipped (rate limit) */
    /* Main loop */
    volatile uint32_t main_max_gap_ms;  /* longest time between two passes since the last report */
    /* Hardware watchdog, read at the loop top just before each feed */
    volatile uint32_t wdt_feeds;        /* feeds (port::watchdog_update calls) */
    volatile uint32_t wdt_cnt_max;      /* highest counter value seen before a feed since the last report */
    volatile uint32_t wdt_cnt_last;     /* counter value before the latest feed */
    /* USB device controller */
    volatile uint32_t usb_setup;
    volatile uint32_t usb_bus_reset;
    volatile uint32_t usb_suspend;
    volatile uint32_t usb_resume;
    volatile uint32_t usb_armed_in[5];  /* dcd_edpt_xfer() per endpoint number */
    volatile uint32_t usb_armed_out[5];
    volatile uint32_t usb_done_in[5];   /* completions per endpoint number */
    volatile uint32_t usb_done_out[5];
    volatile uint16_t usb_set_report[256]; /* control SET_REPORT, by report ID */
    volatile uint16_t usb_out_report[256]; /* interrupt OUT (EP 0x03), by first byte */
    /* Variant P: Bluetooth fragmentation and completed-packet accounting */
    volatile uint32_t bt_acl_out_cont;    /* outgoing ACL packets with pb=0x01 (continuation fragments) */
    volatile uint32_t bt_acl_out_max;     /* largest outgoing ACL payload since the last report */
    volatile uint32_t bt_nocp_events;     /* Number Of Completed Packets events */
    volatile uint32_t bt_nocp_packets;    /* packets they returned, summed */
    /* Variant P: audio streams (SET_INTERFACE on the speaker itf 1 / mic itf 2) */
    volatile uint32_t stream_alt_ms;      /* time of the last alt-setting change on itf 1 or 2, 0 = none */
    volatile uint8_t stream_alt[3];       /* current alt per interface number 1, 2 */
    volatile uint32_t usb_iso_out_short;  /* EP1 completions shorter than the transfer armed */
    volatile uint32_t usb_iso_out_zero;   /* EP1 zero-length packets (RX0BYTE) */
    volatile uint32_t usb_iso_out_min;    /* shortest / longest EP1 completion since the last audio line */
    volatile uint32_t usb_iso_out_max;
    volatile uint32_t usb_disc_caller;    /* return address of the latest tud_disconnect()/tud_connect() caller */
};
extern struct diag_counters diag_cnt;

/* Crumbs that survive a reset (diag_log.c keeps them with the ring). */
void diag_note_bt_rx(uint8_t type, uint8_t code);       /* last packet dispatched to BTstack */
void diag_note_bt_tx(uint8_t type, uint16_t opcode);    /* last packet sent to the controller */
void diag_note_usb_isr(int inside, const uint8_t *setup); /* entry/exit with NULL; with a SETUP packet, records it */

/* dcd_bl616.c: the controller's error/status bits, read only, for the 5 s line. */
void bl616_dcd_diag_status(uint32_t *isg0, uint32_t *isg2, uint32_t *isg3, int *started, int *connected);
/* dcd_bl616.c: bounded waits that timed out, and SETUPs that found a control
 * VDMA still armed, both since boot. */
void bl616_dcd_diag_waits(uint32_t *timeouts, uint32_t *cx_preempted);
/* dcd_bl616.c, variant P: called by the monitor every 250 ms. Returns 0 while
 * the host is talking, else a DIAG_RESET_* reason to reset for, with the
 * explanation in why. */
uint32_t bl616_dcd_diag_check_silence(uint32_t now_ms, char *why, uint32_t cap);
/* dcd_bl616.c, variant P: every DCD endpoint state, the controller's FIFO,
 * VDMA and interrupt registers, and the last USB events, as log lines. */
void bl616_dcd_diag_dump(const char *why);
/* entry_bl616.cpp, variant P: one line of audio pipeline state (queue levels,
 * TinyUSB's speaker FIFO, the worker's state and stack). */
void bl616_diag_audio_state(char *buf, uint32_t cap);

/* The watchdog's counter (diag_log.c), read by the loop-top wrapper before it
 * feeds the watchdog (diag_probes.c). */
uint32_t diag_wdt_counter(void);

#ifdef __cplusplus
}
#endif

#endif
