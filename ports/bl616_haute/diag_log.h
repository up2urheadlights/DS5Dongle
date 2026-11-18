/* Debug-build log for the BL616 Haute target; see diag_log.c. */
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
 * bytes (the first read after boot starts at the oldest byte still held).
 * Returns the payload length. */
uint16_t diag_log_fill_report(uint8_t *buffer, uint16_t reqlen);

/* Starts the 5 s heartbeat line (uptime, free heap, bytes logged). */
void diag_log_start_heartbeat(unsigned long priority);

#ifdef __cplusplus
}
#endif

#endif
