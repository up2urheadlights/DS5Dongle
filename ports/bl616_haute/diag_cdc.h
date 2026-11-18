#pragma once

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Debug build only: console mirrored to a USB CDC-ACM port (VID FFFF PID D012).
 * See diag_cdc.c. */
void diag_cdc_start_task(UBaseType_t priority);
/* USB starts once this is called (Bluetooth up), or after a 20 s fallback. */
void diag_cdc_request_usb(void);

#ifdef __cplusplus
}
#endif
