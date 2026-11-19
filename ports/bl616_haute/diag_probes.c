/*
 * Debug build only: main-loop position markers for the BL616 Haute target.
 *
 * The shared main loop (src/main.cpp) is not edited. Instead every call it
 * makes is wrapped at link time (-Wl,--wrap=<symbol>, CMakeLists.txt), the
 * same mechanism usb_hid_bl616.cpp uses for TinyUSB. Each wrapper records
 * "in X" before the call and "returned from X" after it in the crumb that
 * survives a reset (diag_log.c), so both the stall monitor and the next boot
 * can say where the loop stopped. port::watchdog_update(), the first call of
 * every pass, also counts the pass and the time between passes, and reads the
 * watchdog counter just before it is fed.
 *
 * The symbols are the C++ names the shared code links against (checked with
 * nm on main.cpp.obj). interrupt_loop() is defined in main.cpp itself and so
 * cannot be wrapped: "returned from audio_loop" covers it. A wrapped function
 * called from somewhere other than the main loop (tud_task from usb.cpp) sets
 * the marker too, which only makes the position more precise.
 */
#include <stdbool.h>
#include <stdint.h>

#include "diag_log.h"

#define DIAG_WRAP_VOID(sym, code)              \
    void __real_##sym(void);                   \
    void __wrap_##sym(void);                   \
    void __wrap_##sym(void)                    \
    {                                          \
        diag_set_where(code);                  \
        __real_##sym();                        \
        diag_set_where((code) | DIAG_WHERE_RETURNED); \
    }

/* port::watchdog_update(): the loop top. */
void __real__ZN4port15watchdog_updateEv(void);
void __wrap__ZN4port15watchdog_updateEv(void);
void __wrap__ZN4port15watchdog_updateEv(void)
{
    diag_main_pass();
    /* The watchdog counter just before this feed: with the loop passing every
     * few ms it should read a few counts (1 kHz clock). Always 0 means the
     * counter is not running, so the watchdog could never fire. */
    const uint32_t cnt = diag_wdt_counter();
    diag_cnt.wdt_cnt_last = cnt;
    if (cnt > diag_cnt.wdt_cnt_max) {
        diag_cnt.wdt_cnt_max = cnt;
    }
    diag_cnt.wdt_feeds++;
    diag_set_where(DIAG_WHERE_LOOP_TOP);
    __real__ZN4port15watchdog_updateEv();
    diag_set_where(DIAG_WHERE_LOOP_TOP | DIAG_WHERE_RETURNED);
}

/* port::bt_transport_poll() sets its own finer positions inside
 * (bt_transport_bl616.cpp); this marks entry and return. */
DIAG_WRAP_VOID(_ZN4port17bt_transport_pollEv, DIAG_WHERE_BT_POLL)
DIAG_WRAP_VOID(_Z7bt_taskv, DIAG_WHERE_BT_TASK)
DIAG_WRAP_VOID(_Z18usb_reconnect_taskv, DIAG_WHERE_USB_RECONNECT)
DIAG_WRAP_VOID(_Z9wake_taskv, DIAG_WHERE_WAKE_TASK)
DIAG_WRAP_VOID(_Z20button_shortcut_taskv, DIAG_WHERE_SHORTCUT_TASK)
DIAG_WRAP_VOID(_Z10audio_loopv, DIAG_WHERE_AUDIO_LOOP)
DIAG_WRAP_VOID(_Z16battery_led_tickv, DIAG_WHERE_BATT_LED)
DIAG_WRAP_VOID(_Z12button_checkv, DIAG_WHERE_BUTTON_CHECK)
DIAG_WRAP_VOID(_Z16bt_inquiring_ledv, DIAG_WHERE_INQUIRY_LED)

/* Variant P: who takes the device off the bus. bt.cpp hides the USB device on
 * every Bluetooth disconnection (unless wake is on or the host is suspended),
 * and usb.cpp, main.cpp and wake.cpp disconnect it too; from the host's side
 * each of them looks like "the adapter stopped answering". The caller's return
 * address can be looked up in the build's .map or with addr2line. */
#include "bflb_mtimer.h"

bool __real_tud_disconnect(void);
bool __wrap_tud_disconnect(void);
bool __wrap_tud_disconnect(void)
{
    const uint32_t ra = (uint32_t)(uintptr_t)__builtin_return_address(0);
    diag_cnt.usb_disc_caller = ra;
    diag_logf("[usb] t=%lu tud_disconnect() called from 0x%08lx: D+ pull-up off, the host sees the device removed\r\n",
              (unsigned long)bflb_mtimer_get_time_ms(), (unsigned long)ra);
    return __real_tud_disconnect();
}

bool __real_tud_connect(void);
bool __wrap_tud_connect(void);
bool __wrap_tud_connect(void)
{
    const uint32_t ra = (uint32_t)(uintptr_t)__builtin_return_address(0);
    diag_cnt.usb_disc_caller = ra;
    diag_logf("[usb] t=%lu tud_connect() called from 0x%08lx\r\n", (unsigned long)bflb_mtimer_get_time_ms(),
              (unsigned long)ra);
    return __real_tud_connect();
}

/* tud_task() is an inline wrapper around tud_task_ext() (TinyUSB usbd.h). */
void __real_tud_task_ext(uint32_t timeout_ms, bool in_isr);
void __wrap_tud_task_ext(uint32_t timeout_ms, bool in_isr);
void __wrap_tud_task_ext(uint32_t timeout_ms, bool in_isr)
{
    diag_set_where(DIAG_WHERE_TUD_TASK);
    __real_tud_task_ext(timeout_ms, in_isr);
    diag_set_where(DIAG_WHERE_TUD_TASK | DIAG_WHERE_RETURNED);
}
