//
// Created by awalol on 2026/4/30.
//

#include "wake.h"

#ifdef ENABLE_WAKE_HID

#include <cstdio>
#include <cstring>

#include "tusb.h"
#include "pico/sync.h"
#include "pico/time.h"

#define WAKE_KBD_INSTANCE     1
#define WAKE_KEYCODE_F15      0x68
#define WAKE_SETTLE_US        20000
#define WAKE_KEY_HOLD_US      30000
#define WAKE_KEY_UP_SETTLE_US 10000
#define WAKE_REQUEST_TIMEOUT_US 5000000

typedef enum {
    WAKE_IDLE,
    WAKE_PENDING_PRESS,
    WAKE_REQUESTED,
    WAKE_KEY_DOWN,
    WAKE_KEY_UP_SENT,
    WAKE_DONE,
} wake_state_t;

static critical_section_t wake_cs;
static volatile bool host_suspended = false;
static volatile bool host_resumed_event = false;
static wake_state_t state = WAKE_IDLE;
static uint64_t state_entered_us = 0;
static uint8_t prev_ps_bit = 1;

static void enter_state(wake_state_t s) {
    state = s;
    state_entered_us = time_us_64();
}

void wake_init(void) {
    critical_section_init(&wake_cs);
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    host_suspended = true;
    if (state == WAKE_IDLE || state == WAKE_DONE) {
        state = WAKE_PENDING_PRESS;
        state_entered_us = time_us_64();
        prev_ps_bit = 1;
    }
}

extern "C" void tud_resume_cb(void) {
    host_suspended = false;
    host_resumed_event = true;
}

void wake_on_bt_input(const uint8_t *hid_input, uint16_t len) {
    if (len < 10) return;
    // DualSense BT 0x31 input report layout (after main.cpp's `data + 3` skip):
    //   byte 7 low nibble: D-pad direction (0x08 idle); high nibble: face buttons
    //   byte 8: L1, R1, L2 click, R2 click, share, options, L3, R3
    //   byte 9: PS (bit 0), touchpad-click (bit 1), mute (bit 2)
    // The PS bit is at byte 9, not byte 8 (which is L1). Verified by capturing
    // per-button report deltas with a diagnostic firmware.
    const uint8_t ps_now = hid_input[9] & 0x01;

    critical_section_enter_blocking(&wake_cs);
    const bool edge = (prev_ps_bit == 0 && ps_now == 1);
    prev_ps_bit = ps_now;
    const bool act = edge && state == WAKE_PENDING_PRESS;
    if (act) {
        state = WAKE_REQUESTED;
        state_entered_us = time_us_64();
    }
    critical_section_exit(&wake_cs);

    if (act) {
        // Safe no-op if the host did not enable remote wakeup. We still proceed
        // through the FSM and try to send the key once the host resumes.
        tud_remote_wakeup();
    }
}

void wake_on_bt_disconnect(void) {
    critical_section_enter_blocking(&wake_cs);
    state = WAKE_IDLE;
    prev_ps_bit = 1;
    critical_section_exit(&wake_cs);
}

void wake_task(void) {
    const uint64_t now = time_us_64();

    critical_section_enter_blocking(&wake_cs);
    const wake_state_t s = state;
    const uint64_t entered = state_entered_us;
    critical_section_exit(&wake_cs);

    switch (s) {
        case WAKE_IDLE:
        case WAKE_PENDING_PRESS:
        case WAKE_DONE:
            return;

        case WAKE_REQUESTED: {
            if (host_resumed_event || !host_suspended) {
                host_resumed_event = false;
                if (now - entered < WAKE_SETTLE_US) return;
                if (!tud_hid_n_ready(WAKE_KBD_INSTANCE)) return;
                uint8_t rpt[8] = { 0, 0, WAKE_KEYCODE_F15, 0, 0, 0, 0, 0 };
                if (tud_hid_n_report(WAKE_KBD_INSTANCE, 0, rpt, sizeof(rpt))) {
                    critical_section_enter_blocking(&wake_cs);
                    enter_state(WAKE_KEY_DOWN);
                    critical_section_exit(&wake_cs);
                }
            } else if (now - entered > WAKE_REQUEST_TIMEOUT_US) {
                critical_section_enter_blocking(&wake_cs);
                enter_state(WAKE_DONE);
                critical_section_exit(&wake_cs);
            }
            return;
        }

        case WAKE_KEY_DOWN: {
            if (now - entered < WAKE_KEY_HOLD_US) return;
            if (!tud_hid_n_ready(WAKE_KBD_INSTANCE)) return;
            uint8_t up[8] = { 0 };
            if (tud_hid_n_report(WAKE_KBD_INSTANCE, 0, up, sizeof(up))) {
                critical_section_enter_blocking(&wake_cs);
                enter_state(WAKE_KEY_UP_SENT);
                critical_section_exit(&wake_cs);
            }
            return;
        }

        case WAKE_KEY_UP_SENT: {
            if (now - entered < WAKE_KEY_UP_SETTLE_US) return;
            critical_section_enter_blocking(&wake_cs);
            enter_state(WAKE_DONE);
            critical_section_exit(&wake_cs);
            return;
        }
    }
}

#endif // ENABLE_WAKE_HID
