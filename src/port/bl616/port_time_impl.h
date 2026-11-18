#pragma once

#include <cstdint>

extern "C" {
#include "bflb_mtimer.h"
#include "FreeRTOS.h"
#include "task.h"
}

namespace port {

inline uint64_t now_us() { return bflb_mtimer_get_time_us(); }
inline uint32_t now_us32() { return static_cast<uint32_t>(bflb_mtimer_get_time_us()); }
inline uint32_t now_ms() { return static_cast<uint32_t>(bflb_mtimer_get_time_us() / 1000); }

inline void delay_ms(uint32_t ms) {
    // Before the scheduler runs (or with it suspended) a task delay is not
    // available; busy-wait then, as the Pico build always does.
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        bflb_mtimer_delay_ms(ms);
        return;
    }
    const uint32_t tick_ms = portTICK_PERIOD_MS;
    if (ms >= tick_ms) {
        vTaskDelay(ms / tick_ms);
        ms %= tick_ms;
    }
    if (ms) bflb_mtimer_delay_us(ms * 1000);
}

// Single core: the shared code's spin-waits (the audio worker's 10 us idle
// wait, above all) would otherwise hold the CPU for a whole tick against the
// main loop, which runs at the same priority. Yield after the wait so an
// equal-priority task gets the core. Tasks of higher priority preempt anyway.
inline void delay_us(uint32_t us) {
    bflb_mtimer_delay_us(us);
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) taskYIELD();
}

inline void spin_hint() { __asm__ __volatile__("nop"); }

} // namespace port
