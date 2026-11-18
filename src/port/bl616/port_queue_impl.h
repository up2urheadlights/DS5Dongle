#pragma once

#include <cstddef>
#include <cstdlib>

extern "C" {
#include "FreeRTOS.h"
#include "queue.h"
}

namespace port {

// Same shape as the ESP32-S31 backend: a FreeRTOS queue, with the ISR-safe
// variant chosen at runtime because pico_util queues are callable from either
// context, and a scratch slot so a null destination means "discard oldest"
// (FreeRTOS asserts on a null buffer). See esp32s31/port_queue_impl.h.
class Queue {
public:
    void init(size_t element_size, size_t count) {
        q_ = xQueueCreate(count, element_size);
        configASSERT(q_ != nullptr);
        scratch_ = malloc(element_size);
        configASSERT(scratch_ != nullptr);
    }

    bool try_add(const void *src) {
        if (xPortIsInsideInterrupt()) {
            BaseType_t woken = pdFALSE;
            const BaseType_t ok = xQueueSendFromISR(q_, src, &woken);
            portYIELD_FROM_ISR(woken);
            return ok == pdTRUE;
        }
        return xQueueSend(q_, src, 0) == pdTRUE;
    }

    bool try_remove(void *dst) {
        void *dest = (dst != nullptr) ? dst : scratch_;
        if (xPortIsInsideInterrupt()) {
            BaseType_t woken = pdFALSE;
            const BaseType_t ok = xQueueReceiveFromISR(q_, dest, &woken);
            portYIELD_FROM_ISR(woken);
            return ok == pdTRUE;
        }
        return xQueueReceive(q_, dest, 0) == pdTRUE;
    }

    size_t level() {
        return xPortIsInsideInterrupt() ? uxQueueMessagesWaitingFromISR(q_)
                                        : uxQueueMessagesWaiting(q_);
    }

    bool is_full() { return uxQueueSpacesAvailable(q_) == 0; }
    bool is_empty() { return level() == 0; }

private:
    QueueHandle_t q_ = nullptr;
    void *scratch_ = nullptr;
};

} // namespace port
