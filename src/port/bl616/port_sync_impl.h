#pragma once

#include <cstdint>

extern "C" {
#include "bflb_irq.h"
}

namespace port {

// BL616 is single-core (one E907), so masking interrupts on this core is full
// mutual exclusion -- the spinlock half of the pico-sdk contract has nothing
// to exclude. bflb_irq_save() nests safely, which taskENTER_CRITICAL() would
// too, but it is also callable from an ISR and before the scheduler starts.
class CriticalSection {
public:
    void init() {}
    void enter() { flags_ = bflb_irq_save(); }
    void exit() { bflb_irq_restore(flags_); }

private:
    uintptr_t flags_ = 0;
};

using irq_state_t = uintptr_t;

inline irq_state_t irq_save() { return bflb_irq_save(); }
inline void irq_restore(irq_state_t state) { bflb_irq_restore(state); }

} // namespace port
