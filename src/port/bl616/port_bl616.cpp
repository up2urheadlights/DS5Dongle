// BL616 backend for the platform abstraction layer.
//
// Target board: Haute42 Adapter 4DS (BL616, QFN40, 4 MB flash, no PSRAM). Pin and
// flash assignments live in board_haute.h. The application entry, which runs
// board_init() and starts FreeRTOS before the shared main() gets a task of its
// own, is ports/bl616_haute/main_bl616.c.

#include "../port.h"
#include "board_haute.h"

#include <cstdio>
#include <cstring>

#include "tusb.h"

extern "C" {
#include "bflb_core.h"
#include "bflb_flash.h"
#include "bflb_gpio.h"
#include "bflb_mtimer.h"
#include "bflb_wdg.h"
#include "bl616_glb.h"
#include "bl616_glb_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#if DS5_BL616_DIAG_LOG
#include "diag_log.h"
#endif
}

namespace port {

// ---------------------------------------------------------------- config store

namespace {
alignas(4) uint8_t g_config_shadow[kConfigStoragePageSize];
bool g_config_loaded = false;
static_assert(kConfigStorageSize == BOARD_FLASH_CONFIG_SIZE,
              "config record size must match its flash sector in board_haute.h");
} // namespace

const void *config_storage_read() {
    if (!g_config_loaded) {
        // bflb_flash_read rather than the XIP window, so no cache coherence
        // question arises after config_storage_write() reprograms the sector.
        if (bflb_flash_read(BOARD_FLASH_CONFIG_ADDR, g_config_shadow, sizeof(g_config_shadow)) != 0) {
            // Present erased flash so the caller's magic/CRC check fails and it
            // falls back to defaults, matching a blank Pico.
            memset(g_config_shadow, 0xff, sizeof(g_config_shadow));
        }
        g_config_loaded = true;
    }
    return g_config_shadow;
}

const void *config_storage_read_legacy() {
    // A new target: there is no earlier config location to migrate from.
    return nullptr;
}

bool config_storage_write(const void *data, size_t len) {
    if (len > kConfigStoragePageSize) return false;

    alignas(4) uint8_t page[kConfigStoragePageSize];
    memset(page, 0xff, sizeof(page));
    memcpy(page, data, len);

    // Single core, and the SDK's flash routines run from RAM with the cache
    // handled, so no other core needs parking (the Pico's flash_safe_execute).
    if (bflb_flash_erase(BOARD_FLASH_CONFIG_ADDR, kConfigStorageSize) != 0) {
        printf("[Port] config erase failed\n");
        return false;
    }
    if (bflb_flash_write(BOARD_FLASH_CONFIG_ADDR, page, sizeof(page)) != 0) {
        printf("[Port] config write failed\n");
        return false;
    }
    memcpy(g_config_shadow, page, sizeof(page));
    g_config_loaded = true;
    return true;
}

// ----------------------------------------------------------------- one-shot timer

namespace {
constexpr int kMaxTimers = 4;

// Same ownership scheme as the ESP32-S31 backend (see port_esp32s31.cpp): a
// slot is claimed by an atomic exchange, so exactly one of the arming caller,
// a cancelling caller and the timer callback ever acts on it, and the FreeRTOS
// timer is created once and never deleted, so a late cancel can never touch a
// freed handle. Callbacks run on the FreeRTOS timer service task.
struct TimerSlot {
    void (*cb)(void *);
    void *ctx;
    TimerHandle_t handle;
    StaticTimer_t storage;
    // 32-bit: the E907 has word atomics (RV32A) but no byte-wide ones.
    volatile uint32_t armed;
};

TimerSlot g_slots[kMaxTimers];

void timer_trampoline(TimerHandle_t t) {
    auto *slot = static_cast<TimerSlot *>(pvTimerGetTimerID(t));
    if (!__atomic_exchange_n(&slot->armed, 0u, __ATOMIC_SEQ_CST)) return;
    slot->cb(slot->ctx);
}
} // namespace

TimerId timer_once_ms(uint32_t delay_ms, void (*cb)(void *), void *ctx) {
    for (int i = 0; i < kMaxTimers; ++i) {
        uint32_t expected = 0;
        if (!__atomic_compare_exchange_n(&g_slots[i].armed, &expected, 1u,
                                         false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
            continue; // in use
        }
        TimerSlot &slot = g_slots[i];
        if (slot.handle == nullptr) {
            slot.handle = xTimerCreateStatic("port_oneshot", 1, pdFALSE, &slot,
                                             timer_trampoline, &slot.storage);
            if (slot.handle == nullptr) {
                __atomic_store_n(&slot.armed, 0u, __ATOMIC_SEQ_CST);
                return kNoTimer;
            }
        }
        slot.cb = cb;
        slot.ctx = ctx;
        TickType_t ticks = pdMS_TO_TICKS(delay_ms);
        if (ticks == 0) ticks = 1; // a FreeRTOS timer period must be non-zero
        // ChangePeriod also starts a dormant timer.
        if (xTimerChangePeriod(slot.handle, ticks, 0) != pdPASS) {
            __atomic_store_n(&slot.armed, 0u, __ATOMIC_SEQ_CST);
            return kNoTimer;
        }
        return static_cast<TimerId>(i + 1); // 0 is reserved for kNoTimer
    }
    return kNoTimer;
}

void timer_cancel(TimerId id) {
    if (id <= kNoTimer || id > kMaxTimers) return;
    TimerSlot &slot = g_slots[id - 1];
    if (!__atomic_exchange_n(&slot.armed, 0u, __ATOMIC_SEQ_CST)) return;
    xTimerStop(slot.handle, 0);
}

// ------------------------------------------------------------------ boot button

namespace {
struct bflb_device_s *gpio_dev() {
    static struct bflb_device_s *dev = nullptr;
    if (dev == nullptr) dev = bflb_device_get_by_name("gpio");
    return dev;
}

// The SDK's bflb_gpio_init() traps (ebreak, then spins forever) on a pin this
// package does not lead out, so every pin passes through here first.
bool pin_is_led_out(uint32_t pin) {
    return pin < 64 && GLB_GPIO_Pad_LeadOut_Sts(static_cast<uint8_t>(pin)) == SET;
}
} // namespace

// The button's polarity is decided from the pin's idle level, once.
//
// GPIO2 is the BL616 boot strap: the chip enters its ROM download mode when the
// pin is HIGH at reset, so this application only ever runs after a reset that
// saw GPIO2 low, i.e. with the button released. A board whose button connects
// GPIO2 to 3V3 (active high) therefore needs an external pull-down, which our
// internal pull-up cannot overcome: with the old fixed "active low, pull-up"
// reading, such a board reads "pressed" from the first sample, the shared code
// fires the hold gesture (clear all pairings) 1.5 s after the loop starts, and
// a real press then reads as a release. So: sample the pin with the pull-up
// first. High = nothing pulls it down = the active-low wiring the port assumed
// (unchanged behaviour). Low = an external pull-down = active-high wiring:
// switch to the internal pull-down and treat high as pressed.
namespace {
bool g_button_active_high = false;
#if DS5_BL616_DIAG_LOG
int g_button_last_level = -1;
#endif
} // namespace

bool boot_button_pressed() {
    static bool configured = false;
    if (!configured) {
        bflb_gpio_init(gpio_dev(), BOARD_BUTTON_GPIO, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN);
        bflb_mtimer_delay_us(100); // let the pull-up settle the pin
        const bool idle_high = bflb_gpio_read(gpio_dev(), BOARD_BUTTON_GPIO);
        g_button_active_high = !idle_high;
        if (g_button_active_high) {
            bflb_gpio_init(gpio_dev(), BOARD_BUTTON_GPIO, GPIO_INPUT | GPIO_PULLDOWN | GPIO_SMT_EN);
        }
        printf("[Port] button GPIO%u idle level with pull-up: %s -> reading it as active %s%s\n",
               static_cast<unsigned>(BOARD_BUTTON_GPIO), idle_high ? "high" : "LOW",
               g_button_active_high ? "high" : "low",
               g_button_active_high ? " (external pull-down; internal pull-down now)" : " (pull-up kept)");
#if DS5_BL616_DIAG_LOG
        diag_logf("[btn] t=%lu GPIO%u idle with pull-up=%d -> active %s\r\n",
                  static_cast<unsigned long>(bflb_mtimer_get_time_ms()), static_cast<unsigned>(BOARD_BUTTON_GPIO),
                  idle_high ? 1 : 0, g_button_active_high ? "high" : "low");
#endif
        configured = true;
    }
    const bool level = bflb_gpio_read(gpio_dev(), BOARD_BUTTON_GPIO);
#if DS5_BL616_DIAG_LOG
    if ((level ? 1 : 0) != g_button_last_level) {
        diag_logf("[btn] t=%lu GPIO%u raw level %d (%s)\r\n", static_cast<unsigned long>(bflb_mtimer_get_time_ms()),
                  static_cast<unsigned>(BOARD_BUTTON_GPIO), level ? 1 : 0,
                  level == g_button_active_high ? "pressed" : "released");
        g_button_last_level = level ? 1 : 0;
    }
#endif
    return level == g_button_active_high;
}

void worker_flash_safe_init() {
    // No-op: one core, so nothing else executes from flash during a write.
}

// ---------------------------------------------------------------------- system

void clocks_init() {
    // board_init() in main_bl616.c set the clocks before the shared main() ran.
}

void smps_force_pwm() {
    // No software-controlled regulator mode on this board.
}

namespace {
struct bflb_device_s *g_wdg = nullptr;
} // namespace

bool watchdog_caused_reboot() {
    GLB_RESET_RECORD_Type rec{};
    GLB_Get_Reset_Reason(&rec);
    const uint32_t raw = getreg32(GLB_BASE + GLB_RESET_STS0_OFFSET);
    // The record is sticky: nothing but this clears it, so without the clear a
    // single watchdog reset would read as one at every boot until power-off.
    GLB_Clr_Reset_Reason();
    const uint32_t after = getreg32(GLB_BASE + GLB_RESET_STS0_OFFSET);
    // Printed raw so the bit's meaning can be checked against a known reset;
    // the SDK documents the field only by name.
    printf("[Port] reset record: por=%u ext=%u pds=%u wdt=%u cpu_por=%u sys=%u cpu_sys=%u "
           "(raw 0x%08lx; after clearing 0x%08lx)\n",
           rec.reset_recorder_ana_powb, rec.reset_recorder_ext_rst_n, rec.reset_recorder_pds_reset,
           rec.reset_recorder_wdt_rst_n, rec.reset_recorder_cpu_porst_n,
           rec.reset_recorder_sys_reset_n, rec.reset_recorder_cpu_sys_rstreq_n,
           static_cast<unsigned long>(raw), static_cast<unsigned long>(after));
    return rec.reset_recorder_wdt_rst_n != 0;
}

void watchdog_enable(uint32_t timeout_ms) {
    g_wdg = bflb_device_get_by_name(BFLB_NAME_WDT);
    if (g_wdg == nullptr) {
        printf("[Port] watchdog device not found\n");
        return;
    }
    struct bflb_wdg_config_s cfg = {};
    cfg.clock_source = WDG_CLKSRC_1K; // 1 kHz: comp_val is in milliseconds
    cfg.clock_div = 0;
    cfg.comp_val = timeout_ms > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(timeout_ms);
    cfg.mode = WDG_MODE_RESET;
    bflb_wdg_init(g_wdg, &cfg);
    bflb_wdg_start(g_wdg);
}

void watchdog_update() {
    if (g_wdg != nullptr) bflb_wdg_reset_countervalue(g_wdg);
}

[[noreturn]] void reboot() {
    GLB_SW_System_Reset();
    while (true) {
    }
}

bool has_bootloader_reboot() {
    // The ROM USB bootloader is reached by holding the button while plugging
    // in. No software route to it has been tried on this board.
    return false;
}

[[noreturn]] void reboot_to_bootloader() {
    printf("[Port] no software route to the BL616 ROM bootloader; hold the button "
           "while plugging in\n");
    reboot();
}

// bt_transport_init()/bt_transport_poll() live in bt_transport_bl616.cpp.

// ------------------------------------------------------------------------ GPIO

// Pin numbers go up to 34 on the BL616 die; most are not led out on QFN40 and
// gpio_init_output() refuses those instead of trapping.
const uint32_t kNumGpioPins = 35;

void gpio_init_output(uint32_t pin) {
    if (!pin_is_led_out(pin)) {
        printf("[Port] GPIO%lu is not led out on this package; ignored\n", (unsigned long) pin);
        return;
    }
    bflb_gpio_init(gpio_dev(), static_cast<uint8_t>(pin), GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    gpio_write(pin, false);
}

void gpio_write(uint32_t pin, bool level) {
    if (!pin_is_led_out(pin)) return;
    if (level) {
        bflb_gpio_set(gpio_dev(), static_cast<uint8_t>(pin));
    } else {
        bflb_gpio_reset(gpio_dev(), static_cast<uint8_t>(pin));
    }
}

// ------------------------------------------------------------------------- LED

void led_init() {
    bflb_gpio_init(gpio_dev(), BOARD_STATUS_LED_GPIO_A, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    bflb_gpio_init(gpio_dev(), BOARD_STATUS_LED_GPIO_B, GPIO_OUTPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    led_set(false);
}

void led_set(bool on) {
    // Active low: driving the pin low lights the LED.
    if (on) {
        bflb_gpio_reset(gpio_dev(), BOARD_STATUS_LED_GPIO_A);
        bflb_gpio_reset(gpio_dev(), BOARD_STATUS_LED_GPIO_B);
    } else {
        bflb_gpio_set(gpio_dev(), BOARD_STATUS_LED_GPIO_A);
        bflb_gpio_set(gpio_dev(), BOARD_STATUS_LED_GPIO_B);
    }
}

// Single-colour LED: colour requests are accepted and ignored, as on Pico2W.
void led_set_on_color(uint8_t, uint8_t, uint8_t) {}
void led_reset_on_color() {}

// ---------------------------------------------------------------------------- USB

const uint8_t kUsbRhPort = 0;

// The BL616 USB controller is high-speed with an on-chip HS PHY, and the
// DualSense it presents is a high-speed device.
const uint8_t kUsbSpeed = TUSB_SPEED_HIGH;

void board_init() {
    // Nothing before tusb_init(): the USB clock is set up by board_init() in
    // main_bl616.c and recovered right before the controller starts, after
    // Bluetooth is up -- the order the controller needs on this chip (starting
    // USB before Bluetooth init lost the port mid-session on the Haute).
}

void board_init_after_tusb() {}

size_t usb_get_serial(uint16_t *desc_str, size_t max_chars) {
    // The flash's unique ID, as TinyUSB's BSP does on the RP2350.
    uint8_t id[8] = {};
    if (bflb_flash_get_unique_id(id, sizeof(id)) != 0) return 0;
    static const char hex[] = "0123456789ABCDEF";
    size_t n = 0;
    for (size_t i = 0; i < sizeof(id) && n + 2 <= max_chars; ++i) {
        desc_str[n++] = static_cast<uint16_t>(hex[id[i] >> 4]);
        desc_str[n++] = static_cast<uint16_t>(hex[id[i] & 0x0f]);
    }
    return n;
}

// ---------------------------------------------------------------------- worker

namespace {
StaticTask_t g_worker_tcb;
void (*g_worker_entry)() = nullptr;

void worker_thunk(void *) {
    if (g_worker_entry) g_worker_entry();
    vTaskDelete(nullptr); // entry is not expected to return
}
} // namespace

// Main-loop priority, set by main_bl616.c when it creates the main task.
extern "C" const UBaseType_t g_ds5_main_task_priority;

void launch_worker(void (*entry)(), uint32_t *stack, size_t stack_bytes) {
    g_worker_entry = entry;
    // One core: the "second core" is a task at the main loop's own priority,
    // so FreeRTOS time-slices the two busy loops and port::delay_us() yields
    // between them. The caller's static stack is used as on the other targets.
    xTaskCreateStatic(worker_thunk, "audio_worker",
                      stack_bytes / sizeof(StackType_t), nullptr,
                      g_ds5_main_task_priority,
                      reinterpret_cast<StackType_t *>(stack), &g_worker_tcb);
}

} // namespace port
