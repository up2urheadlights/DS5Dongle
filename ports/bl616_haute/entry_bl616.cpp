// Application entry for the BL616 Haute target, called from main_bl616.c.
//
// The shared firmware keeps a conventional int main() (src/main.cpp) that runs
// the USB/BT service loop and never returns. The SDK's startup calls main()
// before any scheduler exists, while the Bluetooth controller library needs
// FreeRTOS running, so the SDK's main() lands here: bring the board up, run the
// C++ constructors, start FreeRTOS, and give the shared main() a task of its
// own. The build renames the shared one to ds5_shared_main (CMakeLists.txt)
// rather than touching it.

#include <cstdio>

extern "C" {
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "rfparam_adapter.h"
}

#if DS5_BL616_DIAG_CDC
#include "diag_cdc.h"
#endif

extern int ds5_shared_main();

// The SDK links with -nostartfiles, so crtbegin's __dso_handle (the key
// __cxa_atexit files static destructors under) is absent. Destructors never
// run on this firmware; the symbol only has to exist.
extern "C" {
void *__dso_handle = &__dso_handle;
}

// Main loop and audio worker share this priority so FreeRTOS time-slices the
// two busy loops (port_bl616.cpp launch_worker). Everything that blocks -- the
// Bluetooth controller (configMAX_PRIORITIES - 1), the timer service and the
// debug log task -- runs above it and preempts.
extern "C" const UBaseType_t g_ds5_main_task_priority = 4;

namespace {
constexpr uint32_t kMainStackWords = 4096; // 16 KB: BTstack callbacks run on it
StackType_t g_main_stack[kMainStackWords];
StaticTask_t g_main_tcb;

void main_task(void *) {
    ds5_shared_main();
    vTaskDelete(nullptr); // not expected to return
}

// The SDK's startup (start.S: SystemInit, start_load, main) copies .data and
// clears .bss but never runs the C++ constructors the linker script collects
// in .preinit_array/.init_array. The shared firmware has non-trivial globals
// (the WDL resampler, std containers, the iostream objects), so run them here,
// after board_init() has set up the heap some of them allocate from.
extern "C" {
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
}

void run_constructors() {
    for (auto *f = __preinit_array_start; f != __preinit_array_end; ++f) (*f)();
    for (auto *f = __init_array_start; f != __init_array_end; ++f) (*f)();
}
} // namespace

extern "C" void ds5_bl616_bt_ready(void) {
#if DS5_BL616_DIAG_CDC
    diag_cdc_request_usb();
#endif
}

extern "C" void vApplicationMallocFailedHook(void) {
    printf("[BL616] malloc failed\r\n");
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char *name) {
    printf("[BL616] stack overflow in task %s\r\n", name);
    while (true) {
    }
}

extern "C" int ds5_bl616_entry(void) {
    board_init();
    printf("\r\n[BL616] DS5Dongle for Haute Adapter 4DS, build " DS5_FIRMWARE_VERSION "\r\n");
    run_constructors();

#if DS5_BL616_DIAG_CDC
    diag_cdc_start_task(configMAX_PRIORITIES - 2);
#endif

    // RF calibration, before the scheduler and before the controller starts,
    // as Bouffalo's own Bluetooth bring-up orders it.
    const int rf_rc = rfparam_init(0, nullptr, 0);
    printf("[BL616] rfparam_init rc=%d\r\n", rf_rc);
    if (rf_rc != 0) {
        // Without RF calibration the radio is unusable. Start nothing else, but
        // keep the scheduler (and in a debug build the log port) running so the
        // failure can be read.
        printf("[BL616] RF init failed; Bluetooth not started\r\n");
    } else {
        xTaskCreateStatic(main_task, "ds5_main", kMainStackWords, nullptr,
                          g_ds5_main_task_priority, g_main_stack, &g_main_tcb);
    }

    vTaskStartScheduler();
    while (true) {
    }
}
