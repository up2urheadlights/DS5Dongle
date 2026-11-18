#pragma once

#include "port_platform.h"

// Placement attributes for hot code and data.
//
// On RP2350 these map to the pico-sdk's XIP-avoiding sections; on ESP32-S31
// they map to IRAM/DRAM. Both platforms are keeping the same promise: this
// symbol must not be fetched over a flash-cache miss on the audio/BT hot path.

#if defined(PORT_PLATFORM_PICO)

#include "pico.h"

#define PORT_FAST_FUNC(fn) __not_in_flash_func(fn)
#define PORT_FAST_DATA __not_in_flash("port_fast")

#elif defined(PORT_PLATFORM_ESP32S31)

#include "esp_attr.h"

#define PORT_FAST_FUNC(fn) IRAM_ATTR fn
#define PORT_FAST_DATA DRAM_ATTR

#elif defined(PORT_PLATFORM_BL616)

// The SDK linker script copies .tcm_code.* into RAM at boot (ATTR_TCM_SECTION
// in compiler_ld.h). Data is already in RAM on this part -- there is no PSRAM
// on the Haute's BL616 and .data/.bss never live in flash -- so the data macro
// needs no section.
#define PORT_FAST_FUNC(fn) __attribute__((section(".tcm_code.port_fast"))) fn
#define PORT_FAST_DATA

#endif
