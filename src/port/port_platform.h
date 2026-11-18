#pragma once

// Platform selection. Kept free of C++ so C translation units (ram_mem.c) can
// pull in port_mem.h for the placement macros without dragging in the rest.
//
// Exactly one PORT_PLATFORM_* macro is defined, driven by what the build system
// already tells us.

#if defined(PICO_ON_DEVICE) || defined(PICO_SDK_VERSION_MAJOR)
#define PORT_PLATFORM_PICO 1
#elif defined(ESP_PLATFORM)
#define PORT_PLATFORM_ESP32S31 1
#elif defined(BL616)
// The Bouffalo SDK (bl_mcu_sdk) defines the chip macro on every translation unit.
#define PORT_PLATFORM_BL616 1
#else
#error "port: unknown platform; expected pico-sdk, ESP-IDF or the Bouffalo SDK"
#endif

// Targets whose Bluetooth controller is on the same die and reached through a
// vendor HCI shim (ESP32-S31 VHCI, BL616 onchip HCI) rather than the Pico's
// CYW43 over SPI. They share the same BTstack integration: a polled embedded
// run loop, and BTstack left to answer the HCI requests it already answers
// (a second reply from the firmware is a duplicate those controllers reject or
// assert on). Shared code tests this instead of listing targets.
#if defined(PORT_PLATFORM_ESP32S31) || defined(PORT_PLATFORM_BL616)
#define PORT_ONDIE_BT_CONTROLLER 1
#endif
