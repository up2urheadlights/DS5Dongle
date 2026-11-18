#pragma once

// Haute42 "Adapter 4DS" (BL616, QFN40, 4 MB flash, no PSRAM) pin and flash map.
//
// Measured on the hardware, not taken from a schematic (none is published):
//  - Status LED: one LED, active low. The firmware drives both GPIO27 and
//    GPIO28 because one of the two lights it; driving the other is harmless.
//  - Side button: GPIO2, active low. Holding it while plugging in enters the
//    BL616 ROM USB bootloader (VID 349B PID 6160), so the button is also the
//    boot strap and can be read freely once the application runs.
//  - QFN40 leads out only GPIO 0-3, 10-17, 20-22 and 27-30. GPIO32/33 (the
//    bl616dk USB pin setup) do not exist on this package, and the SDK's pad
//    check traps (ebreak, then spins) on any pin that is not led out, so this
//    port never configures them. The BL616 USB PHY needs no pin mux.
//  - UART0 console on GPIO21 (TX) / GPIO22 (RX), as board_init() sets it up.

#define BOARD_STATUS_LED_GPIO_A 27
#define BOARD_STATUS_LED_GPIO_B 28
#define BOARD_BUTTON_GPIO 2

// Flash map. The partition table (boot2 @0x0, table @0xE000/0xF000) is the
// SDK's bl616 default, the same layout the adapter's own firmware uses:
//   FW 0x010000 (2 MB, A slot) / 0x210000 (B slot), media 0x378000 (452 KB),
//   PSM 0x3E9000, KEY 0x3F1000, DATA 0x3F3000, factory 0x3F8000.
// This firmware stores its data ONLY inside "media", which the adapter's own
// firmware leaves blank, and never reads or writes PSM/KEY/DATA/factory. PSM
// is where other BL616 firmwares (EasyFlash/LittleFS) keep their Bluetooth
// bonds; never reading it means a bond left by another firmware cannot be
// offered to a pad that has since paired with this one.
//
// BTstack's bond store (link keys + the firmware's pairing blacklist) is a
// two-bank TLV with its own "BTstack" header magic, so leftover bytes from any
// other firmware fail the header check and the bank is erased, not parsed.
// The config record has its own sector, so dropping a key or wiping the bond
// store never touches settings.
#define BOARD_FLASH_BTSTACK_BANK0 0x378000u
#define BOARD_FLASH_BTSTACK_BANK1 0x37A000u
#define BOARD_FLASH_BTSTACK_BANK_SIZE 0x2000u
#define BOARD_FLASH_CONFIG_ADDR 0x37C000u
#define BOARD_FLASH_CONFIG_SIZE 0x1000u
