# BL616 port: Haute42 Adapter 4DS

A port of the shared firmware to the Bouffalo BL616, targeting the Haute42
Adapter 4DS (BL616 in QFN40, 4 MB flash, no PSRAM). It uses the same port layer
as the ESP32-S31 target (`src/port/`), with the shared sources unchanged apart
from the `PORT_ONDIE_BT_CONTROLLER` guards the two on-die-controller targets
share.

## Status

Stage 1 of the port:

| Part | State |
| --- | --- |
| Bluetooth host | BTstack over the BL616 on-chip controller (`src/port/bl616/bt_transport_bl616.cpp`) |
| Bond store | BTstack two-bank TLV in its own flash region (`board_haute.h`) |
| USB to the PC | **Not yet.** TinyUSB is built with its no-op template driver; a BL616 device-controller driver is the next stage |
| Debug log | Debug build only (`-DebugLog`): USB serial port (CherryUSB CDC-ACM, VID `FFFF` PID `D012`) mirroring the console |
| Audio | Built and running (Opus, resampler); untestable until USB exists |

CI builds the production image (no log port). Until the USB driver exists the
PC sees nothing from it; the debug build pairs the DualSense and shows its
input in the log.

## How it differs from the other targets

- **One core.** The BL616 has a single E907. The audio worker that runs on a
  second core elsewhere is a FreeRTOS task at the main loop's priority, and the
  two busy loops share the core by time-slicing (`port::delay_us()` and the
  transport poll yield).
- **The controller is not H4.** Bouffalo's controller library hands events back
  pre-split (command complete, command status, events, LE meta, ACL) without H4
  headers; the transport rebuilds them for BTstack, as Bouffalo's own host does.
- **Flash.** The partition table is the SDK's BL616 default, the layout the
  adapter's own firmware uses. This firmware keeps its data only in the
  `media` partition, which that firmware leaves blank: BTstack's bonds at
  `0x378000`/`0x37A000`, the config record at `0x37C000`. It never reads or
  writes PSM, KEY, DATA or factory, so a bond another firmware left in PSM is
  never offered to the pad.
- **Pins.** QFN40 leads out only GPIO 0-3, 10-17, 20-22 and 27-30. The SDK traps
  on any other pin, so the port never touches GPIO32/33 (the dev-kit USB pin
  setup; the USB PHY needs none) and `gpio_init_output()` refuses unbonded
  pins. LED: GPIO27 and GPIO28, active low. Button: GPIO2, active low; held
  while plugging in, it enters the ROM USB bootloader.
- **USB after Bluetooth.** USB starts only once Bluetooth is up, with the USB
  clock recovered first; starting it earlier lost the port mid-init on this
  board.

## Build

Windows, with the pinned dependencies (see `.github/workflows/build-firmware.yml`):

```powershell
git submodule update --init lib/btstack lib/opus lib/WDL
powershell -ExecutionPolicy Bypass -File ports\bl616_haute\build.ps1 `
    -SdkPath <bl_mcu_sdk @ BL_SDK_REF> `
    -ToolchainBin <toolchain_gcc_t-head_windows @ BL_TOOLCHAIN_REF>\bin `
    -TinyUsbPath <tinyusb @ TINYUSB_REF> [-DebugLog]
```

Use a short path (for example `subst W: <repo>`): the SDK builds with
`-save-temps`, and deep object paths can pass Windows' 260-character limit.

The result is one image, `build\ds5-bridge-haute-merged.bin`
(`ds5-bridge-haute-debug-merged.bin` with `-DebugLog`): boot2 at
`0x0`, the partition table at `0xE000` and `0xF000`, the firmware at `0x10000`,
gaps `0xFF`. Write it at `0x0` in one session.

## Flash

Hold the button while plugging the adapter in (ROM USB bootloader, a serial
port `VID_349B&PID_6160`), then write the merged image at `0x0` with
Bouffalo's `BLFlashCommand` (in the SDK, `tools/bflb_tools/bouffalo_flash_cube`):

```powershell
BLFlashCommand.exe --interface=uart --chipname=bl616 --port=COMx --baudrate=2000000 `
    --flash --write --start=0x0 --file=ds5-bridge-haute-merged.bin
```

Do not pass any eFuse option. Back the flash up first (`--read --start=0x0
--end=0x3FFFFF`) if the adapter's own firmware is to be restored later.

## Pairing

As on the other targets: with no pad stored the adapter searches at power-up;
otherwise one click of the button starts a search. Put the DualSense in pairing
mode (hold Create and PS). Holding the button for 1.5 s clears all stored
pairings.

A pad found by a search is in pairing mode, so the firmware drops any key it
holds for that pad before connecting, and a pad that rejects a stored key
(authentication failure 0x05 or 0x06) has that one key dropped. Either way only
that pad's key goes; other pads, the blacklist and the settings are untouched.
