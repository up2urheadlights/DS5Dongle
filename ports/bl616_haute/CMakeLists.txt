# Bouffalo SDK (bl_mcu_sdk) project for the BL616 Haute Adapter 4DS target.
#
# Build with build.ps1 (Windows) or build.sh (Linux), which set the SDK,
# toolchain and TinyUSB paths and merge the four flash regions into one image.
# The Pico and ESP32-S31 builds are unaffected by anything here.
#
# Inputs (environment):
#   BL_SDK_BASE   Bouffalo bl_mcu_sdk checkout (pinned in build-firmware.yml)
#   TINYUSB_PATH  TinyUSB checkout (the same ref the Pico build pins)
#   DS5_VERSION   firmware version string (optional, "dev" otherwise)
#   DS5_BL616_DIAG_LOG  1 = debug build: the console log, readable over USB as a
#                       HID feature report (diag_log.c)

cmake_minimum_required(VERSION 3.15)

find_package(bouffalo_sdk REQUIRED HINTS $ENV{BL_SDK_BASE})

set(REPO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(SRC_DIR "${REPO_DIR}/src")
set(BTSTACK_ROOT "${REPO_DIR}/lib/btstack")
set(OPUS_DIR "${REPO_DIR}/lib/opus")
set(WDL_DIR "${REPO_DIR}/lib/WDL/WDL")

if(NOT DEFINED ENV{TINYUSB_PATH} OR NOT EXISTS "$ENV{TINYUSB_PATH}/src/tusb.c")
    message(FATAL_ERROR "TINYUSB_PATH must point at a TinyUSB checkout (got '$ENV{TINYUSB_PATH}')")
endif()
file(TO_CMAKE_PATH "$ENV{TINYUSB_PATH}" TINYUSB_DIR)
foreach(dir "${BTSTACK_ROOT}/src" "${OPUS_DIR}/include" "${WDL_DIR}")
    if(NOT EXISTS "${dir}")
        message(FATAL_ERROR "${dir} is missing; run: git submodule update --init lib/btstack lib/opus lib/WDL")
    endif()
endforeach()

if(DEFINED ENV{DS5_VERSION} AND NOT "$ENV{DS5_VERSION}" STREQUAL "")
    set(DS5_VERSION "$ENV{DS5_VERSION}")
else()
    set(DS5_VERSION "dev")
endif()
set(DS5_BL616_DIAG_LOG 0)
if(DEFINED ENV{DS5_BL616_DIAG_LOG} AND "$ENV{DS5_BL616_DIAG_LOG}" STREQUAL "1")
    set(DS5_BL616_DIAG_LOG 1)
endif()

# ------------------------------------------------------------------ firmware

set(FW_SOURCES
    "${SRC_DIR}/usb_descriptors.cpp"
    "${SRC_DIR}/main.cpp"
    "${SRC_DIR}/usb.cpp"
    "${SRC_DIR}/audio.cpp"
    "${SRC_DIR}/speaker_resample.cpp"
    "${SRC_DIR}/elastic.cpp"
    "${SRC_DIR}/bt.cpp"
    "${SRC_DIR}/button_functions.cpp"
    "${SRC_DIR}/config.cpp"
    "${SRC_DIR}/status_gpio.cpp"
    "${SRC_DIR}/cmd.cpp"
    "${SRC_DIR}/button_remap.cpp"
    "${SRC_DIR}/button_utils.cpp"
    "${SRC_DIR}/battery_led.cpp"
    "${SRC_DIR}/wake.cpp"
    "${SRC_DIR}/button_shortcut.cpp"
    "${SRC_DIR}/wdl_resample.cpp"
    "${SRC_DIR}/port/bl616/port_bl616.cpp"
    "${SRC_DIR}/port/bl616/bt_transport_bl616.cpp"
    "${SRC_DIR}/port/bl616/usb_hid_bl616.cpp"
    "${SRC_DIR}/port/bl616/dcd_bl616.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/entry_bl616.cpp"
)
# ram_mem.c is not built: it moves the Pico's newlib mem ops out of flash; on
# this target PORT_FAST_FUNC places hot code in RAM instead, as on ESP32-S31.

# The shared main() becomes a task body (main_bl616.c owns main()).
set_source_files_properties("${SRC_DIR}/main.cpp" PROPERTIES
    COMPILE_DEFINITIONS "main=ds5_shared_main")
# Per source, because source options are the only ones placed after the SDK's
# own: the SDK compiles C++ as -std=c++11 through its interface library, and
# the shared firmware needs C++20 (std::clamp, designated initialisers,
# constexpr loops). -fsigned-char: WDL's headers static-assert a signed char,
# as on the Pico and ESP32-S31 builds; RISC-V defaults to unsigned.
set(FW_CXX_SOURCES ${FW_SOURCES})
list(FILTER FW_CXX_SOURCES INCLUDE REGEX "\\.cpp$")
set_source_files_properties(${FW_CXX_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-std=gnu++20;-fsigned-char")

# ------------------------------------------------------------------- BTstack

file(GLOB BTSTACK_SOURCES
    "${BTSTACK_ROOT}/src/*.c"
    "${BTSTACK_ROOT}/src/classic/*.c"
    "${BTSTACK_ROOT}/3rd-party/micro-ecc/uECC.c"
    "${BTSTACK_ROOT}/3rd-party/md5/md5.c"
    "${BTSTACK_ROOT}/3rd-party/yxml/yxml.c"
)
list(APPEND BTSTACK_SOURCES
    "${BTSTACK_ROOT}/platform/embedded/btstack_run_loop_embedded.c"
    "${BTSTACK_ROOT}/platform/embedded/hci_dump_embedded_stdout.c"
    "${BTSTACK_ROOT}/platform/embedded/btstack_tlv_flash_bank.c"
)
set(BTSTACK_INCLUDES
    "${BTSTACK_ROOT}/src"
    "${BTSTACK_ROOT}/src/classic"
    "${BTSTACK_ROOT}/platform/embedded"
    "${BTSTACK_ROOT}/3rd-party/micro-ecc"
    "${BTSTACK_ROOT}/3rd-party/md5"
    "${BTSTACK_ROOT}/3rd-party/yxml"
    # Globbed-in LC3/SBC sources are never referenced (Classic HID only); their
    # headers keep them compiling, and --gc-sections drops the code.
    "${BTSTACK_ROOT}/3rd-party/lc3-google/include"
    "${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include"
    "${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include"
)

# ------------------------------------------------------------------- TinyUSB

# The TinyUSB device stack. Its controller driver for the BL616 is
# src/port/bl616/dcd_bl616.c (in FW_SOURCES); TinyUSB has none of its own.
set(TINYUSB_SOURCES
    "${TINYUSB_DIR}/src/tusb.c"
    "${TINYUSB_DIR}/src/common/tusb_fifo.c"
    "${TINYUSB_DIR}/src/device/usbd.c"
    "${TINYUSB_DIR}/src/class/hid/hid_device.c"
    "${TINYUSB_DIR}/src/class/audio/audio_device.c"
    "${TINYUSB_DIR}/src/class/vendor/vendor_device.c"
    "${TINYUSB_DIR}/src/class/cdc/cdc_device.c"
)

# ---------------------------------------------------------------------- Opus

# lib/opus, float, as the Pico and ESP32-S31 builds use -- not the SDK's
# prebuilt archive (CONFIG_OPUS stays off, or the two would collide). Built
# from Opus's own source lists rather than add_subdirectory(): the SDK replaces
# CMake's project() with a macro that sets up a firmware image, so Opus's
# project() call would try to create a second one.
list(APPEND CMAKE_MODULE_PATH "${OPUS_DIR}/cmake")
include(OpusFunctions)
function(ds5_opus_list out mk group)
    get_opus_sources(${group} "${OPUS_DIR}/${mk}" _list)
    set(_abs "")
    foreach(_s ${_list})
        list(APPEND _abs "${OPUS_DIR}/${_s}")
    endforeach()
    set(${out} ${_abs} PARENT_SCOPE)
endfunction()
ds5_opus_list(_opus_src opus_sources.mk OPUS_SOURCES)
ds5_opus_list(_opus_src_float opus_sources.mk OPUS_SOURCES_FLOAT)
ds5_opus_list(_silk_src silk_sources.mk SILK_SOURCES)
ds5_opus_list(_silk_src_float silk_sources.mk SILK_SOURCES_FLOAT)
ds5_opus_list(_celt_src celt_sources.mk CELT_SOURCES)
add_library(ds5_opus STATIC ${_opus_src} ${_opus_src_float} ${_silk_src} ${_silk_src_float} ${_celt_src})
# sdk_intf_lib carries the SDK's -march/-mabi and the rest of its flags.
target_link_libraries(ds5_opus PRIVATE sdk_intf_lib)
target_include_directories(ds5_opus PUBLIC "${OPUS_DIR}/include")
target_include_directories(ds5_opus PRIVATE "${OPUS_DIR}" "${OPUS_DIR}/celt" "${OPUS_DIR}/silk"
                           "${OPUS_DIR}/silk/float" "${OPUS_DIR}/dnn")
# What Opus's CMake would decide for this toolchain: a C99 compiler (VLAs) and
# IEEE-754 floats, so its single-precision approximations are valid.
target_compile_definitions(ds5_opus PRIVATE OPUS_BUILD VAR_ARRAYS FLOAT_APPROX HAVE_LRINT HAVE_LRINTF)
target_compile_options(ds5_opus PRIVATE -O2 -Wno-undef)
# The E907's FPU is single precision only, like the ESP32-S31's, so the same
# CELT single-precision header applies (see ports/esp32s31/components/libopus).
set(_opus_single_srcs ${_celt_src})
foreach(_src ${_opus_src})
    if(_src MATCHES "/src/opus(_encoder|_decoder)?\\.c$")
        list(APPEND _opus_single_srcs "${_src}")
    endif()
endforeach()
set_source_files_properties(${_opus_single_srcs} PROPERTIES
    COMPILE_OPTIONS "-include;${REPO_DIR}/ports/esp32s31/components/libopus/opus_single_precision.h;-fsingle-precision-constant")
# ----------------------------------------------------------------------- app

target_sources(app PRIVATE ${FW_SOURCES} ${BTSTACK_SOURCES} ${TINYUSB_SOURCES})
target_include_directories(app PRIVATE
    "${SRC_DIR}"
    "${SRC_DIR}/port/bl616"
    "${CMAKE_CURRENT_SOURCE_DIR}"
    ${BTSTACK_INCLUDES}
    "${TINYUSB_DIR}/src"
)
# WDL after every other include path: its queue.h would otherwise shadow
# FreeRTOS's (the shared code includes WDL's resample.h by its bare name).
target_compile_options(app PRIVATE -idirafter "${WDL_DIR}")
target_link_libraries(app ds5_opus)

target_compile_definitions(app PRIVATE
    ENABLE_CLASSIC=1
    # CFG_TUSB_MCU / CFG_TUSB_OS must agree across every TinyUSB translation
    # unit. OPT_OS_NONE as on ESP32-S31: tud_task() is called from the
    # superloop and must not block. 5 endpoints: EP0 plus the BL616's EP1-4.
    CFG_TUSB_MCU=OPT_MCU_NONE
    CFG_TUSB_OS=OPT_OS_NONE
    TUP_DCD_ENDPOINT_MAX=5
    BOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
    ENABLE_BATT_LED=1
    # Always 0 here: there is no endpoint left for a CDC port (the debug log
    # rides a HID feature report instead, DS5_BL616_DIAG_LOG).
    ENABLE_SERIAL=0
    ENABLE_DEBUG=0
    ENABLE_VERBOSE=0
    # The wake keyboard and consumer control, as on the other targets; here
    # they are collections inside the DualSense HID interface
    # (src/port/bl616/usb_hid_bl616.cpp).
    ENABLE_WAKE_HID
    DISABLE_SPEAKER_PROC=0
    WDL_RESAMPLE_TYPE=float
    DS5_FIRMWARE_VERSION="${DS5_VERSION}"
    DS5_BL616_DIAG_LOG=${DS5_BL616_DIAG_LOG}
)
# TinyUSB's endpoint buffers in non-cacheable RAM (the controller's DMA reads
# and writes them); force-included so every TinyUSB unit agrees.
# SHELL: keeps the pair together -- CMake de-duplicates a bare "-include" against
# the SDK's own "-include autoconf.h" and drops one.
target_compile_options(app PRIVATE "SHELL:-include ${CMAKE_CURRENT_SOURCE_DIR}/bl616_tusb_mem.h")
# The four-endpoint layout (usb_hid_bl616.cpp): these TinyUSB calls are
# wrapped where the shared code and TinyUSB call each other.
sdk_add_link_options(-Wl,--wrap=tud_descriptor_configuration_cb
                     -Wl,--wrap=tud_hid_descriptor_report_cb
                     -Wl,--wrap=tud_hid_n_ready
                     -Wl,--wrap=tud_hid_n_report
                     -Wl,--wrap=tud_hid_n_keyboard_report
                     -Wl,--wrap=tud_hid_get_report_cb)

target_compile_options(app PRIVATE
    -Wno-undef
    -Wno-missing-field-initializers
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
)
# BTstack and TinyUSB are vendored code built with the project's flags;
# downgrade their warnings rather than patch them.
set_source_files_properties(${BTSTACK_SOURCES} ${TINYUSB_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wno-maybe-uninitialized;-Wno-unused-variable;-Wno-unused-function")

if(DS5_BL616_DIAG_LOG)
    target_sources(app PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/diag_log.c"
                               "${CMAKE_CURRENT_SOURCE_DIR}/diag_probes.c")
    # Every console byte is copied into the log ring by wrapping the SDK's
    # UART output calls (diag_log.c), so nothing else needs to change.
    sdk_add_link_options(-Wl,--wrap=bflb_uart_putchar
                         -Wl,--wrap=bflb_uart_put
                         -Wl,--wrap=bflb_uart_put_block)
    # CPU exceptions are written into the persistent log, then reset
    # (diag_log.c), instead of the SDK's console core dump.
    sdk_add_link_options(-Wl,--wrap=exception_entry)
    # Main-loop position markers (diag_probes.c): every call the shared main
    # loop makes, wrapped where main.cpp links to it. The C++ names are the
    # ones main.cpp.obj references.
    sdk_add_link_options(-Wl,--wrap=_ZN4port15watchdog_updateEv
                         -Wl,--wrap=_ZN4port17bt_transport_pollEv
                         -Wl,--wrap=_Z7bt_taskv
                         -Wl,--wrap=_Z18usb_reconnect_taskv
                         -Wl,--wrap=tud_task_ext
                         -Wl,--wrap=_Z9wake_taskv
                         -Wl,--wrap=_Z20button_shortcut_taskv
                         -Wl,--wrap=_Z10audio_loopv
                         -Wl,--wrap=_Z16battery_led_tickv
                         -Wl,--wrap=_Z12button_checkv
                         -Wl,--wrap=_Z16bt_inquiring_ledv)
    # Variant P: every caller that takes the USB device off the bus, logged
    # with its return address (diag_probes.c).
    sdk_add_link_options(-Wl,--wrap=tud_disconnect
                         -Wl,--wrap=tud_connect)
endif()

sdk_set_main_file(main_bl616.c)

project(ds5_bridge_haute)
