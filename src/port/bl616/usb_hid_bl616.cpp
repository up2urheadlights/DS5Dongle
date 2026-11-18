// BL616: fit the PR's USB device into the controller's four endpoints.
//
// The BL616 has EP1-4 only, one direction per number. The DualSense function
// uses all four (0x01 speaker, 0x82 mic, 0x03 HID OUT, 0x84 HID IN), so the
// wake keyboard and the consumer/system-control interface -- their own HID
// interfaces on EP 0x87/0x88 on the other targets -- cannot be separate
// interfaces here. Instead they become extra top-level collections inside the
// DualSense HID interface, with report IDs no DualSense report uses. (The
// adapter's original firmware does the same: in its DualSense mode Windows
// shows the HID interface with a game controller, a keyboard and a mouse
// collection.)
//
// Nothing in the shared code changes for this. The build wraps, at link time
// (-Wl,--wrap, ports/bl616_haute/CMakeLists.txt), the TinyUSB calls that
// carry the layout:
//   tud_descriptor_configuration_cb  drop the keyboard/consumer interfaces and
//                                    give the HID interface the combined
//                                    report descriptor's length; in keyboard-
//                                    only mode move the keyboard to EP 0x81
//   tud_hid_descriptor_report_cb     DualSense descriptor + the collections
//   tud_hid_n_ready / tud_hid_n_report / tud_hid_n_keyboard_report
//                                    keyboard (instance 1) and consumer
//                                    (instance 2) reports go out on instance 0
//                                    with their report IDs
//   tud_hid_get_report_cb            answer GET_REPORT for those IDs (and, in
//                                    the debug build, the log report)
// Only references from other objects are wrapped, so TinyUSB's own internal
// calls are untouched; the shared callbacks still run behind the wrappers.

#include <cstdint>
#include <cstring>

#include "tusb.h"
#include "usb.h"
#include "usb_descriptors.h"

#if DS5_BL616_DIAG_LOG
#include "diag_log.h"
#endif

namespace {

constexpr uint8_t kHidItf = 3;           // the DualSense HID interface number
constexpr uint8_t kNumBaseItf = 4;       // audio control, speaker, mic, HID
constexpr uint8_t kKbdEpShared = 0x87;   // the other targets' keyboard endpoint
constexpr uint8_t kKbdEpBl616 = 0x81;    // keyboard-only mode here: EP1 IN

constexpr uint8_t kKeyboardReportId = 0x40;
constexpr uint8_t kConsumerReportId = 0x41; // shared code's consumer id 1
constexpr uint8_t kSystemReportId = 0x42;   // shared code's system id 2

constexpr uint8_t kKeyboardInstance = 1; // button_shortcut.cpp / wake.cpp
constexpr uint8_t kConsumerInstance = 2;

// The keyboard of desc_hid_report_kbd (usb_descriptors.cpp) with a report ID.
const uint8_t kKeyboardCollection[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, kKeyboardReportId,
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       //   Usage Minimum (Left Control)
    0x29, 0xE7,       //   Usage Maximum (Right GUI)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data,Var,Abs) -- modifier byte
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x08,       //   Report Size (8)
    0x81, 0x01,       //   Input (Const) -- reserved byte
    0x95, 0x06,       //   Report Count (6)
    0x75, 0x08,       //   Report Size (8)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, SHORTCUT_KEY_USAGE_MAX,
    0x05, 0x07,       //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, SHORTCUT_KEY_USAGE_MAX,
    0x81, 0x00,       //   Input (Data,Array) -- 6 keycodes
    0xC0,             // End Collection
};

// desc_hid_report_consumer (usb_descriptors.cpp) with this target's IDs.
const uint8_t kConsumerCollections[] = {
    0x05, 0x0C,       // Usage Page (Consumer)
    0x09, 0x01,       // Usage (Consumer Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, kConsumerReportId,
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, SHORTCUT_CONSUMER_USAGE_MAX & 0xFF, SHORTCUT_CONSUMER_USAGE_MAX >> 8,
    0x19, 0x00,       //   Usage Minimum (0)
    0x2A, SHORTCUT_CONSUMER_USAGE_MAX & 0xFF, SHORTCUT_CONSUMER_USAGE_MAX >> 8,
    0x75, 0x10,       //   Report Size (16)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x00,       //   Input (Data,Array,Abs)
    0xC0,             // End Collection

    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x80,       // Usage (System Control)
    0xA1, 0x01,       // Collection (Application)
    0x85, kSystemReportId,
    0x15, 0x01,       //   Logical Minimum (1)
    0x25, 0x03,       //   Logical Maximum (3)
    0x75, 0x02,       //   Report Size (2 bits)
    0x95, 0x01,       //   Report Count (1)
    0x09, 0x81,       //   Usage (System Power Down)
    0x09, 0x82,       //   Usage (System Sleep)
    0x09, 0x83,       //   Usage (System Wake Up)
    0x81, 0x00,       //   Input (Data,Array,Abs)
    0x75, 0x06,       //   Report Size (6 bits padding)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x01,       //   Input (Const)
    0xC0,             // End Collection
};

#if DS5_BL616_DIAG_LOG
// Debug build only: a vendor collection whose feature report carries the log.
const uint8_t kLogCollection[] = {
    0x06, 0x00, 0xFF, // Usage Page (Vendor 0xFF00)
    0x09, 0x01,       // Usage (Vendor 1)
    0xA1, 0x01,       // Collection (Application)
    0x85, DIAG_LOG_REPORT_ID,
    0x09, 0x02,       //   Usage (Vendor 2)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, DIAG_LOG_REPORT_SIZE, //   Report Count
    0xB1, 0x02,       //   Feature (Data,Var,Abs)
    0xC0,             // End Collection
};
#endif

constexpr size_t kMaxDsDescriptor = 453; // desc_hid_report_dse
uint8_t g_combined[kMaxDsDescriptor + sizeof(kKeyboardCollection) + sizeof(kConsumerCollections)
#if DS5_BL616_DIAG_LOG
                   + sizeof(kLogCollection)
#endif
];
uint16_t g_ds_len = 0;          // the shared DualSense/Edge descriptor's length
bool g_shortcut_collections = false; // the shared config asked for keyboard + consumer

uint8_t g_config[512];

bool combined_layout() { return !usb_keyboard_only; }

uint16_t extra_len() {
    uint16_t n = 0;
    if (g_shortcut_collections) n += sizeof(kKeyboardCollection) + sizeof(kConsumerCollections);
#if DS5_BL616_DIAG_LOG
    n += sizeof(kLogCollection);
#endif
    return n;
}

} // namespace

extern "C" {

uint8_t const *__real_tud_descriptor_configuration_cb(uint8_t index);
uint8_t const *__real_tud_hid_descriptor_report_cb(uint8_t instance);
bool __real_tud_hid_n_ready(uint8_t instance);
bool __real_tud_hid_n_report(uint8_t instance, uint8_t report_id, void const *report, uint16_t len);
bool __real_tud_hid_n_keyboard_report(uint8_t instance, uint8_t report_id, uint8_t modifier, const uint8_t keycode[6]);
uint16_t __real_tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                      uint8_t *buffer, uint16_t reqlen);

uint8_t const *__wrap_tud_descriptor_configuration_cb(uint8_t index) {
    uint8_t const *src = __real_tud_descriptor_configuration_cb(index);
    if (src == nullptr) return nullptr;
    const uint16_t total = static_cast<uint16_t>(src[2] | (src[3] << 8));
    if (total > sizeof(g_config)) return src; // cannot happen with the PR's descriptors

    if (!combined_layout()) {
        // Keyboard-only mode: the same single keyboard interface, on EP1.
        memcpy(g_config, src, total);
        for (uint16_t i = 0; i + 1 < total && g_config[i] != 0; i += g_config[i]) {
            if (g_config[i + 1] == TUSB_DESC_ENDPOINT && g_config[i + 2] == kKbdEpShared) {
                g_config[i + 2] = kKbdEpBl616;
            }
        }
        return g_config;
    }

    // Full device: keep the configuration header and interfaces 0-3, drop the
    // keyboard and consumer interfaces the shared descriptor appends.
    g_shortcut_collections = src[4] > kNumBaseItf;
    uint16_t out = 0;
    int itf = -1;
    for (uint16_t i = 0; i + 1 < total && src[i] != 0; i += src[i]) {
        const uint8_t len = src[i];
        const uint8_t type = src[i + 1];
        if (type == TUSB_DESC_INTERFACE) itf = src[i + 2];
        if (type == TUSB_DESC_INTERFACE_ASSOCIATION) itf = src[i + 2];
        if (type == TUSB_DESC_CONFIGURATION || itf < kNumBaseItf) {
            memcpy(g_config + out, src + i, len);
            if (type == HID_DESC_TYPE_HID && itf == kHidItf) {
                // The HID descriptor's report-descriptor length (offset 7).
                g_ds_len = static_cast<uint16_t>(src[i + 7] | (src[i + 8] << 8));
                const uint16_t n = static_cast<uint16_t>(g_ds_len + extra_len());
                g_config[out + 7] = static_cast<uint8_t>(n & 0xFF);
                g_config[out + 8] = static_cast<uint8_t>(n >> 8);
            }
            out = static_cast<uint16_t>(out + len);
        }
    }
    g_config[2] = static_cast<uint8_t>(out & 0xFF);
    g_config[3] = static_cast<uint8_t>(out >> 8);
    g_config[4] = kNumBaseItf;
    return g_config;
}

uint8_t const *__wrap_tud_hid_descriptor_report_cb(uint8_t instance) {
    uint8_t const *src = __real_tud_hid_descriptor_report_cb(instance);
    if (!combined_layout() || instance != 0 || src == nullptr || g_ds_len == 0 ||
        g_ds_len > kMaxDsDescriptor) {
        return src;
    }
    uint16_t n = g_ds_len;
    memcpy(g_combined, src, n);
    if (g_shortcut_collections) {
        memcpy(g_combined + n, kKeyboardCollection, sizeof(kKeyboardCollection));
        n += sizeof(kKeyboardCollection);
        memcpy(g_combined + n, kConsumerCollections, sizeof(kConsumerCollections));
        n += sizeof(kConsumerCollections);
    }
#if DS5_BL616_DIAG_LOG
    memcpy(g_combined + n, kLogCollection, sizeof(kLogCollection));
#endif
    return g_combined;
}

bool __wrap_tud_hid_n_ready(uint8_t instance) {
    if (combined_layout() && (instance == kKeyboardInstance || instance == kConsumerInstance)) {
        return g_shortcut_collections && __real_tud_hid_n_ready(0);
    }
    return __real_tud_hid_n_ready(instance);
}

bool __wrap_tud_hid_n_report(uint8_t instance, uint8_t report_id, void const *report, uint16_t len) {
    if (combined_layout() && instance == kKeyboardInstance) {
        if (!g_shortcut_collections) return false;
        return __real_tud_hid_n_report(0, kKeyboardReportId, report, len);
    }
    if (combined_layout() && instance == kConsumerInstance) {
        if (!g_shortcut_collections) return false;
        const uint8_t id = report_id == 2 ? kSystemReportId : kConsumerReportId;
        return __real_tud_hid_n_report(0, id, report, len);
    }
    return __real_tud_hid_n_report(instance, report_id, report, len);
}

bool __wrap_tud_hid_n_keyboard_report(uint8_t instance, uint8_t report_id, uint8_t modifier,
                                      const uint8_t keycode[6]) {
    if (combined_layout() && instance == kKeyboardInstance) {
        if (!g_shortcut_collections) return false;
        uint8_t report[8] = {modifier, 0};
        if (keycode != nullptr) memcpy(report + 2, keycode, 6);
        return __real_tud_hid_n_report(0, kKeyboardReportId, report, sizeof(report));
    }
    return __real_tud_hid_n_keyboard_report(instance, report_id, modifier, keycode);
}

uint16_t __wrap_tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                      uint8_t *buffer, uint16_t reqlen) {
    if (combined_layout() && instance == 0) {
#if DS5_BL616_DIAG_LOG
        if (report_id == DIAG_LOG_REPORT_ID && report_type == HID_REPORT_TYPE_FEATURE) {
            return diag_log_fill_report(buffer, reqlen);
        }
#endif
        // All-up input reports, as the shared keyboard GET_REPORT answers.
        uint16_t n = 0;
        if (report_id == kKeyboardReportId) n = 8;
        if (report_id == kConsumerReportId) n = 2;
        if (report_id == kSystemReportId) n = 1;
        if (n != 0) {
            if (reqlen < n) return 0;
            memset(buffer, 0, n);
            return n;
        }
    }
    return __real_tud_hid_get_report_cb(instance, report_id, report_type, buffer, reqlen);
}

} // extern "C"
