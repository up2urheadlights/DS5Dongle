//
// XInput (Xbox 360 compatible) function.
//
// Added alongside the DualSense interfaces rather than replacing them: Windows
// binds xusb22.sys via the MS OS 2.0 compatible ID "XUSB10" (see
// usb_descriptors.cpp), which is matched per function and needs no Microsoft
// VID/PID, so the device keeps its own identity everywhere else.
//
// Reports the D-pad and A/B only -- enough to navigate the Xbox Game Bar, and
// as close to inert as possible for anything else that enumerates controllers.
//

#include "xinput.h"

#include <cstring>

#include "tusb.h"
#include "device/usbd_pvt.h"
#include "config.h"

#define XINPUT_EP_IN    0x81
#define XINPUT_EP_OUT   0x02
#define XINPUT_IN_SIZE  20
#define XINPUT_OUT_SIZE 32

static uint8_t xinput_rhport = 0;
static bool xinput_opened    = false;
static uint8_t in_buf[XINPUT_IN_SIZE];
static uint8_t out_buf[XINPUT_OUT_SIZE];
static uint8_t last_sent[XINPUT_IN_SIZE];

//--------------------------------------------------------------------+
// TinyUSB application class driver
//--------------------------------------------------------------------+

static void xinput_drv_init(void) {
    xinput_opened = false;
    memset(last_sent, 0, sizeof(last_sent));
}

static bool xinput_drv_deinit(void) { return true; }

static void xinput_drv_reset(uint8_t rhport) {
    (void) rhport;
    xinput_opened = false;
    memset(last_sent, 0, sizeof(last_sent));
}

static uint16_t xinput_drv_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
    // Vendor class 0xFF / subclass 0x5D / protocol 0x01 is the Xbox 360 pattern.
    if (itf_desc->bInterfaceClass != 0xFF) return 0;
    if (itf_desc->bInterfaceSubClass != 0x5D) return 0;
    if (itf_desc->bInterfaceProtocol != 0x01) return 0;

    // The Xbox vendor descriptor sits between the interface and the endpoints,
    // so step by descriptor length rather than assuming a layout.
    uint8_t const *p = (uint8_t const *) itf_desc;
    uint16_t consumed = tu_desc_len(p);
    uint8_t eps = 0;

    while (eps < itf_desc->bNumEndpoints) {
        if (consumed >= max_len) return 0;
        p = tu_desc_next(p);
        const uint16_t len = tu_desc_len(p);
        if (len == 0 || (uint16_t) (consumed + len) > max_len) return 0;
        consumed = (uint16_t) (consumed + len);
        if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
            if (!usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) p)) return 0;
            eps++;
        }
    }

    xinput_rhport = rhport;
    xinput_opened = true;
    memset(last_sent, 0, sizeof(last_sent));

    // Windows sends rumble/LED-ring packets on OUT. Discarded -- the DualSense
    // owns its haptics and lightbar -- but they must be consumed or the host
    // keeps retrying. Trailing false is is_isr; this runs on the usbd task.
    usbd_edpt_xfer(rhport, XINPUT_EP_OUT, out_buf, sizeof(out_buf), false);
    return consumed;
}

static bool xinput_drv_control_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request) {
    (void) rhport;
    (void) stage;
    (void) request;
    return false; // nothing class-specific to answer; let it stall
}

static bool xinput_drv_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    (void) result;
    (void) xferred_bytes;
    if (ep_addr == XINPUT_EP_OUT) {
        usbd_edpt_xfer(rhport, XINPUT_EP_OUT, out_buf, sizeof(out_buf), false); // discard and re-arm
    }
    return true;
}

static const usbd_class_driver_t xinput_driver = {
    .name            = "XINPUT",
    .init            = xinput_drv_init,
    .deinit          = xinput_drv_deinit,
    .reset           = xinput_drv_reset,
    .open            = xinput_drv_open,
    .control_xfer_cb = xinput_drv_control_cb,
    .xfer_cb         = xinput_drv_xfer_cb,
    .xfer_isr        = nullptr,
    .sof             = nullptr,
};

extern "C" usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &xinput_driver;
}

//--------------------------------------------------------------------+
// Report path
//--------------------------------------------------------------------+

// DualSense hat: 0..7 clockwise from north, 8 (or anything else) centred.
// XInput byte 2: bit0 up, bit1 down, bit2 left, bit3 right.
static const uint8_t hat_to_dpad[8] = {
    0x01,        // N
    0x01 | 0x08, // NE
    0x08,        // E
    0x02 | 0x08, // SE
    0x02,        // S
    0x02 | 0x04, // SW
    0x04,        // W
    0x01 | 0x04, // NW
};

static void xinput_send(const uint8_t *pkt) {
    if (memcmp(pkt, last_sent, XINPUT_IN_SIZE) == 0) return;
    if (!tud_ready()) return;
    if (usbd_edpt_busy(xinput_rhport, XINPUT_EP_IN)) return;
    if (!usbd_edpt_claim(xinput_rhport, XINPUT_EP_IN)) return;

    memcpy(in_buf, pkt, XINPUT_IN_SIZE);
    if (usbd_edpt_xfer(xinput_rhport, XINPUT_EP_IN, in_buf, XINPUT_IN_SIZE, false)) {
        memcpy(last_sent, pkt, XINPUT_IN_SIZE);
    } else {
        usbd_edpt_release(xinput_rhport, XINPUT_EP_IN);
    }
}

void xinput_tick(const uint8_t *body, uint16_t len) {
    if (!xinput_opened) return;
    if (!get_config().ps_shortcut_enabled) return;
    if (len < 10) return;

    // body[7]: hat in the low nibble, face buttons in the high nibble.
    const uint8_t hat = body[7] & 0x0F;
    const bool cross  = (body[7] & 0x20) != 0; // Cross  -> A
    const bool circle = (body[7] & 0x40) != 0; // Circle -> B

    uint8_t pkt[XINPUT_IN_SIZE];
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 0x00;             // message type: input report
    pkt[1] = XINPUT_IN_SIZE;   // packet length
    if (hat < 8) pkt[2] = hat_to_dpad[hat];
    if (cross) pkt[3] |= 0x10;
    if (circle) pkt[3] |= 0x20;

    xinput_send(pkt);
}

void xinput_reset_state() {
    uint8_t pkt[XINPUT_IN_SIZE];
    memset(pkt, 0, sizeof(pkt));
    pkt[1] = XINPUT_IN_SIZE;
    if (xinput_opened) xinput_send(pkt);
    memset(last_sent, 0, sizeof(last_sent));
}
