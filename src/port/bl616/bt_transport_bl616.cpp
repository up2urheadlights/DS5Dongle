// BTstack HCI transport over the BL616 on-chip Bluetooth controller.
//
// The controller is Bouffalo's closed btblecontroller library. It is not reached
// over H4: its boundary (hci_onchip.h) takes commands and ACL as parsed structs
// and hands events back pre-split into command-complete, command-status, other
// events, LE meta events and ACL, without the H4 headers. Bouffalo's own host
// rebuilds the headers in bl_hci_wrapper.c; this file does the same for BTstack:
//
//   to the controller   H4 command  -> bt_onchiphci_send(BT_HCI_CMD, {opcode, params})
//                       H4 ACL      -> bt_onchiphci_send(BT_HCI_ACL_DATA, {handle, pb/bc, data})
//   from the controller CMD_CMP_EVT -> 0x0E Command Complete  (opcode in src_id)
//                       CMD_STAT_EVT-> 0x0F Command Status    (opcode in src_id, status in param[0])
//                       EVT         -> event code src_id, params as given
//                       LE_EVT      -> 0x3E LE Meta
//                       ACL_DATA    -> bt_onchiphci_handle_rx_acl() writes header + payload
//
// The receive callback runs on the controller's own FreeRTOS task. As on the
// ESP32-S31 port, it only copies into a ring; the main task drains the ring and
// calls BTstack, so every BTstack callback runs on the main task, which the
// shared firmware requires (see bt_transport_esp32s31.cpp). BTstack runs on its
// pollable embedded run loop, pumped from the superloop.
//
// Link keys live in BTstack's own two-bank TLV store in a flash region nothing
// else uses (board_haute.h). Bonds written by any other firmware on this board
// are never read.

#include "../port.h"
#include "board_haute.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include "bflb_core.h"
#include "bflb_flash.h"
#include "bflb_irq.h"
#include "bflb_mtimer.h"
#include "btble_lib_api.h"
#include "hci_onchip.h"

#include "btstack_memory.h"
#include "btstack_ring_buffer.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_embedded.h"
#include "btstack_tlv.h"
#include "btstack_tlv_flash_bank.h"
#include "btstack_event.h"
#include "btstack_util.h"
#include "classic/btstack_link_key_db_tlv.h"
#include "hal_flash_bank.h"
#include "hci.h"
#include "hci_transport.h"
#include "FreeRTOS.h"
#include "task.h"
}

// Hook from main_bl616.c: start the debug CDC log once Bluetooth is up (USB is
// started only after Bluetooth init on this chip). A no-op in non-debug builds.
extern "C" void ds5_bl616_bt_ready(void);

namespace {

// ------------------------------------------------------------- receive ring

// Each entry is a 2-byte length tag plus the H4 packet (type byte + payload).
// Sized for a burst of full ACL packets plus events: the main loop can be away
// for a few milliseconds (flash writes, USB) while the pad streams 1000/s.
uint8_t g_ring_storage[12 * 1024];
btstack_ring_buffer_t g_ring;

// Scratch the controller writes an incoming ACL packet into, and the frame
// built for an event. Only touched from the controller task.
uint8_t g_rx_scratch[1 + 4 + 2048];

volatile uint32_t g_rx_dropped = 0;
volatile uint32_t g_rx_dropped_events = 0;
volatile uint32_t g_rx_acl_bad_len = 0;
uint32_t g_rx_acl_count = 0; // main task only
constexpr uint32_t kDroppedEventCodes = 8;
volatile uint8_t g_rx_dropped_event_code[kDroppedEventCodes] = {};

uint8_t g_rx_with_pre_buffer[HCI_INCOMING_PRE_BUFFER_SIZE + HCI_INCOMING_PACKET_BUFFER_SIZE];
uint8_t *const g_rx = &g_rx_with_pre_buffer[HCI_INCOMING_PRE_BUFFER_SIZE];

void (*g_packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size) = nullptr;
volatile bool g_tx_complete = false;
bool g_controller_started = false;

void ring_push(const uint8_t *h4, uint32_t len) {
    const uintptr_t flags = bflb_irq_save();
    if (btstack_ring_buffer_bytes_free(&g_ring) < len + 2) {
        bflb_irq_restore(flags);
        // Counted, not printed: this is the controller's task, and blocking it
        // on the console is how links get dropped. Reported from the poll.
        ++g_rx_dropped;
        if (len >= 2 && h4[0] == HCI_EVENT_PACKET) {
            g_rx_dropped_event_code[g_rx_dropped_events % kDroppedEventCodes] = h4[1];
            ++g_rx_dropped_events;
        }
        return;
    }
    uint8_t len_tag[2];
    little_endian_store_16(len_tag, 0, static_cast<uint16_t>(len));
    btstack_ring_buffer_write(&g_ring, len_tag, sizeof(len_tag));
    btstack_ring_buffer_write(&g_ring, const_cast<uint8_t *>(h4), len);
    bflb_irq_restore(flags);
}

// Controller -> host. Runs on the controller task.
void onchip_rx_cb(uint8_t pkt_type, uint16_t src_id, uint8_t *param, uint8_t param_len) {
    uint8_t *f = g_rx_scratch;
    uint32_t len = 0;
    switch (pkt_type) {
        case BT_HCI_CMD_CMP_EVT:
            // Command Complete: num_hci_command_packets, opcode, return params.
            // The controller takes one command at a time, as Bouffalo's host
            // assumes (nb_h2c_cmd_pkts = 1 in bl_hci_wrapper.c).
            f[0] = HCI_EVENT_PACKET;
            f[1] = HCI_EVENT_COMMAND_COMPLETE;
            f[2] = static_cast<uint8_t>(3 + param_len);
            f[3] = 1;
            little_endian_store_16(f, 4, src_id);
            memcpy(&f[6], param, param_len);
            len = 6u + param_len;
            break;
        case BT_HCI_CMD_STAT_EVT:
            // Command Status: status, num_hci_command_packets, opcode.
            f[0] = HCI_EVENT_PACKET;
            f[1] = HCI_EVENT_COMMAND_STATUS;
            f[2] = 4;
            f[3] = param_len ? param[0] : 0;
            f[4] = 1;
            little_endian_store_16(f, 5, src_id);
            len = 7;
            break;
        case BT_HCI_EVT:
            f[0] = HCI_EVENT_PACKET;
            f[1] = static_cast<uint8_t>(src_id);
            f[2] = param_len;
            memcpy(&f[3], param, param_len);
            len = 3u + param_len;
            break;
        case BT_HCI_LE_EVT:
            f[0] = HCI_EVENT_PACKET;
            f[1] = HCI_EVENT_LE_META;
            f[2] = param_len;
            memcpy(&f[3], param, param_len);
            len = 3u + param_len;
            break;
        case BT_HCI_ACL_DATA: {
            // Must be called exactly once per ACL packet, ring full or not: it
            // is what hands the controller's receive buffer back.
            f[0] = HCI_ACL_DATA_PACKET;
            const uint16_t n = bt_onchiphci_handle_rx_acl(param, &f[1]);
            // n is the ACL header (handle, length) plus payload. Check it
            // rather than trust it: a wrong reading of the boundary would
            // otherwise reach L2CAP as garbage.
            if (n < 4 || n > sizeof(g_rx_scratch) - 1 ||
                4u + little_endian_read_16(f, 3) != n || n > HCI_INCOMING_PACKET_BUFFER_SIZE) {
                ++g_rx_acl_bad_len;
                return;
            }
            len = 1u + n;
            break;
        }
        default:
            // BT_HCI_DBG_EVT and anything newer: nothing BTstack consumes.
            return;
    }
    ring_push(f, len);
}

// ------------------------------------------------------------- hci_transport

void transport_init(const void *) {}

int transport_open() {
    btstack_ring_buffer_init(&g_ring, g_ring_storage, sizeof(g_ring_storage));
    if (!g_controller_started) {
        // Same priority Bouffalo's own bring-up gives it (configMAX_PRIORITIES
        // - 1), so the controller preempts everything else on the one core.
        btble_controller_init(configMAX_PRIORITIES - 1);
        const uint8_t err = bt_onchiphci_interface_init(onchip_rx_cb);
        if (err != 0) {
            printf("[BT] bt_onchiphci_interface_init failed: %u\n", err);
            return -1;
        }
        g_controller_started = true;
        printf("[BT] BL616 controller %s started\n", btble_controller_get_lib_ver());
    }
    return 0;
}

int transport_close() { return 0; }

void transport_register_packet_handler(void (*handler)(uint8_t, uint8_t *, uint16_t)) {
    g_packet_handler = handler;
}

int transport_can_send_packet_now(uint8_t) { return 1; }

int transport_send_packet(uint8_t packet_type, uint8_t *packet, int size) {
    hci_pkt_struct pkt;
    memset(&pkt, 0, sizeof(pkt));
    int8_t rc;
    if (packet_type == HCI_COMMAND_DATA_PACKET) {
        if (size < 3) return -1;
        pkt.p.hci_cmd.opcode = little_endian_read_16(packet, 0);
        pkt.p.hci_cmd.param_len = packet[2];
        pkt.p.hci_cmd.params = &packet[3];
        rc = bt_onchiphci_send(BT_HCI_CMD, 0, &pkt);
    } else if (packet_type == HCI_ACL_DATA_PACKET) {
        if (size < 4) return -1;
        const uint16_t handle_flags = little_endian_read_16(packet, 0);
        const uint16_t handle = handle_flags & 0x0FFF;
        pkt.p.acl_data.conhdl = handle;
        pkt.p.acl_data.pb_bc_flag = static_cast<uint8_t>((handle_flags >> 12) & 0x0F);
        pkt.p.acl_data.len = little_endian_read_16(packet, 2);
        pkt.p.acl_data.buffer = &packet[4];
        rc = bt_onchiphci_send(BT_HCI_ACL_DATA, handle, &pkt);
    } else {
        printf("[BT] unsupported outgoing HCI packet type %u\n", packet_type);
        return -1;
    }
    if (rc != 0) {
        printf("[BT] bt_onchiphci_send(type %u) failed: %d\n", packet_type, rc);
    }
    // bt_onchiphci_send() has consumed the packet before it returns -- Bouffalo's
    // own host frees its buffer immediately after the call -- so BTstack's
    // buffer can be released now.
    g_tx_complete = true;
    return rc == 0 ? 0 : -1;
}

const hci_transport_t g_transport = {
    "bl616-onchip",
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_packet,
    nullptr, // set_baudrate
    nullptr, // reset_link
    nullptr, // set_sco_config
};

// ---------------------------------------------------------------- bond store

// hal_flash_bank over the SDK flash driver: two banks in the region of
// board_haute.h. BTstack writes each offset once between erases; NOR flash
// allows that at any alignment, and 4 keeps the entries word-aligned.
uint32_t bank_addr(int bank) {
    return bank == 0 ? BOARD_FLASH_BTSTACK_BANK0 : BOARD_FLASH_BTSTACK_BANK1;
}

uint32_t flash_bank_get_size(void *) { return BOARD_FLASH_BTSTACK_BANK_SIZE; }
uint32_t flash_bank_get_alignment(void *) { return 4; }

void flash_bank_erase(void *, int bank) {
    if (bflb_flash_erase(bank_addr(bank), BOARD_FLASH_BTSTACK_BANK_SIZE) != 0) {
        printf("[BT] bond store: erase of bank %d failed\n", bank);
    }
}

void flash_bank_read(void *, int bank, uint32_t offset, uint8_t *buffer, uint32_t size) {
    if (offset + size > BOARD_FLASH_BTSTACK_BANK_SIZE) return;
    if (bflb_flash_read(bank_addr(bank) + offset, buffer, size) != 0) {
        // Present erased flash: the TLV then finds no valid header and
        // starts clean instead of parsing stale bytes.
        memset(buffer, 0xff, size);
    }
}

void flash_bank_write(void *, int bank, uint32_t offset, const uint8_t *data, uint32_t size) {
    if (offset + size > BOARD_FLASH_BTSTACK_BANK_SIZE) return;
    if (bflb_flash_write(bank_addr(bank) + offset, const_cast<uint8_t *>(data), size) != 0) {
        printf("[BT] bond store: write at bank %d +0x%lx failed\n", bank, (unsigned long) offset);
    }
}

const hal_flash_bank_t g_flash_bank = {
    &flash_bank_get_size,
    &flash_bank_get_alignment,
    &flash_bank_erase,
    &flash_bank_read,
    &flash_bank_write,
};

btstack_tlv_flash_bank_t g_tlv_context;

// Tells main_bl616.c that the stack is up, once, so the debug log port can
// start in the order this chip needs (USB after Bluetooth).
btstack_packet_callback_registration_t g_state_cb;

void state_handler(uint8_t packet_type, uint16_t, uint8_t *packet, uint16_t) {
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != BTSTACK_EVENT_STATE) return;
    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
    static bool reported = false;
    if (reported) return;
    reported = true;
    bd_addr_t addr;
    gap_local_bd_addr(addr);
    printf("[BT] stack working, local address %s\n", bd_addr_to_str(addr));
    ds5_bl616_bt_ready();
}

} // namespace

extern "C" uint32_t hal_time_ms(void) {
    return static_cast<uint32_t>(bflb_mtimer_get_time_ms());
}

// The embedded run loop guards its trigger flag with these. One core, so
// masking interrupts is enough; nested calls are not made by the run loop.
namespace {
uintptr_t g_hal_irq_flags = 0;
} // namespace

extern "C" void hal_cpu_disable_irqs(void) { g_hal_irq_flags = bflb_irq_save(); }
extern "C" void hal_cpu_enable_irqs(void) { bflb_irq_restore(g_hal_irq_flags); }
extern "C" void hal_cpu_enable_irqs_and_sleep(void) {
    // Deliberately does NOT sleep: the caller is main.cpp's superloop, which
    // still has to service USB and feed the watchdog.
    bflb_irq_restore(g_hal_irq_flags);
}

namespace port {

bool bt_transport_init() {
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_embedded_get_instance());

    const btstack_tlv_t *tlv =
        btstack_tlv_flash_bank_init_instance(&g_tlv_context, &g_flash_bank, nullptr);
    btstack_tlv_set_instance(tlv, &g_tlv_context);

    hci_init(&g_transport, nullptr);

    // After hci_init() (which allocates hci_stack) and before power-on; without
    // a link key db every Link Key Request is answered negatively and PS-only
    // reconnect cannot work. See bt_transport_esp32s31.cpp.
    hci_set_link_key_db(btstack_link_key_db_tlv_get_instance(tlv, &g_tlv_context));

    g_state_cb.callback = &state_handler;
    hci_add_event_handler(&g_state_cb);
    return true;
}

static void emit_packet_sent_if_pending() {
    if (!g_tx_complete) return;
    g_tx_complete = false;
    if (g_packet_handler == nullptr) return;
    uint8_t event[] = {HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    g_packet_handler(HCI_EVENT_PACKET, event, sizeof(event));
}

void bt_transport_poll() {
    while (true) {
        // Release BTstack's packet buffer between dispatches; see the ESP32-S31
        // transport for why this must not wait until the end of the drain.
        emit_packet_sent_if_pending();

        uint32_t got = 0;
        uint8_t len_tag[2];
        const uintptr_t flags = bflb_irq_save();
        if (btstack_ring_buffer_bytes_available(&g_ring) == 0) {
            bflb_irq_restore(flags);
            break;
        }
        btstack_ring_buffer_read(&g_ring, len_tag, sizeof(len_tag), &got);
        const uint32_t len = little_endian_read_16(len_tag, 0);
        btstack_ring_buffer_read(&g_ring, g_rx, len, &got);
        bflb_irq_restore(flags);

        if (g_packet_handler != nullptr && len >= 1) {
            if (g_rx[0] == HCI_ACL_DATA_PACKET) ++g_rx_acl_count;
            g_packet_handler(g_rx[0], &g_rx[1], static_cast<uint16_t>(len - 1));
        }
    }

#if DS5_BL616_DIAG_CDC
    // Debug build: the incoming ACL rate, so the log shows the pad's input
    // reports arriving even while there is no USB device to forward them to.
    // A DualSense streaming input over Bluetooth sends several hundred a second.
    {
        static uint32_t last_ms = 0, last_count = 0;
        const uint32_t now = port::now_ms();
        if (now - last_ms >= 5000) {
            const uint32_t count = g_rx_acl_count;
            if (count != last_count) {
                printf("[BT] incoming ACL: %lu packets in the last %lu ms\n",
                       (unsigned long) (count - last_count), (unsigned long) (now - last_ms));
            }
            last_ms = now;
            last_count = count;
        }
    }
#endif

    {
        static uint32_t reported_drops = 0, reported_events = 0, reported_bad = 0;
        const uint32_t drops = g_rx_dropped;
        if (drops != reported_drops) {
            const uint32_t events = g_rx_dropped_events;
            printf("[BT] HCI rx ring full, dropped %lu packet(s), %lu of them HCI events",
                   (unsigned long) (drops - reported_drops),
                   (unsigned long) (events - reported_events));
            const uint32_t new_events = events - reported_events;
            const uint32_t shown = new_events < kDroppedEventCodes ? new_events : kDroppedEventCodes;
            for (uint32_t i = events - shown; i != events; i++) {
                printf(" 0x%02X", g_rx_dropped_event_code[i % kDroppedEventCodes]);
            }
            printf("\n");
            reported_drops = drops;
            reported_events = events;
        }
        const uint32_t bad = g_rx_acl_bad_len;
        if (bad != reported_bad) {
            printf("[BT] %lu incoming ACL packet(s) with an unexpected length were dropped\n",
                   (unsigned long) (bad - reported_bad));
            reported_bad = bad;
        }
    }

    emit_packet_sent_if_pending();
    btstack_run_loop_embedded_execute_once();

    // The main loop never blocks; give equal-priority tasks (the audio worker)
    // the core once per pass. Higher-priority tasks preempt regardless.
    taskYIELD();
}

} // namespace port
