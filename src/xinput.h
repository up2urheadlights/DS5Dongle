//
// XInput (Xbox 360 compatible) function, for Xbox Game Bar navigation.
//

#ifndef DS5_BRIDGE_XINPUT_H
#define DS5_BRIDGE_XINPUT_H

#include <cstdint>

// Feed the XInput function from a DualSense input report body -- the same
// buffer ps_shortcut_tick() and wake_on_bt_input() are handed, with the button
// bytes at offsets 7/8/9. No-op unless the Game Bar shortcut is enabled and the
// host has configured the interface.
void xinput_tick(const uint8_t *body, uint16_t len);

// Release any held navigation state, e.g. when the controller disconnects, so
// the host does not see a D-pad direction stuck down.
void xinput_reset_state();

#endif //DS5_BRIDGE_XINPUT_H
