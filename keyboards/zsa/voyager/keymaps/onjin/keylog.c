// Typing measurement (KEYLOG=yes builds only), see keylog/README.md.
//
// Every physical key event goes to the HID console before combos, mod-taps
// and tap dances see it:
//   KL <time ms, 16 bit> <d|u> <row> <col> <base layer keycode hex> <layer_state hex>
//
// Pressing both MO(5) keys (bottom outer corners) toggles silent mode: key
// events are still logged but never reach the host, so chords can be
// practised without letters, home-row mods or shortcuts firing.
#include QMK_KEYBOARD_H
#include "print.h"
#include "keymap_introspection.h"

static bool    silent  = false;
static uint8_t corners = 0; // MO(5) keys held

bool pre_process_record_user(uint16_t keycode, keyrecord_t *record) {
    if (!IS_KEYEVENT(record->event)) return true;
    keypos_t k    = record->event.key;
    uint16_t base = keycode_at_keymap_location(0, k.row, k.col);
    uprintf("KL %u %c %u %u %04X %02X\n", record->event.time, record->event.pressed ? 'd' : 'u', k.row, k.col, base,
            (unsigned)layer_state);

    if (base == MO(5)) {
        if (record->event.pressed) {
            if (++corners == 2) {
                silent = !silent;
                uprintf("KL mode %s\n", silent ? "silent" : "normal");
                if (silent) clear_keyboard();
            }
        } else if (corners) {
            corners--;
        }
        if (silent || corners == 2) return false;
    }
    return !silent;
}
