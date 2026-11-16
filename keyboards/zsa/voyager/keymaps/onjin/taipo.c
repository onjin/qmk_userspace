// Taipo chording (https://inkeys.wiki/en/keymaps/taipo) for QMK.
//
// Based on dlip's implementation, default flavour:
// https://github.com/dlip/qmk_firmware/blob/chouchou/users/dlip/taipo.c
// Copyright 2023 Dane Lipscombe (@dlip)
//
// The chord map follows the Taipo reference card (inkeys wiki): a chord
// alone, + the Space thumb (shift) or + the Backspace thumb (symbols).
// FN (Tab chord + Backspace thumb) is one-shot: FN, digit = F1-F9 and
// FN, 0, digit = F10-F19. The Enter chord + Backspace thumb is one-shot AltGr.
//
// Changes: table-driven chord map, chords fire TAIPO_CHORD_TERM ms after
// the first key instead of on release, thumb-only chords never auto-repeat,
// and holding the Space thumb of both halves together turns on
// TAIPO_NAV_LAYER until one of them is released.
//
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "taipo.h"

#ifndef TAIPO_TAP_TIMEOUT
#    define TAIPO_TAP_TIMEOUT 150
#endif
// Keys pressed within this many ms of the first one form one chord, which
// then fires without waiting for release.
#ifndef TAIPO_CHORD_TERM
#    define TAIPO_CHORD_TERM 50
#endif

// Bits of one hand, indexed by key % 10.
#define K_R (1 << 0)  // top pinky
#define K_S (1 << 1)  // top ring
#define K_N (1 << 2)  // top middle
#define K_I (1 << 3)  // top index
#define K_A (1 << 4)  // home pinky
#define K_O (1 << 5)  // home ring
#define K_T (1 << 6)  // home middle
#define K_E (1 << 7)  // home index
#define K_IT (1 << 8) // inner thumb: Backspace / symbols
#define K_OT (1 << 9) // outer thumb: Space / shift

// Pseudo keycodes in the chord table, handled in fire().
#define TAIPO_FN QK_USER_0
#define TAIPO_ALTGR QK_USER_1

typedef struct {
    uint8_t  chord;
    uint16_t kc[3]; // alone, + outer thumb (shift), + inner thumb (symbols)
} taipo_chord_t;

// clang-format off
static const taipo_chord_t chords[] = {
    {K_R,             {KC_R,    S(KC_R), KC_GT}},
    {K_S,             {KC_S,    S(KC_S), KC_RCBR}},
    {K_N,             {KC_N,    S(KC_N), KC_RBRC}},
    {K_I,             {KC_I,    S(KC_I), KC_RPRN}},
    {K_A,             {KC_A,    S(KC_A), KC_LT}},
    {K_O,             {KC_O,    S(KC_O), KC_LCBR}},
    {K_T,             {KC_T,    S(KC_T), KC_LBRC}},
    {K_E,             {KC_E,    S(KC_E), KC_LPRN}},
    {K_E | K_O,       {KC_C,    S(KC_C), KC_1}},
    {K_T | K_O,       {KC_U,    S(KC_U), KC_2}},
    {K_T | K_A,       {KC_Q,    S(KC_Q), KC_3}},
    {K_O | K_A,       {KC_L,    S(KC_L), KC_4}},
    {K_I | K_N,       {KC_Y,    S(KC_Y), KC_5}},
    {K_I | K_S,       {KC_F,    S(KC_F), KC_6}},
    {K_N | K_S,       {KC_P,    S(KC_P), KC_7}},
    {K_N | K_R,       {KC_Z,    S(KC_Z), KC_8}},
    {K_S | K_R,       {KC_B,    S(KC_B), KC_9}},
    {K_E | K_T,       {KC_H,    S(KC_H), KC_0}},
    {K_E | K_A,       {KC_D,    S(KC_D), KC_AT}},
    {K_I | K_R,       {KC_G,    S(KC_G), KC_HASH}},
    {K_T | K_R,       {KC_X,    S(KC_X), KC_CIRC}},
    {K_I | K_O,       {KC_K,    S(KC_K), KC_PLUS}},
    {K_E | K_S,       {KC_V,    S(KC_V), KC_ASTR}},
    {K_N | K_A,       {KC_J,    S(KC_J), KC_EQL}},
    {K_E | K_R,       {KC_M,    S(KC_M), KC_DLR}},
    {K_I | K_A,       {KC_W,    S(KC_W), KC_AMPR}},
    {K_T | K_S,       {KC_SLSH, KC_BSLS, KC_PIPE}},
    {K_N | K_O,       {KC_MINS, KC_UNDS, KC_PERC}},
    {K_I | K_T,       {KC_QUES, KC_EXLM, KC_NO}},
    {K_E | K_N,       {KC_COMM, KC_DOT,  KC_TILD}},
    {K_O | K_R,       {KC_SCLN, KC_COLN, KC_NO}},
    {K_S | K_A,       {KC_QUOT, KC_DQUO, KC_GRV}},
    {K_I | K_N | K_S, {KC_TAB,  KC_DEL,  TAIPO_FN}},
    {K_E | K_T | K_O, {KC_ENT,  KC_ESC,  TAIPO_ALTGR}},
    // Same-finger pairs: alone they are modifiers (see pair_mods).
    {K_A | K_R,       {KC_NO,   KC_RGHT, KC_PGUP}},
    {K_O | K_S,       {KC_NO,   KC_UP,   KC_HOME}},
    {K_T | K_N,       {KC_NO,   KC_DOWN, KC_END}},
    {K_E | K_I,       {KC_NO,   KC_LEFT, KC_PGDN}},
};
// clang-format on

typedef struct {
    uint16_t combo;
    uint16_t chord_timer;
    uint16_t timer;
    uint16_t kc;
    uint8_t  mods;
    bool     held;
} hand_t;

static hand_t hands[2];
static bool   nav_active;

// One-shot FN: FN, digit -> F1-F9; FN, 0, digit -> F10-F19.
static enum { FN_OFF, FN_ON, FN_TEN } fn_state;

static void clear_hand(hand_t *h) {
    *h = (hand_t){0};
}

// Fingers pressing only whole same-finger pairs, no thumbs: modifiers.
static uint8_t pair_mods(uint16_t combo) {
    static const struct {
        uint8_t bits;
        uint8_t mod;
    } pairs[] = {
        {K_A | K_R, MOD_BIT(KC_LGUI)},
        {K_O | K_S, MOD_BIT(KC_LALT)},
        {K_T | K_N, MOD_BIT(KC_LCTL)},
        {K_E | K_I, MOD_BIT(KC_LSFT)},
    };
    if (combo & (K_IT | K_OT)) return 0;
    uint8_t mods = 0;
    for (uint8_t p = 0; p < ARRAY_SIZE(pairs); p++) {
        uint8_t hit = combo & pairs[p].bits;
        if (hit == pairs[p].bits) {
            mods |= pairs[p].mod;
        } else if (hit) {
            return 0;
        }
    }
    return mods;
}

static uint16_t resolve(uint16_t combo, uint8_t *mods) {
    uint8_t fingers = combo & 0xFF;
    uint8_t thumbs  = ((combo & K_OT) ? 1 : 0) | ((combo & K_IT) ? 2 : 0);

    *mods = pair_mods(combo);
    if (*mods) return KC_NO;
    if (!fingers) return thumbs == 1 ? KC_SPC : thumbs == 2 ? KC_BSPC : KC_NO;
    if (thumbs == 3) return KC_NO;
    for (uint8_t c = 0; c < ARRAY_SIZE(chords); c++) {
        if (chords[c].chord == fingers) return chords[c].kc[thumbs];
    }
    return KC_NO;
}

// Applies a pending FN to the chord's keycode. Returns false when the
// keycode was swallowed (FN itself, or the 0 of FN, 0, digit).
static bool apply_fn(uint16_t *kc) {
    if (*kc == TAIPO_FN) {
        fn_state = FN_ON;
        *kc      = KC_NO;
        return false;
    }
    if (fn_state == FN_OFF || *kc == KC_NO) return true;
    bool digit = *kc >= KC_1 && *kc <= KC_0; // KC_1..KC_9, KC_0
    uint8_t n  = *kc == KC_0 ? 0 : *kc - KC_1 + 1;
    if (fn_state == FN_ON && digit && n == 0) {
        fn_state = FN_TEN;
        *kc      = KC_NO;
        return false;
    }
    if (digit) {
        if (fn_state == FN_ON) {
            *kc = KC_F1 + n - 1; // 1-9 -> F1-F9
        } else {
            // 0-9 -> F10-F19; F13 and up are not contiguous with F12.
            *kc = n <= 2 ? KC_F10 + n : KC_F13 + n - 3;
        }
    }
    fn_state = FN_OFF;
    return true;
}

static void fire(hand_t *h, bool hold) {
    h->kc = resolve(h->combo, &h->mods);
    if (h->kc == TAIPO_ALTGR) {
        h->kc   = KC_NO;
        h->mods = MOD_BIT(KC_RALT);
    }
    if (!apply_fn(&h->kc)) return;
    if (hold) {
        h->held = true;
        if (h->mods) {
            register_mods(h->mods);
        } else if (h->kc != KC_NO) {
            register_code16(h->kc);
        }
    } else {
        if (h->mods) {
            add_oneshot_mods(h->mods);
        } else if (h->kc != KC_NO) {
            tap_code16(h->kc);
        }
    }
}

static void release_hold(hand_t *h) {
    if (h->mods) {
        unregister_mods(h->mods);
    } else if (h->kc != KC_NO) {
        unregister_code16(h->kc);
    }
}

// A chord is pending in h->combo until it fires. Keys of a fired chord that
// are still down are "spent": their release is ignored, so the next chord on
// the same hand can start before the previous one is fully released.
bool taipo_process_record(uint8_t key, keyrecord_t *record) {
    hand_t  *h   = &hands[key / 10];
    uint16_t bit = 1 << (key % 10);
    uint16_t now = record->event.time;

    if (record->event.pressed) {
        if (h->held) {
            if (h->kc != KC_NO || h->mods) {
                release_hold(h);
                clear_hand(h);
            } else {
                h->held = false; // held an unfinished chord: keep building it
            }
        }
        if (!h->chord_timer) h->chord_timer = (now + TAIPO_CHORD_TERM) | 1;
        h->combo |= bit;
        h->timer = (now + TAIPO_TAP_TIMEOUT) | 1;
        if (!nav_active && hands[0].combo == K_OT && hands[1].combo == K_OT) {
            clear_hand(&hands[0]);
            clear_hand(&hands[1]);
            nav_active = true;
            layer_on(TAIPO_NAV_LAYER);
        }
        return false;
    }

    if (nav_active && bit == K_OT) {
        nav_active = false;
        layer_off(TAIPO_NAV_LAYER);
        clear_hand(h);
        return false;
    }
    if (h->held) {
        release_hold(h);
        clear_hand(h);
    } else if (h->combo & bit) {
        fire(h, false);
        clear_hand(h);
    }
    return false;
}

void taipo_matrix_scan(void) {
    uint16_t now = timer_read();
    for (uint8_t k = 0; k < 2; k++) {
        hand_t *h = &hands[k];
        if (!h->combo || h->held) continue;
        if (h->chord_timer && timer_expired(now, h->chord_timer)) {
            h->chord_timer = 0;
            // Fire finger chords without waiting for release. Modifier pairs
            // and thumbs alone still wait: release = tap, timeout = hold.
            if ((h->combo & 0xFF) && !pair_mods(h->combo)) {
                fire(h, false);
                clear_hand(h);
                continue;
            }
        }
        if (h->timer && timer_expired(now, h->timer)) {
            h->timer = 0;
            // Thumbs alone never hold: Space must wait for release or nav.
            if (h->combo & 0xFF) fire(h, true);
        }
    }
}
