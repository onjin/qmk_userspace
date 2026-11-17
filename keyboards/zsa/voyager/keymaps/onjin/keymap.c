#include QMK_KEYBOARD_H
#include "version.h"
#include "taipo.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  HSV_246_109_243,
  HSV_74_255_255,
  HSV_155_115_250,
  ST_MACRO_0,
  // Taipo keys, order must match taipo.h
  TP_TLP, TP_TLR, TP_TLM, TP_TLI, TP_BLP, TP_BLR, TP_BLM, TP_BLI, TP_LIT, TP_LOT,
  TP_TRP, TP_TRR, TP_TRM, TP_TRI, TP_BRP, TP_BRR, TP_BRM, TP_BRI, TP_RIT, TP_ROT,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_ESCAPE,      KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           TD(DANCE_0),    
    KC_TAB,         KC_Q,           KC_W,           KC_E,           KC_R,           KC_T,                                           KC_Y,           KC_U,           KC_I,           KC_O,           KC_P,           TD(DANCE_1),    
    KC_MEH,         MT(MOD_LGUI, KC_A),MT(MOD_LALT, KC_S),MT(MOD_LCTL, KC_D),MT(MOD_LSFT, KC_F),ALL_T(KC_G),                                    ALL_T(KC_H),    MT(MOD_RSFT, KC_J),MT(MOD_RCTL, KC_K),MT(MOD_LALT, KC_L),MT(MOD_RGUI, KC_SCLN),KC_MEH,         
    MO(5),          KC_Z,           MT(MOD_RALT, KC_X),KC_C,           KC_V,           MEH_T(KC_B),                                    MEH_T(KC_N),    KC_M,           KC_COMMA,       MT(MOD_RALT, KC_DOT),KC_SLASH,       MO(5),          
                                                    LT(4, KC_SPACE),KC_BSPC,                                        LT(3, KC_ENTER),LT(2, KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          TD(DANCE_2),
    KC_NO,          TP_TLP,         TP_TLR,         TP_TLM,         TP_TLI,         KC_NO,                                          KC_NO,          TP_TRI,         TP_TRM,         TP_TRR,         TP_TRP,         KC_NO,
    KC_NO,          TP_BLP,         TP_BLR,         TP_BLM,         TP_BLI,         KC_NO,                                          KC_NO,          TP_BRI,         TP_BRM,         TP_BRR,         TP_BRP,         KC_NO,
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,
                                                    TP_LOT,         TP_LIT,                                         TP_RIT,         TP_ROT
  ),
  [2] = LAYOUT_voyager(
    ST_MACRO_0,     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_F6,          
    KC_TRANSPARENT, KC_LBRC,        KC_7,           KC_8,           KC_9,           KC_RBRC,                                        KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         KC_F12,         
    KC_TRANSPARENT, KC_QUOTE,       KC_4,           KC_5,           KC_6,           KC_EQUAL,                                       KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_BSLS,                                        KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_0,           KC_MINUS,                                       KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_LCBR,        KC_AMPR,        KC_KP_ASTERISK, KC_LPRN,        KC_RCBR,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_DQUO,        KC_DLR,         KC_PERC,        KC_CIRC,        KC_KP_PLUS,                                     KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TILD,        KC_EXLM,        KC_AT,          KC_HASH,        KC_PIPE,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_RPRN,        KC_UNDS,                                        KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_MS_WH_LEFT,  KC_MS_WH_DOWN,  KC_MS_WH_UP,    KC_MS_WH_RIGHT, KC_MS_BTN3,     KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_AUDIO_MUTE,  KC_TRANSPARENT,                                 KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_UP,       KC_MS_RIGHT,    KC_MS_BTN2,     KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_MEDIA_STOP,  KC_MEDIA_PLAY_PAUSE,KC_TRANSPARENT,                                 KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       CW_TOGG,        KC_TRANSPARENT, 
    KC_TRANSPARENT, LALT(KC_RIGHT), KC_TRANSPARENT, HSV_246_109_243,HSV_74_255_255, HSV_155_115_250,                                KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_INSERT,      KC_DELETE,      
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_MS_BTN1,     KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_0,           KC_9,           KC_8,           KC_7,           KC_6,                                           KC_5,           KC_4,           KC_3,           KC_2,           KC_1,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_P,           KC_O,           KC_I,           KC_U,           KC_Y,                                           KC_T,           KC_R,           KC_E,           KC_W,           KC_Q,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_SCLN,        KC_L,           KC_K,           KC_J,           KC_H,                                           KC_G,           KC_F,           KC_D,           KC_S,           KC_A,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_SLASH,       KC_DOT,         KC_COMMA,       KC_M,           KC_N,                                           KC_B,           KC_V,           KC_C,           KC_X,           KC_Z,           KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_ENTER,                                       KC_BSPC,        KC_TRANSPARENT
  ),
  [6] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_BSPC,        
    MT(MOD_LGUI, KC_TAB),KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, TD(DANCE_3),    
    KC_LEFT_SHIFT,  KC_A,           KC_S,           KC_D,           KC_F,           KC_G,                                           KC_H,           KC_J,           KC_K,           KC_L,           KC_SCLN,        KC_RIGHT_SHIFT, 
    KC_LEFT_CTRL,   KC_TRANSPARENT, KC_X,           KC_TRANSPARENT, KC_TRANSPARENT, KC_B,                                           KC_N,           KC_TRANSPARENT, KC_TRANSPARENT, KC_DOT,         KC_TRANSPARENT, KC_RIGHT_CTRL,  
                                                    KC_SPACE,       KC_LEFT_ALT,                                    KC_ENTER,       KC_RIGHT_ALT
  ),
  [7] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_MS_WH_LEFT,  KC_MS_WH_DOWN,  KC_MS_WH_UP,    KC_MS_WH_RIGHT, KC_MS_BTN3,     KC_TRANSPARENT,
    KC_TRANSPARENT, LCTL(KC_Z),     LCTL(KC_X),     LCTL(KC_C),     LCTL(KC_V),     KC_TRANSPARENT,                                 KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_UP,       KC_MS_RIGHT,    KC_MS_BTN2,     KC_TRANSPARENT,
    KC_TRANSPARENT, KC_LEFT_GUI,    KC_LEFT_ALT,    KC_LEFT_CTRL,   KC_LEFT_SHIFT,  KC_TRANSPARENT,                                 KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       CW_TOGG,        KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_INSERT,      KC_DELETE,
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};


// Base layer combos: the Taipo chords (see taipo.c) on QWER/ASDF and
// UIOP/JKL;. The inner thumb (Backspace / Enter) is the Taipo symbol thumb,
// the outer thumb (Space) the Taipo shift thumb. Only chords that need a
// thumb or three fingers are mapped, two-finger letter chords would misfire
// while typing.
#define L_A MT(MOD_LGUI, KC_A)
#define L_S MT(MOD_LALT, KC_S)
#define L_D MT(MOD_LCTL, KC_D)
#define L_F MT(MOD_LSFT, KC_F)
#define L_OT LT(4, KC_SPACE)
#define L_IT KC_BSPC
#define R_J MT(MOD_RSFT, KC_J)
#define R_K MT(MOD_RCTL, KC_K)
#define R_L MT(MOD_LALT, KC_L)
#define R_SC MT(MOD_RGUI, KC_SCLN)
#define R_IT LT(3, KC_ENTER)
#define R_OT LT(2, KC_SPACE)

// Generated from the Taipo chord table, left hand first.
enum combo_events {
  CB_L_LPRN, CB_L_LBRC, CB_L_LCBR, CB_L_LT, CB_L_RPRN, CB_L_RBRC, CB_L_RCBR, CB_L_GT,
  CB_L_1, CB_L_2, CB_L_3, CB_L_4, CB_L_5, CB_L_6, CB_L_7, CB_L_8,
  CB_L_9, CB_L_0, CB_L_AT, CB_L_HASH, CB_L_CIRC, CB_L_PLUS, CB_L_ASTR, CB_L_EQL,
  CB_L_DLR, CB_L_AMPR, CB_L_PIPE, CB_L_PERC, CB_L_TILD, CB_L_GRV, CB_L_ENT, CB_L_ESC,
  CB_L_TAB, CB_R_LPRN, CB_R_LBRC, CB_R_LCBR, CB_R_LT, CB_R_RPRN, CB_R_RBRC, CB_R_RCBR,
  CB_R_GT, CB_R_1, CB_R_2, CB_R_3, CB_R_4, CB_R_5, CB_R_6, CB_R_7,
  CB_R_8, CB_R_9, CB_R_0, CB_R_AT, CB_R_HASH, CB_R_CIRC, CB_R_PLUS, CB_R_ASTR,
  CB_R_EQL, CB_R_DLR, CB_R_AMPR, CB_R_PIPE, CB_R_PERC, CB_R_TILD, CB_R_GRV, CB_R_ENT,
  CB_R_ESC, CB_R_TAB,
};

const uint16_t PROGMEM cb_l_lprn[] = { L_F, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_lbrc[] = { L_D, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_lcbr[] = { L_S, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_lt[] = { L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_rprn[] = { KC_R, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_rbrc[] = { KC_E, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_rcbr[] = { KC_W, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_gt[] = { KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_1[] = { L_F, L_S, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_2[] = { L_D, L_S, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_3[] = { L_D, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_4[] = { L_S, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_5[] = { KC_R, KC_E, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_6[] = { KC_R, KC_W, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_7[] = { KC_E, KC_W, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_8[] = { KC_E, KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_9[] = { KC_W, KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_0[] = { L_F, L_D, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_at[] = { L_F, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_hash[] = { KC_R, KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_circ[] = { L_D, KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_plus[] = { KC_R, L_S, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_astr[] = { L_F, KC_W, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_eql[] = { KC_E, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_dlr[] = { L_F, KC_Q, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_ampr[] = { KC_R, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_pipe[] = { L_D, KC_W, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_perc[] = { KC_E, L_S, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_tild[] = { L_F, KC_E, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_grv[] = { KC_W, L_A, L_IT, COMBO_END};
const uint16_t PROGMEM cb_l_ent[] = { L_F, L_D, L_S, COMBO_END};
const uint16_t PROGMEM cb_l_esc[] = { L_F, L_D, L_S, L_OT, COMBO_END};
const uint16_t PROGMEM cb_l_tab[] = { KC_R, KC_E, KC_W, COMBO_END};
const uint16_t PROGMEM cb_r_lprn[] = { R_J, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_lbrc[] = { R_K, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_lcbr[] = { R_L, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_lt[] = { R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_rprn[] = { KC_U, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_rbrc[] = { KC_I, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_rcbr[] = { KC_O, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_gt[] = { KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_1[] = { R_J, R_L, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_2[] = { R_K, R_L, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_3[] = { R_K, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_4[] = { R_L, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_5[] = { KC_U, KC_I, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_6[] = { KC_U, KC_O, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_7[] = { KC_I, KC_O, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_8[] = { KC_I, KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_9[] = { KC_O, KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_0[] = { R_J, R_K, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_at[] = { R_J, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_hash[] = { KC_U, KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_circ[] = { R_K, KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_plus[] = { KC_U, R_L, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_astr[] = { R_J, KC_O, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_eql[] = { KC_I, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_dlr[] = { R_J, KC_P, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_ampr[] = { KC_U, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_pipe[] = { R_K, KC_O, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_perc[] = { KC_I, R_L, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_tild[] = { R_J, KC_I, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_grv[] = { KC_O, R_SC, R_IT, COMBO_END};
const uint16_t PROGMEM cb_r_ent[] = { R_J, R_K, R_L, COMBO_END};
const uint16_t PROGMEM cb_r_esc[] = { R_J, R_K, R_L, R_OT, COMBO_END};
const uint16_t PROGMEM cb_r_tab[] = { KC_U, KC_I, KC_O, COMBO_END};

combo_t key_combos[] = {
    [CB_L_LPRN] = COMBO(cb_l_lprn, KC_LPRN),
    [CB_L_LBRC] = COMBO(cb_l_lbrc, KC_LBRC),
    [CB_L_LCBR] = COMBO(cb_l_lcbr, KC_LCBR),
    [CB_L_LT] = COMBO(cb_l_lt, KC_LT),
    [CB_L_RPRN] = COMBO(cb_l_rprn, KC_RPRN),
    [CB_L_RBRC] = COMBO(cb_l_rbrc, KC_RBRC),
    [CB_L_RCBR] = COMBO(cb_l_rcbr, KC_RCBR),
    [CB_L_GT] = COMBO(cb_l_gt, KC_GT),
    [CB_L_1] = COMBO(cb_l_1, KC_1),
    [CB_L_2] = COMBO(cb_l_2, KC_2),
    [CB_L_3] = COMBO(cb_l_3, KC_3),
    [CB_L_4] = COMBO(cb_l_4, KC_4),
    [CB_L_5] = COMBO(cb_l_5, KC_5),
    [CB_L_6] = COMBO(cb_l_6, KC_6),
    [CB_L_7] = COMBO(cb_l_7, KC_7),
    [CB_L_8] = COMBO(cb_l_8, KC_8),
    [CB_L_9] = COMBO(cb_l_9, KC_9),
    [CB_L_0] = COMBO(cb_l_0, KC_0),
    [CB_L_AT] = COMBO(cb_l_at, KC_AT),
    [CB_L_HASH] = COMBO(cb_l_hash, KC_HASH),
    [CB_L_CIRC] = COMBO(cb_l_circ, KC_CIRC),
    [CB_L_PLUS] = COMBO(cb_l_plus, KC_PLUS),
    [CB_L_ASTR] = COMBO(cb_l_astr, KC_ASTR),
    [CB_L_EQL] = COMBO(cb_l_eql, KC_EQL),
    [CB_L_DLR] = COMBO(cb_l_dlr, KC_DLR),
    [CB_L_AMPR] = COMBO(cb_l_ampr, KC_AMPR),
    [CB_L_PIPE] = COMBO(cb_l_pipe, KC_PIPE),
    [CB_L_PERC] = COMBO(cb_l_perc, KC_PERC),
    [CB_L_TILD] = COMBO(cb_l_tild, KC_TILD),
    [CB_L_GRV] = COMBO(cb_l_grv, KC_GRV),
    [CB_L_ENT] = COMBO(cb_l_ent, KC_ENTER),
    [CB_L_ESC] = COMBO(cb_l_esc, KC_ESCAPE),
    [CB_L_TAB] = COMBO(cb_l_tab, KC_TAB),
    [CB_R_LPRN] = COMBO(cb_r_lprn, KC_LPRN),
    [CB_R_LBRC] = COMBO(cb_r_lbrc, KC_LBRC),
    [CB_R_LCBR] = COMBO(cb_r_lcbr, KC_LCBR),
    [CB_R_LT] = COMBO(cb_r_lt, KC_LT),
    [CB_R_RPRN] = COMBO(cb_r_rprn, KC_RPRN),
    [CB_R_RBRC] = COMBO(cb_r_rbrc, KC_RBRC),
    [CB_R_RCBR] = COMBO(cb_r_rcbr, KC_RCBR),
    [CB_R_GT] = COMBO(cb_r_gt, KC_GT),
    [CB_R_1] = COMBO(cb_r_1, KC_1),
    [CB_R_2] = COMBO(cb_r_2, KC_2),
    [CB_R_3] = COMBO(cb_r_3, KC_3),
    [CB_R_4] = COMBO(cb_r_4, KC_4),
    [CB_R_5] = COMBO(cb_r_5, KC_5),
    [CB_R_6] = COMBO(cb_r_6, KC_6),
    [CB_R_7] = COMBO(cb_r_7, KC_7),
    [CB_R_8] = COMBO(cb_r_8, KC_8),
    [CB_R_9] = COMBO(cb_r_9, KC_9),
    [CB_R_0] = COMBO(cb_r_0, KC_0),
    [CB_R_AT] = COMBO(cb_r_at, KC_AT),
    [CB_R_HASH] = COMBO(cb_r_hash, KC_HASH),
    [CB_R_CIRC] = COMBO(cb_r_circ, KC_CIRC),
    [CB_R_PLUS] = COMBO(cb_r_plus, KC_PLUS),
    [CB_R_ASTR] = COMBO(cb_r_astr, KC_ASTR),
    [CB_R_EQL] = COMBO(cb_r_eql, KC_EQL),
    [CB_R_DLR] = COMBO(cb_r_dlr, KC_DLR),
    [CB_R_AMPR] = COMBO(cb_r_ampr, KC_AMPR),
    [CB_R_PIPE] = COMBO(cb_r_pipe, KC_PIPE),
    [CB_R_PERC] = COMBO(cb_r_perc, KC_PERC),
    [CB_R_TILD] = COMBO(cb_r_tild, KC_TILD),
    [CB_R_GRV] = COMBO(cb_r_grv, KC_GRV),
    [CB_R_ENT] = COMBO(cb_r_ent, KC_ENTER),
    [CB_R_ESC] = COMBO(cb_r_esc, KC_ESCAPE),
    [CB_R_TAB] = COMBO(cb_r_tab, KC_TAB),
};

// Three- and four-key chords get the Taipo chord term. QMK waits for the
// longest term of all candidates, so finger + thumb waits for it too.
uint16_t get_combo_term(uint16_t index, combo_t *combo) {
    uint8_t n = 0;
    for (const uint16_t *k = combo->keys; pgm_read_word(k) != COMBO_END; k++) n++;
    return n >= 3 ? TAIPO_CHORD_TERM : COMBO_TERM;
}



extern rgb_config_t rgb_matrix_config;

RGB hsv_to_rgb_with_value(HSV hsv) {
  RGB rgb = hsv_to_rgb( hsv );
  float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
  return (RGB){ f * rgb.r, f * rgb.g, f * rgb.b };
}

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255} },

    [1] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,70}, {169,255,70}, {169,255,70}, {169,255,70}, {0,0,0}, {0,0,0}, {169,255,70}, {169,255,70}, {169,255,70}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {169,255,70}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {85,255,255}, {0,0,0}, {169,255,70}, {169,255,70}, {169,255,70}, {169,255,70}, {0,0,0}, {0,0,0}, {169,255,255}, {169,255,70}, {169,255,70}, {169,255,70}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,70}, {169,255,255} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [5] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0} },

    [6] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {191,83,247}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {191,83,247}, {0,0,0}, {191,83,247}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [7] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,70}, {169,255,70}, {169,255,70}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {246,109,243}, {246,109,243}, {246,109,243}, {246,109,243}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {169,255,255} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb_with_value(hsv);
        rgb_matrix_set_color(i, rgb.r, rgb.g, rgb.b);
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (!keyboard_config.disable_layer_led) { 
    switch (biton32(layer_state)) {
      case 0:
        set_layer_color(0);
        break;
      case 1:
        set_layer_color(1);
        break;
      case 2:
        set_layer_color(2);
        break;
      case 4:
        set_layer_color(4);
        break;
      case 5:
        set_layer_color(5);
        break;
      case 6:
        set_layer_color(6);
        break;
      case 7:
        set_layer_color(7);
        break;
     default:
        if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
          rgb_matrix_set_color_all(0, 0, 0);
        }
    }
  } else {
    if (rgb_matrix_get_flags() == LED_FLAG_NONE) {
      rgb_matrix_set_color_all(0, 0, 0);
    }
  }

  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,      
    SINGLE_HOLD,         
    DOUBLE_TAP,          
    DOUBLE_HOLD,         
    DOUBLE_SINGLE_TAP,   
    MORE_TAPS            
};

static tap dance_state[4];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case DOUBLE_TAP: layer_move(1); break;
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
    }
    dance_state[0].step = 0;
}
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case DOUBLE_TAP: layer_move(6); break;
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
    }
    dance_state[1].step = 0;
}
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case DOUBLE_TAP: layer_move(0); break;
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
    }
    dance_state[2].step = 0;
}
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case DOUBLE_TAP: layer_move(0); break;
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
    }
    dance_state[3].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_3_finished, dance_3_reset),
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  if (keycode >= TP_TLP && keycode <= TP_ROT) {
    return taipo_process_record(keycode - TP_TLP, record);
  }
  switch (keycode) {
  case QK_MODS ... QK_MODS_MAX:
    // Mouse and consumer keys (volume, media) with modifiers work inconsistently across operating systems,
    // this makes sure that modifiers are always applied to the key that was pressed.
    if (IS_MOUSE_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode)) || IS_CONSUMER_KEYCODE(QK_MODS_GET_BASIC_KEYCODE(keycode))) {
      if (record->event.pressed) {
        add_mods(QK_MODS_GET_MODS(keycode));
        send_keyboard_report();
        wait_ms(2);
        register_code(QK_MODS_GET_BASIC_KEYCODE(keycode));
        return false;
      } else {
        wait_ms(2);
        del_mods(QK_MODS_GET_MODS(keycode));
      }
    }
    break;
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_ESCAPE)SS_DELAY(100)  SS_LSFT(SS_TAP(X_SCLN))SS_DELAY(100)  SS_TAP(X_W)SS_DELAY(100)  SS_TAP(X_Q)  SS_DELAY(100) SS_TAP(X_ENTER));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_246_109_243:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(246,109,243);
      }
      return false;
    case HSV_74_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(74,255,255);
      }
      return false;
    case HSV_155_115_250:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(155,115,250);
      }
      return false;
  }
  return true;
}

void matrix_scan_user(void) {
  taipo_matrix_scan();
}
