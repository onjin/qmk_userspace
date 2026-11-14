// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "action.h"

// key: 0-9 left hand, 10-19 right hand, in the order
// top pinky, top ring, top middle, top index,
// home pinky, home ring, home middle, home index,
// inner thumb (Backspace), outer thumb (Space).
bool taipo_process_record(uint8_t key, keyrecord_t *record);
void taipo_matrix_scan(void);
