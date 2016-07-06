/*
 * upsat-comms-software: Communication Subsystem Software for UPSat satellite
 *
 *  Copyright (C) 2016, Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INC_CW_H_
#define INC_CW_H_

#include <stdlib.h>
#include <stdint.h>

/**
 * The different CW symbols
 */
typedef enum {
  CW_DOT = 0,          //!< CW_DOT a dot (pulse of short duration)
  CW_DASH = 1,         //!< CW_DASH a dash (pulse of long duration)
  CW_CHAR_DELIM = 2,   //!< CW_CHAR_DELIM character pause delimiter
  CW_WORD_DELIM = 3,   //!< CW_WORD_DELIM word pause delimiter
  CW_SYMBOL_DELIM = 4, //!< CW_SYMBOL_DELIM symbol pause delimiter
  CW_INVALID = 5       //!< CW_INVALID invalid symbol
} cw_symbol_t;

/**
 * The status of the CW encoding/transmission
 */
typedef enum {
  CW_OK = 0,   //!< CW_OK all ok
  CW_ERROR = -1//!< CW_ERROR an error occured
} cw_status_t;

/**
 * A CW character and its mapping to CW symbols
 */
typedef struct {
  cw_symbol_t s[10];
  uint8_t s_num;
  uint8_t is_valid;
} cw_char_t;


void
cw_init ();

int32_t
cw_encode(cw_char_t *out, const uint8_t *in, size_t len);

#endif /* INC_CW_H_ */
