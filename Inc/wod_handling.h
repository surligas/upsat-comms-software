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

#ifndef INC_WOD_HANDLING_H_
#define INC_WOD_HANDLING_H_

#include <stdint.h>
#include <stdlib.h>
#include "utils.h"
#include <math.h>

/**
 * The number of bytes in every WOD data set WITHOUT the mode bit
 */
#define WOD_DATASET_SIZE 7

/**
 * The maximum number of datasets that each WOD can carry
 */
#define WOD_MAX_DATASETS 32

int32_t
prepare_wod(uint8_t *wod, const uint8_t *obc_wod, size_t len);

uint8_t
wod_convert_temperature(float val);

#endif /* INC_WOD_HANDLING_H_ */
