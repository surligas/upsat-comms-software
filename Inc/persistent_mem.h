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

#ifndef INC_PERSISTENT_MEM_H_
#define INC_PERSISTENT_MEM_H_

#include <stdint.h>
#include <stdlib.h>

#define COMMS_PERSIST_MEM_BASE_ADDR 0x0

void
comms_write_persistent_word(uint32_t word);

uint32_t
comms_read_persistent_word();

#endif /* INC_PERSISTENT_MEM_H_ */
