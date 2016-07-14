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

#ifndef INC_COMMS_MANAGER_H_
#define INC_COMMS_MANAGER_H_

#include "rx_manager.h"
#include "tx_manager.h"
#include "config.h"

#define FRAME_OK	1

int32_t
recv_payload(uint8_t *out, size_t len, size_t timeout_ms);

int32_t
send_payload(const uint8_t *in, size_t len, size_t timeout_ms);

int32_t
send_payload_cw(const uint8_t *in, size_t len);

int32_t
send_cw_beacon();

uint8_t
is_tx_enabled();

void
comms_init();

int32_t
comms_routine_dispatcher(uint8_t *send_cw);


#endif /* INC_COMMS_MANAGER_H_ */
