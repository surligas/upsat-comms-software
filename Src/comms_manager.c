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

#include "comms_manager.h"
#include "ax25.h"
#include "status.h"

static uint8_t interm_buf[AX25_MAX_FRAME_LEN + 2];
static uint8_t spi_buf[AX25_MAX_FRAME_LEN + 2];
/**
 * This function receives a valid frame from the uplink interface and extracts
 * its payload.
 * Valid frames are considered those that have passed the AX.25 CRC-16 and
 * have in the Destination Address field the appropriate destination callsign.
 *
 * @param out the output buffer, large enough to hold a valid payload
 * @param len the size of the buffer
 * @param timeout_ms the timeout limit in milliseconds
 * @return the size of the AX.25 payload or appropriate error code
 */
int32_t
recv_payload(uint8_t *out, size_t len, size_t timeout_ms)
{
  int32_t ret;
  uint8_t check;
  if(len > AX25_MAX_FRAME_LEN) {
    return COMMS_STATUS_BUFFER_OVERFLOW;
  }
  ret = rx_data(interm_buf, len, spi_buf, timeout_ms);
  if(ret < 1){
    return ret;
  }

  /* Now check if the frame was indented for us */
  check = ax25_check_dest_addr(interm_buf, (size_t)ret, UPSAT_CALLSIGN);
  if(!check){
    return COMMS_STATUS_INVALID_FRAME;
  }

  /* NOTE: UPSat frames using only Short Address field */
  ret = ax25_extract_payload(out, interm_buf, (size_t) ret, AX25_MIN_ADDR_LEN);
  return ret;
}
