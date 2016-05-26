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

#include "rx_manager.h"
#include "ax25.h"
#include "cc112x_spi.h"
#include "status.h"

static uint8_t tmp_buf[AX25_MAX_FRAME_LEN + 2];

/**
 * Received and decodes using the AX.25 encapsulation a new frame.
 * This is a blocking call. It will block either until a frame is received
 * or the timeout limit is reached
 * @param out the output buffer
 * @param len the length of the output buffer
 * @param dev_rx_buffer a buffer that will hold the SPI resulting bytes
 * @param timeout_ms the timeout limit in milliseconds
 * @return the number of bytes received and decoded or appropriate error code.
 * Note that this function does not perform any AX.25 header extraction
 */
int32_t
rx_data(uint8_t *out, size_t len, uint8_t *dev_rx_buffer, size_t timeout_ms)
{
  int32_t ret;

  ret = cc_rx_data(tmp_buf, AX25_MAX_FRAME_LEN, COMMS_DEFAULT_TIMEOUT_MS);
  if(ret < 1){
    return ret;
  }

  /* Frame received. Try to decode it using the AX.25 encapsulation */
  ret = ax25_recv(out, tmp_buf, ret);
  return ret;
}
