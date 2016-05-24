/*
 * rx_manager.c
 *
 *  Created on: May 24, 2016
 *      Author: surligas
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
 * @param timeout_ms the timeout limit in millisecconds
 * @return
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
