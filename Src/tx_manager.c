/*
 * tx_manager.c
 *
 *  Created on: May 14, 2016
 *      Author: surligas
 */

#include "tx_manager.h"
#include "ax25.h"
#include "cc112x_spi.h"
#include "status.h"


static uint8_t tmp_buf[2 * AX25_MAX_FRAME_LEN];

/**
 * This routine sends the data using the AX.25 encapsulation
 * @param in the input buffer containing the raw data
 * @param len the length of the input buffer
 * @param dev_rx_buffer a buffer that will hold the SPI resulting bytes
 * @param timeout_ms the timeout in miliseconds
 * @return the number of bytes sent or appropriate error code
 */
int32_t
tx_data(const uint8_t *in, size_t len, uint8_t *dev_rx_buffer,
	size_t timeout_ms)
{
  int32_t ret = 0;
  /* This routine can not handle large payloads */
  if(len == 0) {
    return STATUS_NO_DATA;
  }

  if(len > COMMS_MAX_PAYLOAD_LEN) {
    return STATUS_BUFFER_OVERFLOW;
  }

  /* Prepare the AX.25 frame */
  ret = ax25_send(tmp_buf, in, len);
  if(ret < 1){
    return STATUS_NO_DATA;
  }

  /* Issue the frame at the CC1120 */
  ret = cc_tx_data (tmp_buf, ret, dev_rx_buffer, timeout_ms);
  if(ret < 1){
    return ret;
  }
  return len;
}

