/*
 * tx_manager.c
 *
 *  Created on: May 14, 2016
 *      Author: surligas
 */

#include "tx_manager.h"
#include "ax25.h"
#include "cc112x_spi.h"

/**
 * This routine sends the data with AX.25 encapuslation.
 * @param in the input buffer holding
 * @param len
 * @return
 */
size_t
tx_data(uint8_t *dev_tx_buffer, uint8_t *dev_rx_buffer,
	const uint8_t *in, size_t len)
{
  int32_t ret = 0;
  /* This routine can not handle large payloads */
  if(len == 0 || len > COMMS_MAX_PAYLOAD_LEN) {
    return 0;
  }

  /* Prepare the AX.25 frame */
  ret = ax25_send(dev_tx_buffer + 2, in, len);
  if(ret < 1){
    return 0;
  }

  /* Issue the frame at the CC1120 */
  cc_tx_data (dev_tx_buffer, ret, dev_rx_buffer, COMMS_DEFAULT_TIMEOUT_MS);
  return len;
}

size_t
tx_large_data(const uint8_t *in, size_t len)
{

}

void
tx_fifo_thr_irq_handler ()
{

}

