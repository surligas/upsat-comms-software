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

#include "cc112x_spi.h"
#include "cc1120_config.h"
#include "utils.h"
#include "log.h"
#include "status.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
volatile extern uint8_t tx_thr_flag;
volatile extern uint8_t tx_fin_flag;
volatile extern uint8_t rx_sync_flag;
volatile extern uint8_t rx_finished_flag;
volatile extern uint8_t rx_thr_flag;

static uint8_t tx_frag_buf[2 + CC1120_TX_FIFO_SIZE];
static uint8_t rx_spi_tx_buf[2 + CC1120_RX_FIFO_SIZE];
static uint8_t rx_tmp_buf[2 + CC1120_RX_FIFO_SIZE];


/**
 * Delay for \p us microseconds.
 * NOTE: This delay function is not so accurate!!!
 * @param us how many microseconds to delay the execution
 */
static inline void
delay_us(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = 0;
	do  {
	  start++;
	} while(start < cycles);
}

uint8_t
cc_tx_rd_reg (uint16_t add, uint8_t *data)
{

  uint8_t temp_TxBuffer[4];
  uint8_t temp_RxBuffer[4] = { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (add >= CC_EXT_ADD) {
    len = 3;

    temp_TxBuffer[0] = 0xAF;
    temp_TxBuffer[1] = (uint8_t) (0x00FF & add);
    temp_TxBuffer[2] = 0;
  }
  else {
    len = 2;
    /* bit masked for read function */
    add |= 0x0080;
    temp_TxBuffer[0] = (uint8_t) (0x00FF & add);
    temp_TxBuffer[1] = 0;
  }

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi1, (uint8_t *) temp_TxBuffer,
			   (uint8_t *) temp_RxBuffer, len, 5000);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  *data = temp_RxBuffer[len - 1];

  return temp_RxBuffer[0];
}

uint8_t
cc_tx_wr_reg (uint16_t add, uint8_t data)
{

  uint8_t aTxBuffer[4];
  uint8_t aRxBuffer[4] =
    { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (add >= CC_EXT_ADD) {
    len = 3;

    aTxBuffer[0] = 0x2F;
    aTxBuffer[1] = (uint8_t) (0x00FF & add);
    aTxBuffer[2] = data;
  }
  else {
    len = 2;

    aTxBuffer[0] = (uint8_t) (0x00FF & add);
    aTxBuffer[1] = data;
  }

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  delay_us(400);
  HAL_SPI_TransmitReceive (&hspi1, (uint8_t *) aTxBuffer, (uint8_t *) aRxBuffer,
			   len, 5000); //send and receive 3 bytes
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}

uint8_t
cc_tx_cmd (uint8_t CMDStrobe)
{

  uint8_t tx_buf;
  uint8_t rx_buf;

  tx_buf = CMDStrobe;

  /* chip select LOw */
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  /* Send-receive 1 byte */
  HAL_SPI_TransmitReceive (&hspi1, &tx_buf, &rx_buf, sizeof(uint8_t), 5000);

  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*
   * TODO: Return the whole RX buffer
   */
  return rx_buf;
}

/**
 * Write to the TX FIFO \p len bytes using the SPI bus
 * @param data the input buffer containing the data
 * @param spi_rx_data the SPI buffer for the return bytes
 * @param len the number of bytes to be sent
 * @return 0 on success of HAL_StatusTypeDef appropriate error code
 */
HAL_StatusTypeDef
cc_tx_spi_write_fifo(const uint8_t *data, uint8_t *spi_rx_data, size_t len)
{
  HAL_StatusTypeDef ret;
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  delay_us(4);
  ret = HAL_SPI_TransmitReceive (&hspi1, data, spi_rx_data, len,
				 COMMS_DEFAULT_TIMEOUT_MS);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  return ret;
}

/**
 * Send a buffer containing user data into the RF chain.
 * NOTE: No additional headers or space for headers should be appended.
 * his function performs automatically all the necessary padding and
 * header insertions.
 *
 * @param data the buffer containing the data
 * @param size the size of the data
 * @param rec_data receive data buffer for storing the feedback from the SPI
 * @return the number of bytes sent. In case of error -1 is returned
 */
int32_t
cc_tx_data (const uint8_t *data, uint8_t size, uint8_t *rec_data,
	    size_t timeout_ms)
{
  uint8_t gone = 0;
  uint8_t in_fifo = 0;
  uint8_t issue_len;
  uint8_t processed;
  uint8_t first_burst = 1;
  uint8_t frame_size;
  uint32_t start_tick;

  /* Reset the packet transmitted flag */
  tx_fin_flag = 0;

  /*
   * The routine should continue operate until either all bytes have been
   * sent or are already placed in the FIFO and waiting to be sent.
   */
  frame_size = size + 1;
  while( gone + in_fifo < frame_size) {
    tx_thr_flag = 0;

    /*
     * Only the first FIFO burst needs the frame length info.
     * However all the consecutive burst need the BURST TX FIFO flag in front
     */
    if(first_burst) {
      issue_len = min(CC1120_TX_FIFO_SIZE - 1, size);
      tx_frag_buf[0] = BURST_TXFIFO;
      tx_frag_buf[1] = size;
      first_burst = 0;
      memcpy(tx_frag_buf + 2, data, issue_len);

      cc_tx_spi_write_fifo (tx_frag_buf, rec_data, issue_len + 2);
      cc_tx_cmd (STX);
      /* Take into consideration the extra length byte */
      issue_len++;
    }
    else{
      issue_len = min(CC1120_TXFIFO_IRQ_THR, frame_size - gone - in_fifo);
      tx_frag_buf[0] = BURST_TXFIFO;
      memcpy(tx_frag_buf + 1, data + gone + in_fifo - 1, issue_len);
      cc_tx_spi_write_fifo(tx_frag_buf, rec_data, issue_len + 1);
    }

    /* Track the number of bytes in the TX FIFO*/
    in_fifo += issue_len;

    /* If the data in the FIFO is above the IRQ limit wait for that IRQ */
    if (in_fifo >= CC1120_TXFIFO_IRQ_THR && frame_size != issue_len) {
      start_tick = HAL_GetTick();
      while(HAL_GetTick() - start_tick < timeout_ms) {
	if (tx_thr_flag) {
	  break;
	}
      }

      /* Timeout occurred. Abort */
      if(!tx_thr_flag){
	cc_tx_cmd (SFTX);
        return COMMS_STATUS_TIMEOUT;
      }

      processed = in_fifo - CC1120_TXFIFO_IRQ_THR + 1;
      gone += processed;
      in_fifo -= processed;
    }
    else {
      gone += issue_len;
      in_fifo -= issue_len;
    }
  }

  /* Wait the FIFO to empty */
  start_tick = HAL_GetTick();
  while(HAL_GetTick() - start_tick < timeout_ms) {
    if (tx_fin_flag) {
      break;
    }
  }

  cc_tx_cmd (SFTX);
  return gone + in_fifo - 1;
}




uint8_t
cc_rx_rd_reg (uint16_t add, uint8_t *data)
{

  uint8_t temp_TxBuffer[4];
  uint8_t temp_RxBuffer[4] =
    { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (add >= CC_EXT_ADD) {
    len = 3;

    temp_TxBuffer[0] = 0xAF;
    /* extended address */
    temp_TxBuffer[1] = (uint8_t) (0x00FF & add);
    /* send dummy so that i can read data */
    temp_TxBuffer[2] = 0;
  }
  else {
    len = 2;
    /*bit masked for read function*/
    add |= 0x0080;
    /* extended address */
    temp_TxBuffer[0] = (uint8_t) (0x00FF & add);
    temp_TxBuffer[1] = 0;
  }

  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  delay_us(10);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) temp_TxBuffer,
			   (uint8_t *) temp_RxBuffer, len, 5000);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  delay_us(10);
  *data = temp_RxBuffer[len - 1];

  return temp_RxBuffer[0];
}

uint8_t
cc_rx_wr_reg (uint16_t add, uint8_t data)
{

  uint8_t aTxBuffer[4];
  uint8_t aRxBuffer[4] = { 0, 0, 0, 0 };
  uint8_t len = 0;

  if (add >= CC_EXT_ADD) {
    len = 3;

    aTxBuffer[0] = 0x2F;
    /* extended address */
    aTxBuffer[1] = (uint8_t) (0x00FF & add);
    /* send dummy so that i can read data */
    aTxBuffer[2] = data;
  }
  else {
    len = 2;
    /* extended address */
    aTxBuffer[0] = (uint8_t) (0x00FF & add);
    /* send dummy so that i can read data */
    aTxBuffer[1] = data;
  }

  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  delay_us(20);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) aTxBuffer, (uint8_t *) aRxBuffer,
			   len, 5000);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}

/**
 * Get a frame from the air. This method blocks for \p timeout_ms milliseconds
 * until a valid frame is received.
 * @param out the output buffer
 * @param len the size of the output buffer
 * @param timeout_ms the timeout period in milliseconds
 * @return the size of the frame in bytes or a negative number indicating
 * the appropriate error
 */
int32_t
cc_rx_data(uint8_t *out, size_t len, size_t timeout_ms)
{
  HAL_StatusTypeDef ret;
  uint8_t frame_len;
  uint8_t received = 0;
  uint32_t start_tick;
  uint8_t timeout = 1;
  uint8_t rx_n_bytes;

  /*Reset all the RX-related flags */
  rx_sync_flag = 0;
  rx_thr_flag = 0;
  rx_finished_flag = 0;

  /* Start the reception by issuing the start-RX command */
  cc_rx_cmd(SFRX);
  cc_rx_cmd(SRX);
  start_tick = HAL_GetTick();

  /* Now wait for the SYNC word to be received */
  while(HAL_GetTick() - start_tick < timeout_ms) {
    if(rx_sync_flag) {
      timeout = 0;
      break;
    }
  }

  /* Timeout occurred, just return */
  if(timeout){
    cc_rx_cmd(SFRX);
    cc_rx_cmd(SIDLE);
    return COMMS_STATUS_TIMEOUT;
  }

  /*
   * Time to extract the frame length,. This is indicated by the first byte
   * after the SYNC word
   */
  do {
      cc_rx_rd_reg(NUM_RXBYTES, &rx_n_bytes);
  } while (rx_n_bytes < 2);

  /* One byte FIFO access */
  cc_rx_rd_reg(SINGLE_RXFIFO, &frame_len);

  /*
   * Now that we have the frame length check if the FIFO should be dequeued
   * multiple times or not
   */
  while(frame_len - received > CC1120_RXFIFO_THR) {
    /* Wait for the RX FIFO above threshold interrupt */
    start_tick = HAL_GetTick ();
    timeout = 1;
    /*Reset the flag */
    rx_thr_flag = 0;
    while (HAL_GetTick () - start_tick < timeout_ms) {
      if (rx_thr_flag) {
	timeout = 0;
	break;
      }
    }

    if (timeout) {
      cc_rx_cmd (SFRX);
      cc_rx_cmd(SIDLE);
      return COMMS_STATUS_TIMEOUT;
    }

    /* We can now dequeue CC1120_BYTES_IN_RX_FIF0 bytes */
    ret = cc_rx_spi_read_fifo(out + received, CC1120_BYTES_IN_RX_FIF0);

    if(ret) {
      cc_rx_cmd(SFRX);
      cc_rx_cmd(SIDLE);
      return COMMS_STATUS_NO_DATA;
    }
    received += CC1120_BYTES_IN_RX_FIF0;
  }

  /* Wait for the packet end interrupt */
  start_tick = HAL_GetTick ();
  timeout = 1;
  while (HAL_GetTick () - start_tick < timeout_ms) {
    if (rx_finished_flag) {
      timeout = 0;
      break;
    }
  }
  if(timeout){
    cc_rx_cmd(SFRX);
    cc_rx_cmd(SIDLE);
    return COMMS_STATUS_TIMEOUT;
  }

  /* Now dequeue the remaining bytes in the FIFO if any left*/
  if (frame_len - received > 1) {
    ret = cc_rx_spi_read_fifo (out + received, frame_len - received);
    if (ret) {
      cc_rx_cmd (SFRX);
      cc_rx_cmd(SIDLE);
      return COMMS_STATUS_NO_DATA;
    }
  }
  else if(frame_len - received == 1) {
    /* One byte FIFO access */
    cc_rx_rd_reg(SINGLE_RXFIFO, out + received);
  }
  cc_rx_cmd(SFRX);
  cc_rx_cmd(SIDLE);
  return frame_len;
}


/**
 * Read len bytes from the RX FIFO using the SPI
 * @param out the output buffer
 * @param len the number of bytes to be read
 * @return 0 on success of HAL_StatusTypeDef appropriate error code
 */
HAL_StatusTypeDef
cc_rx_spi_read_fifo(uint8_t *out, size_t len)
{
  HAL_StatusTypeDef ret;
  /* Reset the send buffer */
  memset(rx_spi_tx_buf, 0, sizeof(rx_spi_tx_buf));
  rx_spi_tx_buf[0] = BURST_RXFIFO;
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_RESET);
  delay_us(4);
  /* Remove the response SPI byte */
  ret = HAL_SPI_TransmitReceive (&hspi2, rx_spi_tx_buf, rx_tmp_buf, len + 1,
				 COMMS_DEFAULT_TIMEOUT_MS);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_SET);
  memcpy(out, rx_tmp_buf + 1, len);
  return ret;
}

uint8_t
cc_rx_cmd (uint8_t CMDStrobe)
{

  uint8_t aTxBuffer[1];
  uint8_t aRxBuffer[1];

  aTxBuffer[0] = CMDStrobe;

  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t*) aTxBuffer, (uint8_t *) aRxBuffer,
			   1, COMMS_DEFAULT_TIMEOUT_MS);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}

