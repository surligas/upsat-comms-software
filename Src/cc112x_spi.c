#include "cc112x_spi.h"
#include "cc1120_config.h"
#include "utils.h"
#include "log.h"
#include "status.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart5;
extern uint8_t uart_temp[100];
extern uint8_t AX_aRxBuffer[];
volatile extern uint8_t tx_thr_flag;
volatile extern uint8_t tx_fin_flag;
volatile extern uint8_t rx_sync_flag;
volatile extern uint8_t rx_finished_flag;
volatile extern uint8_t rx_thr_flag;

static uint8_t tx_frag_buf[2 + CC1120_TX_FIFO_SIZE];
static uint8_t rx_spi_tx_buf[CC1120_RX_FIFO_SIZE];


static inline void
delayUS_DWT(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = 0;
	do  {
	  start++;
	} while(start < cycles);
}

uint8_t
cc_tx_readReg (uint16_t add, uint8_t *data)
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
cc_tx_writeReg (uint16_t add, uint8_t data)
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
  HAL_Delay (1);
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
  uint8_t ret;
  uint8_t gone = 0;
  uint8_t in_fifo = 0;
  uint8_t issue_len;
  uint8_t res_fifo[6];
  uint8_t first_burst = 1;
  uint32_t start_tick;

  /* Reset the packet transmitted flag */
  tx_fin_flag = 0;

  /*
   * The routine should continue operate until either all bytes have been
   * sent or are already placed in the FIFO and waiting to be sent.
   */
  size++;
  while( gone + in_fifo < size) {
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

      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive (&hspi1, tx_frag_buf, rec_data,
  			     issue_len + 2, 1);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

      ret = cc_tx_cmd (STX);
      /* Take into consideration the extra length byte */
      issue_len++;
    }
    else{
      issue_len = min(CC1120_TX_FIFO_SIZE - in_fifo, size - gone - in_fifo);
      tx_frag_buf[0] = BURST_TXFIFO;
      memcpy(tx_frag_buf + 1, data + gone + in_fifo - 1, issue_len);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive (&hspi1, tx_frag_buf, rec_data, issue_len + 1,
			       1);
      HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }

    /* Track the number of bytes in the TX FIFO*/
    in_fifo += issue_len;

    /* If the data in the FIFO is above the IRQ limit wait for that IRQ */
    if (in_fifo > CC1120_TXFIFO_THR && size != issue_len) {
      start_tick = HAL_GetTick();
      while(HAL_GetTick() - start_tick < timeout_ms) {
	if (tx_thr_flag) {
	  break;
	}
      }

      /* Timeout occurred. Abort */
      if(!tx_thr_flag){
	LOG_UART_ERROR(&huart5, "Timeout while trying to send data.");
	cc_tx_readReg(MODEM_STATUS0, res_fifo);
	ret = cc_tx_cmd (SFTX);
        return -1;
      }

      gone += CC1120_TXFIFO_THR;
      in_fifo -= CC1120_TXFIFO_THR;
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

  cc_tx_readReg(MODEM_STATUS0, res_fifo);
  ret = cc_tx_cmd (SFTX);
  return gone + in_fifo - 1;
}




uint8_t
cc_rx_readReg (uint16_t add, uint8_t *data)
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
  delayUS_DWT(10);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) temp_TxBuffer,
			   (uint8_t *) temp_RxBuffer, len, 5000);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
  delayUS_DWT(10);
  *data = temp_RxBuffer[len - 1];

  return temp_RxBuffer[0];
}

uint8_t
cc_rx_writeReg (uint16_t add, uint8_t data)
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
  HAL_Delay (1);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t *) aTxBuffer, (uint8_t *) aRxBuffer,
			   len, 5000);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}


int32_t
cc_rx_data(uint8_t *out, size_t len, size_t timeout_ms)
{
  HAL_StatusTypeDef ret;
  uint8_t frame_len;
  uint8_t received = 0;
  uint32_t start_tick;
  uint8_t timeout = 1;
  uint8_t rx_n_bytes;
  uint8_t tmp[2];

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
    return STATUS_TIMEOUT;
  }

  /*
   * Time to extract the frame length,. This is indicated by the first byte
   * after the SYNC word
   */
  do {
      cc_rx_readReg(NUM_RXBYTES, &rx_n_bytes);
  } while (rx_n_bytes < 1);

  ret = cc_rx_spi_read_fifo(tmp, 2);
  frame_len = tmp[1];
  if(ret) {
    cc_rx_cmd(SFRX);
    return STATUS_NO_DATA;
  }

  /*
   * Now that we have the frame length check if the FIFO should be dequeued
   * multiple times or not
   */
  while(frame_len - received > CC1120_RXFIFO_THR) {
    /* Wait for the RX FIFO above threshold interrupt */
    start_tick = HAL_GetTick ();
    timeout = 1;
    while (HAL_GetTick () - start_tick < timeout_ms) {
      if (rx_thr_flag) {
	timeout = 0;
	break;
      }
    }
    if (timeout) {
      cc_rx_cmd (SFRX);
      return STATUS_TIMEOUT;
    }

    /* We can now dequeue CC1120_TXFIFO_THR bytes */
    ret = cc_rx_spi_read_fifo(out + received, CC1120_BYTES_IN_RX_FIF0);
    if(ret) {
      cc_rx_cmd(SFRX);
      return STATUS_NO_DATA;
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
    return STATUS_TIMEOUT;
  }

  /* Now dequeue the remaining bytes in the FIFO if any left*/
  if (frame_len - received) {
    ret = cc_rx_spi_read_fifo (out + received, frame_len - received);
    if (ret) {
      cc_rx_cmd (SFRX);
      return STATUS_NO_DATA;
    }
  }
  cc_rx_cmd(SFRX);
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
  memset(rx_spi_tx_buf, 0, CC1120_RX_FIFO_SIZE);
  rx_spi_tx_buf[0] = BURST_RXFIFO;
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_RESET);
  ret = HAL_SPI_TransmitReceive (&hspi2, rx_spi_tx_buf, out, len,
				 COMMS_DEFAULT_TIMEOUT_MS);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15, GPIO_PIN_SET);
  return ret;
}

/*
 * FIXME: Why global?
 */
uint8_t rx_size;

uint8_t cc_RX_DATA(uint8_t *data, uint8_t *size, uint8_t *rec_data) {

    /*Set tx packet len*/
    //cc_tx_writeReg(PKT_LEN, size-1);

    data[0] = 0xFF;

    uint8_t res2[6] = {0, 0, 0, 0, 0, 0}, res_fifo[6] = {0, 0, 0, 0, 0, 0};
      
      res2[0] = cc_rx_readReg(RXFIRST, &res_fifo[0]);
      res2[1] = cc_rx_readReg(RXLAST, &res_fifo[1]);
      res2[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifo[2]);
      res2[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifo[3]);

      sprintf((char*)uart_temp, "x1: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
      //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
      HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
      
    rx_size = 0;
    uint8_t res = 0;
    res = cc_rx_readReg(NUM_RXBYTES, &res_fifo[2]);
    
    rx_size = res_fifo[2];

    sprintf((char*)uart_temp, "NUM_RXBYTES %x,%d %x\n", rx_size, rx_size, res);
    //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
    HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    res2[3] = cc_rx_readReg(MODEM_STATUS1, &res_fifo[3]);

    sprintf((char*)uart_temp, "MODEM STATUS %x\n", res_fifo[3]);
    //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
    HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    if(rx_size > 0) {
 
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET); 	//chip select LOw
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)data, (uint8_t *)rec_data, rx_size, 5000); //send and receive 3 bytes
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
        
              res2[0] = cc_rx_readReg(RXFIRST, &res_fifo[0]);
      res2[1] = cc_rx_readReg(RXLAST, &res_fifo[1]);
      res2[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifo[2]);
      res2[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifo[3]);

      sprintf((char*)uart_temp, "x2: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
      //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
      HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
      
        //rec_data[rx_size] = 0x7e;
        //rx_size++;
        *size = rx_size;

        sprintf((char*)uart_temp, "Read data %x, %s\nAP %x %x\n", rx_size, rec_data, rec_data[rx_size +2], rec_data[rx_size +3]);
        //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
        HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
        
        uint16_t cnt = 0;
        for(uint16_t i = 0; i < 500*8 ; i++){ AX_aRxBuffer[i] = 0; }
        for(size_t i = 2; i < rx_size; i++){
          
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 7);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 6);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 5);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 4);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 3);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 2);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i] >> 1);
          AX_aRxBuffer[cnt++] = 0x01 & (rec_data[i]);

        }
        *size = cnt;
        //if(rx_size > 1) { ax25_rx_test(); }
        
        cc_rx_cmd(SFRX);
        sprintf((char*)uart_temp, "RX RESET\n");
        //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
        HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
        HAL_Delay(100);
        cc_rx_cmd(SRX);
        HAL_Delay(100);
    }
    
    if((0x07 & (res >> 4)) == 6) {
        cc_rx_cmd(SFRX);
        sprintf((char*)uart_temp, "RX FIFO ERROR\n");
        //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
        HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
        HAL_Delay(100);
        cc_tx_cmd(SRX);
        HAL_Delay(100);
      }
    
    for(size_t i = 0; i < rx_size; i++){
      sprintf(uart_temp, "%02x\n", rec_data[i]);
      //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);      
      HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    }
    
    sprintf(uart_temp, "\n");
    //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
    HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    for(size_t i = 0; i < 500; i++){ rec_data[i] = 0; } 

      //uint8_t res_fifo[6];
  
      res2[0] = cc_rx_readReg(RXFIRST, &res_fifo[0]);
      res2[1] = cc_rx_readReg(RXLAST, &res_fifo[1]);
      res2[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifo[2]);
      res2[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifo[3]);

      sprintf((char*)uart_temp, "2: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
      //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
      HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
      //cc_tx_cmd(RTX);   	   //transmit command

      res2[0] = cc_rx_readReg(RXFIRST, &res_fifo[0]);
      res2[1] = cc_rx_readReg(RXLAST, &res_fifo[1]);
      res2[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifo[2]);
      res2[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifo[3]);

      sprintf((char*)uart_temp, "3: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
      //HAL_UART_Transmit(&huart2, uart_temp, strlen(uart_temp), 10000);
      HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    HAL_Delay(100);
    
    //cc_tx_cmd(SRES);
    
    //cc_tx_cmd(SFTX);	   //flash Tx fifo. SFTX when in IDLE
    
    HAL_Delay(50);
    return rec_data[0];
}

uint8_t
cc_rx_cmd (uint8_t CMDStrobe)
{

  uint8_t aTxBuffer[1];
  uint8_t aRxBuffer[1];

  aTxBuffer[0] = CMDStrobe;

  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive (&hspi2, (uint8_t*) aTxBuffer, (uint8_t *) aRxBuffer,
			   1, 5000);
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

  return aRxBuffer[0];
}

