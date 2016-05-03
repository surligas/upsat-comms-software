#include "cc112x_spi.h"
//#include "stm32f4xx_hal_conf.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart5;
extern uint8_t uart_temp[100];

uint8_t cc_tx_readReg(uint16_t add, uint8_t *data) {

    uint8_t temp_TxBuffer[4];
    uint8_t temp_RxBuffer[4] = {0, 0, 0, 0};
    uint8_t len = 0;

    if(add >= CC_EXT_ADD) {
        len = 3;

        temp_TxBuffer[0] = 0xAF;
        temp_TxBuffer[1] = (uint8_t)(0x00FF & add);	 //extended address
        temp_TxBuffer[2] = 0;                      //send dummy so that i can read data
    } else {
        len = 2;

        add |= 0x0080; /*bit masked for read function*/
        temp_TxBuffer[0] = (uint8_t)(0x00FF & add);    		//		extended address
        temp_TxBuffer[1] = 0;         //      send dummy so that i can read data
    }

    

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET); 	//chip select LOw
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)temp_TxBuffer, (uint8_t *)temp_RxBuffer, len, 5000); //send and receive 3 bytes
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
        
    HAL_Delay(50);
    *data = temp_RxBuffer[len - 1];

    uint8_t uart_temp[100];
   //sprintf((char*)uart_temp, "read Reg %d, %d\n", temp_RxBuffer[0], *data);
    //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

    return temp_RxBuffer[0];  //if need be please change this part to return the whole buffer
}

uint8_t cc_tx_writeReg(uint16_t add, uint8_t data) {

    uint8_t aTxBuffer[4];
    uint8_t aRxBuffer[4] = {0, 0, 0, 0};
    uint8_t len = 0;

    if(add >= CC_EXT_ADD) {
        len = 3;

        aTxBuffer[0] = 0x2F;
        aTxBuffer[1] = (uint8_t)(0x00FF & add);	 //extended address
        aTxBuffer[2] = data;                      //send dummy so that i can read data
    } else {
        len = 2;

        aTxBuffer[0] = (uint8_t)(0x00FF & add);    		//		extended address
        aTxBuffer[1] = data;         //      send dummy so that i can read data
    }

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET); 	//chip select LOw
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)aTxBuffer, (uint8_t *)aRxBuffer, len, 5000); //send and receive 3 bytes
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
    
    // uint8_t uart_temp[100];
   // sprintf((char*)uart_temp, "write Reg %x\n", aRxBuffer[0]);
   // HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    //HAL_Delay(50);
    return aRxBuffer[0];  //if need be please change this part to return the whole buffer
}

uint8_t cc_TX_DATA(uint8_t *data, uint8_t size, uint8_t *rec_data) {

    /*Set tx packet len*/
    //cc_tx_writeReg(PKT_LEN, size-1);

    data[0] = 0x7F;
    data[1] = size;
    size++;
    size++;
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET); 	//chip select LOw
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)data, (uint8_t *)rec_data, size, 5000); //send and receive 3 bytes
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

      uint8_t res2[6];
      uint8_t res_fifo[6];
  
      res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]);
      res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);
      res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);
      res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]);

     // sprintf((char*)uart_temp, "2: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
     // HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    cc_tx_cmd(STX);   	   //transmit command

      res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]);
      res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);
      res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);
      res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]);

      //sprintf((char*)uart_temp, "3: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
      //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
    
    HAL_Delay(100);
    
    //cc_tx_cmd(SRES);
    
    //cc_tx_cmd(SFTX);	   //flash Tx fifo. SFTX when in IDLE
    
    HAL_Delay(50);
    return rec_data[0];
}

uint8_t cc_tx_cmd(uint8_t CMDStrobe) {

    uint8_t aTxBuffer[1];
    uint8_t aRxBuffer[1];

    aTxBuffer[0]= CMDStrobe;

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //send and receive 1 bytes
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

    HAL_Delay(50);
    return aRxBuffer[0];   //if need be please change this part to return the whole buffer
}

//manos functions
//static uint8_t SPI1CommandStrobe(uint8_t CMDStrobe)
//{
//
//
////uint8_t aTxBuffer[1];
////uint8_t aRxBuffer[1];
//
//
//aTxBuffer[0]= CMDStrobe;    		//		extended address
////      send dummy so that i can read data
//aTxBuffer[1]=0x00;
//aTxBuffer[2]=0x00;
//aTxBuffer[3]=0x00;
//
//aRxBuffer[0]=0x00;
//aRxBuffer[1]=0x00;
//aRxBuffer[2]=0x00;
//aRxBuffer[3]=0x00;      //      one more for slot for contingency
//
//
////      one more for slot for contingency
//
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
////delay10ms (15);
//HAL_Delay(1);
//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //send and receive 1 bytes
////delay10ms (15);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
//
////delay10ms (50);
//HAL_Delay(50);
//return aRxBuffer[0];   //if need be please change this part to return the whole buffer
//
//}

//static uint8_t SPI1WriteSingledata(uint8_t swAdrr, uint8_t swData)
//{
//
//
//aTxBuffer[0]=swAdrr;    		//		extended address
//aTxBuffer[1]=swData;         //      send dummy so that i can read data
//
//
//aRxBuffer[0]=0x00;
//aRxBuffer[1]=0x00;
//aRxBuffer[2]=0x00;
////      one more for slot for contingency
//
//
////ME KANE DUO BUFFER ME KATUSTERHSH
//
//
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET); 	//chip select LOw
////delay10ms (15);
//HAL_Delay(1);
//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 2, 5000); //send and receive 3 bytes
////delay10ms (15);
////HAL_Delay(1);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
//
////delay10ms (50);
//
//HAL_Delay(50);
//
//return aRxBuffer[1];   //if need be please change this part to return the whole buffer
//
//}


//static uint8_t SPI1ReadExtended(uint8_t srExtndAdrr, uint8_t srData)
//{
//
//	//uint8_t aTxBuffer[3];
//	//uint8_t aRxBuffer[4];
//
//aTxBuffer[0]=0xaf;
//aTxBuffer[1]=srExtndAdrr;    		//		extended address
//aTxBuffer[2]=srData;         //      send dummy so that i can read data
//
//
//aRxBuffer[0]=0x00;
//aRxBuffer[1]=0x00;
//aRxBuffer[2]=0x00;
//aRxBuffer[3]=0x00;      //      one more for slot for contingency
//
//
//
//
//
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
////delay10ms (1);
//HAL_Delay(1);
//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 3, 10); //send and receive 3 bytes
////delay10ms (1);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
//HAL_Delay(50);
//return aRxBuffer[2];  //if need be please change this part to return the whole buffer
//
//}
//
//static uint8_t SPI1ReadSingle(uint8_t srAdrr)
//{
//srAdrr= srAdrr+ 0x80;
//
////uint8_t aTxBuffer[2];
////uint8_t aRxBuffer[3];
//
//
//aTxBuffer[0]=srAdrr;    		//		extended address
//aTxBuffer[1]=0x00;         //      send dummy so that i can read data
//
//
//aRxBuffer[0]=0x00;
//aRxBuffer[1]=0x00;
//aRxBuffer[2]=0x00;
////      one more for slot for contingency
//
//
//
//
//
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
////delay10ms (15);
//HAL_Delay(1);
//HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 2, 5000); //send and receive 3 bytes
////delay10ms (15);
//HAL_Delay(1);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
//
////delay10ms (50);
//HAL_Delay(50);
//return aRxBuffer[1];   //if need be please change this part to return the whole buffer
//
//}
