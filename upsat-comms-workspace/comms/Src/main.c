/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <cc_tx_init.h>
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "cc_commands.h"
#include "cc_definitions.h"
#include "ax25.h"
#include "cc112x_spi.h"
#include "ax25.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[500];
uint8_t aRxBuffer[500];

uint8_t pkt_size;

  uint8_t addr_buf[AX25_MAX_ADDR_LEN];
  uint8_t prepare_buf[2*AX25_MAX_FRAME_LEN];
  uint8_t encode_buf[2*AX25_MAX_FRAME_LEN*8];
  uint8_t decode_buf[2*AX25_MAX_FRAME_LEN];

  uint8_t payload[100] = "HELLO WORLD FROM STM";
  uint8_t src_addr[100] = "ABCD";
  uint8_t dest_addr[100] = "ABCD";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ax25_test();

/*static uint8_t SPI2WriteSingledata(uint8_t swAdrr, uint8_t swData);
static uint8_t SPI2CommandStrobe(uint8_t CMDStrobe);
static void manualCalibration(void);
static uint8_t SPI2ReadExtended(uint8_t srExtndAdrr, uint8_t srData);

static uint8_t SPI1WriteSingledata(uint8_t swAdrr, uint8_t swData);
static uint8_t SPI1ReadSingle(uint8_t srAdrr);
static uint8_t SPI1ReadExtended(uint8_t srExtndAdrr, uint8_t srData);
static uint8_t SPI1CommandStrobe(uint8_t CMDStrobe);

static uint8_t SPI1WriteManydata(uint8_t *data, uint8_t size);

extern void cc_Tx_INIT();*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t uart_temp[100];
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, RESETN_TX_Pin, GPIO_PIN_SET);//PA10
  HAL_GPIO_WritePin(PA_CNTRL_GPIO_Port, PA_CNTRL_Pin, GPIO_PIN_SET); //POWER AMP CONTROL
  	  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);//csn1


  	  	uint8_t uart_temp[100];
  	  HAL_Delay(100);
  	  uint8_t cc_id;

  	  //sprintf((char*)uart_temp, "HELLO %d\n", cc_id);
  	  cc_tx_readReg(0x2f8F, &cc_id);
  	  //sprintf((char*)uart_temp, "HELLO %d\n", cc_id);
  	  //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

  	  registerConfig();

  	  HAL_Delay(100);

  	  cc_tx_readReg(0x2f8F, &cc_id);
  	 // sprintf((char*)uart_temp, "INIT finished %d\n", cc_id);
  	  //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

  	 manualCalibration();

  	 cc_tx_readReg(0x2f8F, &cc_id);
  	 // sprintf((char*)uart_temp, "calibration finished %d\n", cc_id);
  	  //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

  	  //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);


  	  HAL_Delay(100);
  //SPI1ReadExtended(0x8F, 0x00);
  
 /* cc_Tx_INIT();
  
  HAL_Delay(100);
  
  manualCalibration();

  HAL_Delay(100);*/

  //SPI1WriteSingledata(0x7F, 0x48); //should expext 112 bytes (110 and 2) needs buffer
  	  aTxBuffer[2] = 0x7e;
  	  aTxBuffer[3] = 0x41;
  	  aTxBuffer[4] = 0x21;
  	  aTxBuffer[5] = 0x61;
  	  aTxBuffer[6] = 0x11;
  	  aTxBuffer[7] = 0x02;
  	  aTxBuffer[8] = 0x02;
  	  aTxBuffer[9] = 0x06;
  	  aTxBuffer[10] = 0x41;
  	  aTxBuffer[11] = 0x21;
  	  aTxBuffer[12] = 0x61;
  	  aTxBuffer[13] = 0x11;
  	  aTxBuffer[14] = 0x02;
  	  aTxBuffer[15] = 0x02;
  	  aTxBuffer[16] = 0x86;
  	  aTxBuffer[17] = 0xc0;
  	  aTxBuffer[18] = 0x0f;
  	  aTxBuffer[19] = 0x12;
  	  aTxBuffer[20] = 0xa2;
  	  aTxBuffer[21] = 0x32;
  	  aTxBuffer[22] = 0x32;
  	  aTxBuffer[23] = 0xf2;
  	  aTxBuffer[24] = 0x04;
  	  aTxBuffer[25] = 0xea;
  	  aTxBuffer[26] = 0xf2;
  	  aTxBuffer[27] = 0x4a;
  	  aTxBuffer[28] = 0x32;
  	  aTxBuffer[29] = 0x22;
  	  aTxBuffer[30] = 0x04;
  	  aTxBuffer[31] = 0x62;
  	  aTxBuffer[32] = 0x4a;
  	  aTxBuffer[33] = 0xf2;
  	  aTxBuffer[34] = 0xb2;
  	  aTxBuffer[35] = 0x04;
  	  aTxBuffer[36] = 0xca;
  	  aTxBuffer[37] = 0x0a;
  	  aTxBuffer[38] = 0x82;
  	  aTxBuffer[39] = 0xc2;
  	  aTxBuffer[40] = 0xa2;
  	  aTxBuffer[41] = 0x9b;
  	  aTxBuffer[42] = 0x9a;
  	  aTxBuffer[43] = 0x7e;

  	  //cc_TX_DATA(aTxBuffer, 42, aRxBuffer);
  	  uint8_t res;
  	  uint8_t res2[6];
  	  uint8_t res_fifo[6];
  	  uint8_t loop = 0;

/*
  //for(uint8_t i = 1; i < 120; i++) { aTxBuffer[i] = 0x48; }
  //SPI1WriteManydata(aTxBuffer, 120);
  SPI1WriteManydata(aTxBuffer, 39);
  
  SPI1CommandStrobe(STX);   	   //transmit command
  
  SPI1CommandStrobe(SRES);         //reset devise. After reset device should be in IDLE mabe no need to

  SPI1CommandStrobe(SFTX);	   //flash Tx fifo. SFTX when in IDLE
*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	        HAL_Delay(100);

	        res = cc_tx_readReg(0x2f8F, &cc_id);
	       // sprintf((char*)uart_temp, "HELLO %x, state: %x\n", cc_id, res);
	       // HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	        //cc_Tx_INIT();

	        HAL_Delay(100);

	        //manualCalibration();

	        HAL_Delay(100);

	        res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]); // pointer at first
	        res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);  // pointer at last
	        res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);  // number of bytes
	        res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]); //number of free bytes

	        //sprintf((char*)uart_temp, "1: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
	        //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	        uint8_t tbuf[255];
	        //for(uint8_t i = 0; i < 255; i++) { tbuf[i] = 0x0f; }
	        //cc_TX_DATA(aTxBuffer, 42, aRxBuffer);

	        //snprintf(payload, 100, "HELLO WORLD FROM STM %d\n", loop);
	        ax25_test();
	        cc_TX_DATA(aTxBuffer, pkt_size, aRxBuffer);
	        loop++;

	        res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]);
	        res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);
	        res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);
	        res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]);

	       // sprintf((char*)uart_temp, "4: FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
	        //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
	        //HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* UART5 init function */
void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart5);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PA_CNTRL_GPIO_Port, PA_CNTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESETN_RX_GPIO_Port, RESETN_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SPI2_RX_GPIO_Port, CS_SPI2_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESETN_TX_Pin|CS_SPI1_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA_CNTRL_Pin RESETN_TX_Pin CS_SPI1_TX_Pin */
  GPIO_InitStruct.Pin = PA_CNTRL_Pin|RESETN_TX_Pin|CS_SPI1_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RESETN_RX_Pin */
  GPIO_InitStruct.Pin = RESETN_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESETN_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SPI2_RX_Pin */
  GPIO_InitStruct.Pin = CS_SPI2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SPI2_RX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ax25_test() {


  size_t addr_len;
  size_t prepare_len;
  size_t encode_len;
  size_t decode_len;
  ax25_encode_status_t encode_status;
  ax25_decode_status_t decode_status;




  addr_len = ax25_create_addr_field(addr_buf, dest_addr, 0, src_addr, 0);
  if(addr_len < AX25_MIN_ADDR_LEN) {
    //throw std::runtime_error("Wrong address");
  }

  prepare_len = ax25_prepare_frame(prepare_buf, payload,
				   strnlen(payload, 100), AX25_UI_FRAME, addr_buf,
				   addr_len, 0x03, 1);

  encode_status = ax25_bit_stuffing(encode_buf, &encode_len, prepare_buf,
				    prepare_len);

  //printf(" bit stuffing %d\n", encode_len);
  //for(size_t i = 0; i < encode_len; i++){
  //  printf(" 0x%02x\n", encode_buf[i]);
  //}

  uint8_t cnt = 2;
  //printf(" bit stuffing in byte\n");
  for(size_t i = 0; i < encode_len; i+=8){
    uint8_t temp = (encode_buf[i] << 7) | (encode_buf[i+1] << 6) | (encode_buf[i+2] << 5) | (encode_buf[i+3] << 4) | (encode_buf[i+4] << 3) | (encode_buf[i+5] << 2) | (encode_buf[i+6] << 1) | (encode_buf[i+7]);
    aTxBuffer[cnt] = temp;
    cnt++;

    //printf(" %02x", temp);

  }

  pkt_size = cnt;
  //cnt = 1;
  //printf(" bit stuffing in byte\n");
  //for(size_t i = 0; i < encode_len; i+=8){
  //  uint8_t temp = (encode_buf[i] << 7) | (encode_buf[i+1] << 6) | (encode_buf[i+2] << 5) | (encode_buf[i+3] << 4) | (encode_buf[i+4] << 3) | (encode_buf[i+5] << 2) | (encode_buf[i+6] << 1) | (encode_buf[i+7]);

  //  printf("aTxBuffer[%d] = 0x%02x;\n",cnt, temp);
  //  cnt++;
  //}

  //printf("len %d", cnt);

  if(encode_status != AX25_ENC_OK) {
    //throw std::runtime_error("Encoding failed");
    //sprintf((char*)uart_temp, "AX Encoding failed\n");
    //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
  }
  else{
    //std::cout << "Successful encoding" << std::endl;
    //sprintf((char*)uart_temp, "AX Successful encoding\n");
    //HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
  }}
/*static void manualCalibration(void) {



uint8_t orgnlfscal2;

uint8_t Resultsvcdac_strtH0=0X00;
uint8_t Resultsvcdac_strtH1=0X00;
uint8_t Resultsvcdac_strtH2=0X00;

uint8_t Resultsvcdac_strtM0=0X00;
uint8_t Resultsvcdac_strtM1=0X00;
uint8_t Resultsvcdac_strtM2=0X00;

uint8_t writeByte3;

1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
writeByte3 = 0x00;

halRfWriteReg(FS_VCO2,writeByte3);



2) Start with high VCDAC (original VCDAC_START + 2): IT IS A READ OP

orgnlfscal2 = SPI1ReadExtended((uint8_t)(0x00FF & FS_CAL2), 0x00);

writeByte3 = orgnlfscal2 + 0x02;


halRfWriteReg(FS_CAL2,writeByte3);

3) Calibrate and wait for calibration to be done
	         (radio back in IDLE state)

SPI1CommandStrobe(SCAL);
//delay10ms(1);
HAL_Delay(1);
do {
	marcstate=SPI2ReadExtended(CC112X_MARCSTATE, 0x00);

} while (marcstate != 0x41);

 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
  	   	   	   	   high VCDAC_START value




Resultsvcdac_strtH0=	SPI1ReadExtended((uint8_t)(0x00FF & FS_CAL2), 0x00);
Resultsvcdac_strtH1=	SPI1ReadExtended((uint8_t)(0x00FF & FS_VCO4), 0x00);
Resultsvcdac_strtH2=	SPI1ReadExtended((uint8_t)(0x00FF & FS_CHP), 0x00);
 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)

writeByte3 = 0x00;

halRfWriteReg(FS_VCO2,writeByte3);


6) Set VCO cap-array to 0 (FS_VCO2 = 0x00)

writeByte3 = orgnlfscal2;

halRfWriteReg(FS_CAL2,writeByte3);

  7) Calibrate and wait for calibration to be done
(radio back in IDLE state)

SPI1CommandStrobe(SCAL);
//delay10ms(1);
HAL_Delay(1);
do {
	marcstate=SPI2ReadExtended(CC112X_MARCSTATE, 0x00);

} while (marcstate != 0x41);

8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
       with mid VCDAC_START value

Resultsvcdac_strtM0=	SPI1ReadExtended((uint8_t)(0x00FF & FS_CAL2), 0x00);
Resultsvcdac_strtM1=	SPI1ReadExtended((uint8_t)(0x00FF & FS_VCO4), 0x00);
Resultsvcdac_strtM2=	SPI1ReadExtended((uint8_t)(0x00FF & FS_CHP), 0x00);

9) Write back highest FS_VCO2 and corresponding FS_VCO
and FS_CHP result

if (Resultsvcdac_strtH0 >
Resultsvcdac_strtM0) {

	writeByte3 = Resultsvcdac_strtH0;
    halRfWriteReg(FS_VCO2,writeByte3);
    writeByte3 = Resultsvcdac_strtH1;
    halRfWriteReg(FS_VCO4,writeByte3);
    writeByte3 = Resultsvcdac_strtH2;
    halRfWriteReg(FS_CHP,writeByte3);
} else {
	writeByte3 = Resultsvcdac_strtM0;
	halRfWriteReg(FS_VCO2,writeByte3);
	writeByte3 = Resultsvcdac_strtM1;
	halRfWriteReg(FS_VCO4,writeByte3);
	writeByte3 = Resultsvcdac_strtM2;
	halRfWriteReg(FS_CHP,writeByte3);
}

}*/

/*static uint8_t SPI1CommandStrobe(uint8_t CMDStrobe)
{


//uint8_t aTxBuffer[1];
//uint8_t aRxBuffer[1];


aTxBuffer[0]= CMDStrobe;    		//		extended address
//      send dummy so that i can read data
aTxBuffer[1]=0x00;
aTxBuffer[2]=0x00;
aTxBuffer[3]=0x00;

aRxBuffer[0]=0x00;
aRxBuffer[1]=0x00;
aRxBuffer[2]=0x00;
aRxBuffer[3]=0x00;      //      one more for slot for contingency


//      one more for slot for contingency

HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
//delay10ms (15);
HAL_Delay(1);
HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 1, 5000); //send and receive 1 bytes
//delay10ms (15);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

//delay10ms (50);
HAL_Delay(50);
return aRxBuffer[0];   //if need be please change this part to return the whole buffer

}

static uint8_t SPI1ReadExtended(uint8_t srExtndAdrr, uint8_t srData)
{

	//uint8_t aTxBuffer[3];
	//uint8_t aRxBuffer[4];

aTxBuffer[0]=0xaf;
aTxBuffer[1]=srExtndAdrr;    		//		extended address
aTxBuffer[2]=srData;         //      send dummy so that i can read data


aRxBuffer[0]=0x00;
aRxBuffer[1]=0x00;
aRxBuffer[2]=0x00;
aRxBuffer[3]=0x00;      //      one more for slot for contingency





HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
//delay10ms (1);
HAL_Delay(1);
HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 3, 10); //send and receive 3 bytes
//delay10ms (1);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
HAL_Delay(50);
return aRxBuffer[2];  //if need be please change this part to return the whole buffer

}*/

static uint8_t SPI1ReadSingle(uint8_t srAdrr)
{
srAdrr= srAdrr+ 0x80;

//uint8_t aTxBuffer[2];
//uint8_t aRxBuffer[3];


aTxBuffer[0]=srAdrr;    		//		extended address
aTxBuffer[1]=0x00;         //      send dummy so that i can read data


aRxBuffer[0]=0x00;
aRxBuffer[1]=0x00;
aRxBuffer[2]=0x00;
//      one more for slot for contingency





HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);  	//chip select LOw
//delay10ms (15);
HAL_Delay(1);
HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, 2, 5000); //send and receive 3 bytes
//delay10ms (15);
HAL_Delay(1);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

//delay10ms (50);
HAL_Delay(50);
return aRxBuffer[1];   //if need be please change this part to return the whole buffer

}

static uint8_t SPI1WriteManydata(uint8_t *data, uint8_t size)
{

HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET); 	//chip select LOw
//delay10ms (15);
HAL_Delay(1);
HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)data, (uint8_t *)aRxBuffer, size, 5000); //send and receive 3 bytes
//delay10ms (15);
HAL_Delay(1);
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);

//delay10ms (50);
HAL_Delay(50);
return aRxBuffer[1];   //if need be please change this part to return the whole buffer

}




/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
