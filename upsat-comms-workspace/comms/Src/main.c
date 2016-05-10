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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "cc_commands.h"
#include "cc_definitions.h"
#include "ax25.h"
#include "cc112x_spi.h"
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
uint8_t AX_aRxBuffer[500*8];
uint8_t aRxBuffer[500];
uint8_t pkt_size;

uint8_t addr_buf[AX25_MAX_ADDR_LEN];
uint8_t prepare_buf[2*AX25_MAX_FRAME_LEN];
uint8_t encode_buf[2*AX25_MAX_FRAME_LEN*8];
uint8_t decode_buf[2*AX25_MAX_FRAME_LEN];

uint8_t payload[100] = "HELLO WORLD FROM UPSAT";
uint8_t src_addr[100] = "ABCD";
uint8_t dest_addr[100] = "ABCD";

	uint8_t res;
	uint8_t resRX;
	uint8_t res2[6];
	uint8_t res2RX[6];
	uint8_t res_fifo[6];
	uint8_t res_fifoRX[6];
	uint8_t loop = 0;
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
void ax25_tx_test();
void ax25_rx_test();

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

	/* USER CODE BEGIN 2  */

	/* TX Pins RESETN_TX, PA_CTRL_PIN, CSN1 */
	HAL_GPIO_WritePin(GPIOA, RESETN_TX_Pin, GPIO_PIN_SET);//PA10
	HAL_GPIO_WritePin(PA_CNTRL_GPIO_Port, PA_CNTRL_Pin, GPIO_PIN_SET); //POWER AMP CONTROL
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);//csn1

	/* RX Pins RESETN_RX, CSN2 */

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);//PIN36 2RESETN
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);//PIN36 2CSN


	uint8_t uart_temp[100];

	HAL_Delay(100);

	uint8_t cc_id_tx;
	uint8_t cc_id_rx;

	//fetch tx id
	cc_tx_readReg(0x2f8F, &cc_id_tx);
	sprintf((char*)uart_temp, "1st hello from TX %d\n", cc_id_tx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	//fetch rx id
	cc_rx_readReg(0x2f8F, &cc_id_rx);
	sprintf((char*)uart_temp, "1st hello from RX %d\n", cc_id_rx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	//Configure TX CC1120
	tx_registerConfig();


	HAL_Delay(10);
	cc_tx_readReg(0x2f8F, &cc_id_tx);
	sprintf((char*)uart_temp, "TX CC1120 %d configured\n", cc_id_tx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	//Configure RX CC1120
	rx_registerConfig();


	HAL_Delay(10);
	cc_rx_readReg(0x2f8F, &cc_id_rx);
	sprintf((char*)uart_temp, "RX CC1120 %d configured\n", cc_id_rx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);


	//Calibrate TX
	tx_manualCalibration();

	cc_tx_readReg(0x2f8F, &cc_id_tx);
	sprintf((char*)uart_temp, "TX CC1120 %d calibrated\n", cc_id_tx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	//Calibrate RX
	rx_manualCalibration();

	cc_rx_readReg(0x2f8F, &cc_id_tx);
	sprintf((char*)uart_temp, "RX CC1120 %d calibrated\n", cc_id_tx);
	HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

	HAL_Delay(100);


	cc_rx_cmd(SRX);

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		HAL_Delay(100);

		res = cc_tx_readReg(0x2f8F, &cc_id_tx);
		sprintf((char*)uart_temp, "TX %x, state: %x\n", cc_id_tx, res);
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

		HAL_Delay(100);
		//resRX = cc_rx_readReg(0x2f8F, &cc_id_rx);
		//sprintf((char*)uart_temp, "RX %x, state: %x\n", cc_id_rx, resRX);
		//HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

		HAL_Delay(300);


		/*--------------TX------------*/

		res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]); // pointer at first
		res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);  // pointer at last
		res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);  // number of bytes
		res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]); //number of free bytes

		sprintf((char*)uart_temp, "TX FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

		//uint8_t tbuf[255];

		snprintf(payload, 100, "HELLO WORLD FROM STM %d\n", loop);
		ax25_test();

		cc_TX_DATA(aTxBuffer, pkt_size, aRxBuffer);
		loop++;

		res2[0] = cc_tx_readReg(TXFIRST, &res_fifo[0]);
		res2[1] = cc_tx_readReg(TXLAST, &res_fifo[1]);
		res2[2] = cc_tx_readReg(NUM_TXBYTES, &res_fifo[2]);
		res2[3] = cc_tx_readReg(FIFO_NUM_TXBYTES, &res_fifo[3]);

		sprintf((char*)uart_temp, "TX FIFO after %x,%x %x,%x %x,%x %x,%x\n", res_fifo[0], res2[0], res_fifo[1], res2[1], res_fifo[2], res2[2], res_fifo[3], res2[3]);
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
		HAL_Delay(10);


		HAL_Delay(500);


		/*--------------RX------------*/

		res2RX[0] = cc_rx_readReg(RXFIRST, &res_fifoRX[0]); // pointer at first
		res2RX[1] = cc_rx_readReg(RXLAST, &res_fifoRX[1]);  // pointer at last
		res2RX[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifoRX[2]);  // number of bytes
		res2RX[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifoRX[3]); //number of free bytes

		//uint8_t tbuf[255];

		cc_RX_DATA(aTxBuffer, &pkt_size, aRxBuffer);

		res2RX[0] = cc_rx_readReg(RXFIRST, &res_fifoRX[0]);
		res2RX[1] = cc_rx_readReg(RXLAST, &res_fifoRX[1]);
		res2RX[2] = cc_rx_readReg(NUM_RXBYTES, &res_fifoRX[2]);
		res2RX[3] = cc_rx_readReg(FIFO_NUM_RXBYTES, &res_fifoRX[3]);

		sprintf((char*)uart_temp, "RX FIFO %x,%x %x,%x %x,%x %x,%x\n", res_fifoRX[0], res2RX[0], res_fifoRX[1], res2RX[1], res_fifoRX[2], res2RX[2], res_fifoRX[3], res2RX[3]);
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

		sprintf(uart_temp, "\n");
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);

		HAL_Delay(100);


	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)//
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
	uint8_t AX_aRxBuffer[500*8];
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

	//uint8_t AX_aRxBuffer[500*8];

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
	huart5.Init.BaudRate = 9600;
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

void ax25_tx_test() {


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
	uint8_t AX_aRxBuffer[500*8];
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



void ax25_rx_test() {


	size_t addr_len;
	size_t prepare_len;
	size_t encode_len;
	size_t decode_len;
	ax25_encode_status_t encode_status;
	ax25_decode_status_t decode_status;


	decode_status = ax25_decode(decode_buf, &decode_len, AX_aRxBuffer, pkt_size);

	if(decode_status != AX25_DEC_OK) {
		sprintf((char*)uart_temp, "AX Decoded failed\n");
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
	}
	else{
		//std::cout << "Successful encoding" << std::endl;
		sprintf((char*)uart_temp, "AX Successful Decoded\n");
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
	}

	if(decode_len > 0) {
		//printf("%c", decode_buf[i]);
		decode_buf[decode_len] = 0;

		sprintf((char*)uart_temp, "AX: %s\n", decode_buf);
		HAL_UART_Transmit(&huart5, uart_temp, strlen(uart_temp), 10000);
	}
	//std::cout << std::endl;

	//std::cout << "Hex Format" << std::endl;
	//for(size_t i = 0; i < decode_len; i++){
	//  printf("0x%02x ", decode_buf[i]);
	//}
	//std::cout << std::endl;

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
