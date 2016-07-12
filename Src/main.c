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
#include "log.h"
#include "cc112x_spi.h"
#include "cc_tx_init.h"
#include "cc_rx_init.h"
#include <string.h>
#include "comms.h"
#include "pkt_pool.h"
#include "service_utilities.h"
#include "comms_manager.h"
#include "sensors.h"
#include "stats.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t aTxBuffer[500];
uint8_t aRxBuffer[500] = {0};
uint8_t spi_buffer[500] = {0};

uint8_t tx_buf[2 * AX25_MAX_FRAME_LEN];

uint8_t payload[AX25_MAX_FRAME_LEN] = "HELLO WORLD FROM UPSAT HELLO WORLD FROM UPSAT HELLO WORLD FROM UPSAT HELLO WORLD FROM UPSAT";

uint8_t res;
uint8_t resRX;
uint8_t res2[6];
uint8_t res2RX[6];
uint8_t res_fifo[6];
uint8_t res_fifoRX[6];
uint8_t loop = 0;
volatile uint8_t tx_thr_flag = 0;
volatile uint8_t tx_fin_flag = 0;
volatile uint8_t rx_sync_flag = 0;
volatile uint8_t rx_finished_flag = 0;
volatile uint8_t rx_thr_flag = 0;

uint8_t dbg_msg = 0;
uint8_t uart_temp[200];

comms_rf_stat_t comms_stats;
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
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static inline void
debug_ecss()
{
  for (size_t i = 0; i < 50; i++) {
    payload[0] = 8;
    payload[1] = 1;
    payload[2] = 192;
    payload[4] = 0;
    payload[5] = 0;
    payload[6] = 5;
    payload[7] = 16;
    payload[8] = 17;
    payload[9] = 2;
    payload[10] = 7;
    payload[11] = 0;
    payload[12] = 200;
    send_payload (payload, 13, COMMS_DEFAULT_TIMEOUT_MS);
    HAL_Delay(1);
  }
  HAL_Delay(1000);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  int32_t ret = 0;
  uint8_t rst_src;
  uint32_t cw_tick;
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
  MX_IWDG_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */

  /* TX Pins RESETN_TX, PA_CTRL_PIN, CSN1 */
  HAL_GPIO_WritePin (GPIOA, RESETN_TX_Pin, GPIO_PIN_SET); //PA10
  HAL_GPIO_WritePin (PA_CNTRL_GPIO_Port, PA_CNTRL_Pin, GPIO_PIN_SET); //POWER AMP CONTROL
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_15, GPIO_PIN_SET); //csn1

  /* RX Pins RESETN_RX, CSN2 */
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_1, GPIO_PIN_SET); //PIN36 2RESETN
  HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET); //PIN36 2CSN

  /*Must use this in order the compiler occupies flash sector 3*/
  flash_INIT();
  
  uint32_t add_read = flash_read_trasmit();

  HAL_Delay (4000);

  comms_init();
  LOG_UART_DBG(&huart5, "RF systems initialized and calibrated %d", add_read);

  HAL_Delay (100);

  pkt_pool_INIT ();

  uint16_t size = 0;

  event_crt_pkt_api (uart_temp, "COMMS STARTED", 666, 666, "", &size, SATR_OK);
  HAL_uart_tx (DBG_APP_ID, (uint8_t *) uart_temp, size);

  /*Uart inits*/
  HAL_UART_Receive_IT (&huart5, comms_data.obc_uart.uart_buf, UART_BUF_SIZE);

  /* Sent to OBC the reason of re-booting */
  HAL_reset_source(&rst_src);
  event_boot(rst_src, 0);

  /*Initialize the COMMS statistics mechanism */
  comms_rf_stats_init(&comms_stats);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  cw_tick = HAL_GetTick();
  while (1) {
    HAL_IWDG_Refresh(&hiwdg);
    import_pkt (OBC_APP_ID, &comms_data.obc_uart);
    export_pkt (OBC_APP_ID, &comms_data.obc_uart);

    /* Check if a CW beacon should be sent */
    if(HAL_GetTick() - cw_tick > __CW_INTERVAL_MS ) {

      /* Wait a little to finish the previous transmission if any */
      HAL_Delay(1000);
      ret = snprintf ((char *) payload, AX25_MAX_FRAME_LEN,
		      "UPSAT %d", loop);
      send_payload_cw(payload, ret);
      cw_tick = HAL_GetTick();
    }

    /*--------------TX------------*/
    if(dbg_msg == 1)
    {
      /* Send a dummy message towards earth */
      ret = snprintf ((char *) payload, AX25_MAX_FRAME_LEN,
          "HELLO WORLD FROM UPSAT 0 "
		      "HELLO WORLD FROM UPSAT 1"
		      "HELLO WORLD FROM UPSAT 2"
		      "HELLO WORLD FROM UPSAT 3"
		      "loop %d", loop);

      ret =  send_payload(payload, (size_t)ret, COMMS_DEFAULT_TIMEOUT_MS);
      HAL_Delay (50);
      if (ret > 0) {
        HAL_Delay (50);
        LOG_UART_DBG(&huart5, "Frame transmitted Loop %u Ret %d", loop, ret);
      }
      else {
        LOG_UART_DBG(&huart5, "Error %d at frame transmission", ret);
      }
      loop++;
      HAL_Delay (500);
    }

    /*--------------RX------------*/
#if 1
    memset(aRxBuffer, 0, 255);
    ret = recv_payload(aRxBuffer, 255, COMMS_DEFAULT_TIMEOUT_MS * 2 );
    if(ret < 0){
      LOG_UART_DBG(&huart5, "RX Failed %d\n", ret);
    }
    else{
      LOG_UART_DBG(&huart5, "RX OK %d\n", ret);
      HAL_Delay (50);
      LOG_UART_DBG(&huart5, "RX Msg OK: %s", aRxBuffer);
      ret = rx_ecss(aRxBuffer, ret);
      if(ret == SATR_OK){
	LOG_UART_DBG(&huart5, "ECSS Valid");
      }
      else{
	LOG_UART_DBG(&huart5, "Invalid ECSS. Error %d", ret);
      }
    }
    large_data_IDLE();
#endif

    /*Update the statistics of the COMMS */
    comms_rf_stats_update(&comms_stats);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Rank = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

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

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init(&hiwdg);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart3.Init.BaudRate = 9600;
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
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  /*Configure GPIO pin : CC_GPIO2_START_END_OF_PACKET_Pin */
  GPIO_InitStruct.Pin = CC_GPIO2_START_END_OF_PACKET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CC_GPIO2_START_END_OF_PACKET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SPI2_RX_Pin */
  GPIO_InitStruct.Pin = CS_SPI2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_SPI2_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CC_GPIO0_RXFIFO_THR_Pin */
  GPIO_InitStruct.Pin = CC_GPIO0_RXFIFO_THR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CC_GPIO0_RXFIFO_THR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PA_CNTRL_GPIO_Port, PA_CNTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESETN_RX_GPIO_Port, RESETN_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_SPI2_RX_GPIO_Port, CS_SPI2_RX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESETN_TX_Pin|CS_SPI1_TX_Pin, GPIO_PIN_SET);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void
HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  GPIO_PinState state;
  switch(GPIO_Pin){
    case GPIO_PIN_3:
      tx_thr_flag = 1;
      break;
    case GPIO_PIN_2:
      tx_fin_flag = 1;
      break;
    case CC_GPIO2_START_END_OF_PACKET_Pin:
      state = HAL_GPIO_ReadPin (CC_GPIO2_START_END_OF_PACKET_GPIO_Port,
				CC_GPIO2_START_END_OF_PACKET_Pin);
      if(state){
	rx_sync_flag = 1;
      }
      else {
	rx_finished_flag = 1;
      }
      break;
    case CC_GPIO0_RXFIFO_THR_Pin:
      rx_thr_flag = 1;
      break;
  }
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
