/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * Pinouts:
  * PA7 / A6 SPI MOSI
  * PA6 / A5 SPI MISO
  * PA5 / A4 SPI CLK
  * PA4 / A3 SPI NSS (GPIO HIGH ON START)
  * PA1 / A1 RESET
  * PA10/ D0 DIO0 Interrupt In.
  *
  * PA12/D2 For an attached speaker (dirty digital pulse only)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "lora_sx1276.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t inputReceived = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t inputBuffer[5];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int bufferReady = 0;
uint8_t rxCommandBuffer[7]; //6 char commands + eol
volatile uint8_t rxCommandBufferIndex = 0;
uint8_t rxChar;
volatile uint8_t serialRecieved = 0;

uint8_t loRaRXBuffer[64];
volatile uint8_t loRaRXBufferStatus = 0; //1 is good and ready to read.

/*
 * Commands:
 * GET T	(Get temperature)
 * GSYNC	(Ask GPS TO Sync time)
 * RES S	(Reset System)
 * RES G	(Reset GPS Module)
 * GET L	(Download log)
 * GET S1	(Get last sensor 1 value)
 * GET S2	(Get last sensor 2 value)
 * INF		(Current system health and status)
 * PING		(Lora comms test)
 * */

lora_sx1276 lora;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_USART_Transmit(&husart2, (uint8_t *)"Starting!\r\n", 11U, 100U);

  // SX1276 compatible module connected to SPI?, NSS pin connected to GPIO with label LORA_NSS
  uint8_t res = lora_init(&lora, &hspi1, SPI1_LORA_NSS_GPIO_Port, SPI1_LORA_NSS_Pin, LORA_BASE_FREQUENCY_US);

  if (res != LORA_OK) {
	DebugOutput("RFM95 init failed ", res);
  } else {
	DebugOutput("RFM95 init success ", res);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  sendIntroduction();
  sendCommandPrompt();

  HAL_UART_Receive_IT(&huart2, &rxChar, 1);

  lora_enable_interrupt_rx_done(&lora);
  lora_mode_receive_continuous(&lora); //if receive once, make sure this is set again

  while (1)
  {
	if (serialRecieved == 1){

		//filter allowed keys
		if (rxChar == 13 || rxChar == 127 || rxChar == 32 || rxChar == 63 || ( rxChar >= 48 && rxChar <= 57 ) || (rxChar >= 65 && rxChar <= 90) || (rxChar >= 97 && rxChar <= 122)){

			if (rxChar == 13){
				processCommand();
				rxCommandBufferIndex = 0;
				rxCommandBuffer[rxCommandBufferIndex] = '\0';
				//sendCommandPrompt();
			} else if (rxChar == 127){
				if (rxCommandBufferIndex > 0) {
					rxCommandBufferIndex--;
					rxCommandBuffer[rxCommandBufferIndex] = '\0';
					//rxChar = rxCommandBuffer[rxCommandBufferIndex];
					HAL_UART_Transmit(&huart2, &rxChar, 1U, 10U);
				}
			} else {
				if (rxCommandBufferIndex < 6){
					rxCommandBuffer[rxCommandBufferIndex] = rxChar;
					rxCommandBuffer[rxCommandBufferIndex+1] = '\0';
					rxCommandBufferIndex++;
					HAL_UART_Transmit(&huart2, &rxChar, 1U, 10U);
				}
			}

		}

		serialRecieved = 0;
	}

	// Receive buffer
	if (loRaRXBufferStatus == 1){
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2U, 10U);
		DebugOutput("===========> System ACK Received loRaRXBufferStatus: ", loRaRXBufferStatus);

		HAL_UART_Transmit(&huart2, (uint8_t*)loRaRXBuffer, strlen((char*)loRaRXBuffer), 10U);
		HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2U, 10U);

		memset(loRaRXBuffer, '\0', sizeof(loRaRXBuffer));
		sendCommandPrompt();
		loRaRXBufferStatus = 0; //clear: ready for interrupt to accept the next LoRa packet.
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_RESET_Pin|SPI1_LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LORA_RESET_Pin SPI1_LORA_NSS_Pin SPEAKER_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin|SPI1_LORA_NSS_Pin|SPEAKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_EXT_Pin */
  GPIO_InitStruct.Pin = DIO0_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_EXT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void DebugOutput(const char *message, int16_t number){
	//max: 93 message chars, 5 number chars, 2 lf/cr
	char buffer[100];
	snprintf(buffer, sizeof(buffer), "%s%d\r\n", message, number);

	int length = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, length, 10U);

}

void processCommand(){
	if (strcmp((const char *)rxCommandBuffer, "?") == 0 || strcmp((const char *)rxCommandBuffer, "help") == 0){
		sendCommandHelp();
	}

	if (strcmp((const char *)rxCommandBuffer, "ping") == 0){
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);
		HAL_UART_Transmit(&huart2, rxCommandBuffer, strlen((const char *)rxCommandBuffer), 10U);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);

		//uint8_t res = lora_send_packet(&lora, (uint8_t *)"PING!", 5);
		uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"PING", 4U, 1000U);

		if (res != LORA_OK) {
			HAL_UART_Transmit(&huart2, (uint8_t *)"Send to Sensor System Failed\n\r", 30U, 100U);
			DebugOutput("ERROR IS: ", res);
			uint8_t resetResult = lora_init(&lora, &hspi1, SPI1_LORA_NSS_GPIO_Port, SPI1_LORA_NSS_Pin, LORA_BASE_FREQUENCY_US);
			DebugOutput("RESET RESULT: ", resetResult);
		} else {
			HAL_UART_Transmit(&huart2, (uint8_t *)"Send to Sensor System Success\n\r", 31U, 100U);
		}

		lora_mode_receive_continuous(&lora);
		HAL_Delay(2000);
	}

	if (strcmp((const char *)rxCommandBuffer, "get s1") == 0){
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);
		HAL_UART_Transmit(&huart2, rxCommandBuffer, strlen((const char *)rxCommandBuffer), 10U);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);

		uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"GET S1", 6U, 1000U);

		if (res != LORA_OK) {
			DebugOutput("ERROR IS: ", res);
		} else {
			DebugOutput("GET S1 to Sensor System sent ", res);
		}

		lora_mode_receive_continuous(&lora);
		HAL_Delay(2000);
	}

	if (strcmp((const char *)rxCommandBuffer, "get l") == 0){
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);
		HAL_UART_Transmit(&huart2, rxCommandBuffer, strlen((const char *)rxCommandBuffer), 10U);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);

		uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"GET L", 5U, 1000U);

		if (res != LORA_OK) {
			DebugOutput("ERROR IS: ", res);
		} else {
			DebugOutput("LOG request sent ", res);
		}

		lora_mode_receive_continuous(&lora);
		HAL_Delay(2000);
	}

	if (strcmp((const char *)rxCommandBuffer, "res g") == 0){
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);
		HAL_UART_Transmit(&huart2, rxCommandBuffer, strlen((const char *)rxCommandBuffer), 10U);
		HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2U, 10U);

		uint8_t res = lora_send_packet_blocking(&lora, (uint8_t *)"RES G", 5U, 1000U);

		if (res != LORA_OK) {
			DebugOutput("ERROR IS: ", res);
		} else {
			DebugOutput("GPS reset request sent. ", res);
		}

		lora_mode_receive_continuous(&lora);
		HAL_Delay(2000);
	}

	sendCommandPrompt();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == DIO0_EXT_Pin){

		if (loRaRXBufferStatus == 1)
			return; //don't accept another packet until the first is taken and processed.

		uint8_t len = lora_receive_packet(&lora, loRaRXBuffer, sizeof(loRaRXBuffer), NULL); //no storing error info yet

		if (len > 0) {
			loRaRXBufferStatus = 1;
		}

		//lora_clear_interrupt_rx_all(&lora);
	}
}

void sendIntroduction(){
	uint8_t introduction[] = "\r\n-----------------------------\r\n"
							 "\x1b[1;31mSensor System Gateway v1.0   \r\n"
							 "\x1b[1;37mtype: help or ? for commands.\r\n"
							 "-----------------------------\r\n\r\n";
	HAL_UART_Transmit(&huart2, introduction, strlen((const char *)introduction), 20U);
}

void sendCommandHelp(){
	uint8_t help[] ="\r\n\r\nCommands:\r\n"
  				    "\x1b[1;37mGET T  \t \x1b[1;32m(Get temperature)\r\n"
				    "\x1b[1;37mGSYNC  \t \x1b[1;32m(Ask GPS to Sync time)\r\n"
				    "\x1b[1;37mRES S  \t \x1b[1;32m(Reset System)\r\n"
				    "\x1b[1;37mRES G  \t \x1b[1;32m(Reset GPS Module)\r\n"
				    "\x1b[1;37mGET L  \t \x1b[1;32m(Download log)\r\n"
					"\x1b[1;37mGET S1 \t \x1b[1;32m(Get last sensor 1 value)\r\n"
					"\x1b[1;37mGET S2 \t \x1b[1;32m(Get last sensor 2 value)\r\n"
					"\x1b[1;37mINF    \t \x1b[1;32m(Current system health and status)\r\n"
					"\x1b[1;37mPING   \t \x1b[1;32m(LoRa comms test)\r\n"
					"\x1b[1;37m";

	HAL_UART_Transmit(&huart2, help, strlen((const char *)help), 40U);
}

void sendCommandPrompt(){
	HAL_UART_Transmit(&huart2, (uint8_t *)"COMMAND>", 8U, 10U);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if (serialRecieved == 1){
		return;
	}
	if (huart->Instance == USART2){
		//echo back
		//HAL_USART_Transmit(&husart2, &RX_Char, 1, 10);
		//HAL_USART_Transmit(&husart2, &rxBuffer, 10, 10);
		//new line for each carriage return.
		//if (RX_Char == 13){
			//HAL_USART_Transmit(&husart2, (uint8_t *)"\n", 1, 10);
		//}
		serialRecieved = 1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_UART_Receive_IT(huart, &rxChar, 1);
	}

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
