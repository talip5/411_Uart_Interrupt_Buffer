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
  */

#include "stm32f4xx_hal.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define RxBufferSize 10
uint16_t RxBuffer[RxBufferSize];
uint32_t data_Rx = 0;
uint32_t rx_data = 0;
uint32_t temp_data = 0;
uint8_t buffer_size = 0;

#define BUFFER_SIZE 256
typedef struct UART_Buffer_Type{
	uint32_t buffer[BUFFER_SIZE];
	uint32_t head_pointer;
	uint32_t tail_pointer;
}UART_Buffer_t;

volatile UART_Buffer_t UART_BufferRX;
volatile UART_Buffer_t UART_BufferTX;



uint32_t deneme_Tx[10];
uint8_t RXNE_value = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  //USART2->CR1 |= USART_CR1_RXNEIE;

    //SET_BIT(USART2->CR1, USART_CR1_TXEIE);

  /* 4- Enable UART Receive Data Register Not Empty */
   SET_BIT(USART2->CR1, USART_CR1_RXNEIE);

   /* 5 - Enable UART Interrupt in NVIC */

	HAL_NVIC_SetPriority(USART2_IRQn, 15, 15);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_TogglePin(LED12_GPIO_Port, LED12_Pin);
		HAL_Delay(1000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//deneme_Tx = USART2->DR;
		/*RXNE_value = USART2->SR & USART_SR_RXNE;
		 if (RXNE_value == 32) {
		 HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
		 time1();
		 }*/
		while (USART2->SR & USART_SR_RXNE) {
			HAL_GPIO_TogglePin(LED13_GPIO_Port, LED13_Pin);
			time1();
		}
		//HAL_GPIO_TogglePin(LED13_GPIO_Port,LED13_Pin);
		/*HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_SET);
		 HAL_Delay(500);
		 HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_RESET);
		 HAL_Delay(500);*/
		// for(uint16_t i=0; i<50; i++){
		//RxBuffer[i]='A';
		// }
		//data_Rx = USART2->DR;
		 //HAL_Delay(100);
		/*RXNE_value = USART2->SR & USART_SR_RXNE;
		if (RXNE_value == 32) {
			HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_SET);
			//deneme_Tx = USART2->DR;
			//HAL_Delay(100);
			while(USART2->SR & USART_SR_RXNE){
				HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin, GPIO_PIN_SET);

				//USART2->DR = 'K';
				//HAL_Delay(100);
				//deneme_Tx = USART2->DR;

				//USART2->SR &=~(USART_SR_RXNE);

				//USART2->CR1 |= USART_CR1_RXNEIE;

			}

			//HAL_GPIO_WritePin(LED12_GPIO_Port,LED12_Pin, GPIO_PIN_SET);

			//deneme_Tx = USART2->DR;
		} else if (RXNE_value == 0) {
			HAL_GPIO_WritePin(LED15_GPIO_Port, LED15_Pin, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(LED12_GPIO_Port,LED12_Pin, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(LED14_GPIO_Port, LED14_Pin, GPIO_PIN_RESET);
			//HAL_Delay(100);
		}*/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED12_Pin|LED13_Pin|LED14_Pin|LED15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED12_Pin LED13_Pin LED14_Pin LED15_Pin */
  GPIO_InitStruct.Pin = LED12_Pin|LED13_Pin|LED14_Pin|LED15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void USART2_IRQHandler(void) {

	uint32_t isrflags = USART2->SR;
	uint32_t control_reg1 = USART2->CR1;

	/* UART in mode Receiver */
	if (((isrflags & USART_SR_RXNE) != RESET)
			&& ((control_reg1 & USART_CR1_RXNEIE) != RESET)) {
		if (buffer_size <= 3) {
			time1();
			deneme_Tx[buffer_size] = USART2->DR;
			HAL_GPIO_TogglePin(LED14_GPIO_Port, LED14_Pin);

			buffer_size++;
			return;
		} else {
			HAL_GPIO_TogglePin(LED15_GPIO_Port, LED15_Pin);
			SET_BIT(USART2->CR1, USART_CR1_TXEIE);
			time2();
			//temp_data = USART2->DR;
			USART2->SR &= ~(USART_SR_RXNE);
			return;
		}

	}
	//UART in mode Transmitter
	if (((isrflags & USART_SR_TXE) != RESET)
			&& ((control_reg1 & USART_CR1_TXEIE) != RESET)) {
		//USART2->DR = 'K';
		for(int i=0; i<=3; i++){
			USART2->DR=deneme_Tx[i];
			time1();
		}
		HAL_GPIO_TogglePin(LED14_GPIO_Port, LED14_Pin);
		USART2->CR1 &=~(USART_CR1_TXEIE);
		time1();
		USART2->SR |=(USART_SR_RXNE);
		buffer_size = 0;
		time1();
		return;
	}
}

void time1(void) {
	for (int var = 0; var < 50000; ++var) {

	}
}

void time2(void) {
	for (int var = 0; var < 5000000; ++var) {

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
