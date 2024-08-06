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
uint32_t left_toggles = 0;
uint8_t BANDERALEFT1=0;
uint8_t BANDERALEFT2=0;
uint8_t BANDERARIGHT1=0;
uint8_t BANDERARIGHT2=0;
uint32_t buttonPressCount = 0;
uint32_t button2PressCount = 0;
uint32_t button3PressCount = 0;
uint32_t lastButtonPressTime = 0;
uint32_t debounceTime = 200; // Tiempo de debounce en ms
uint32_t doublePressTime = 500; // Tiempo máximo entre dos pulsaciones para considerar una doble pulsación
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void turn_signal_left(void) //FUNCION ENCARGADA DEL ENCENDIDO Y APAGADO DEL LEDLEFT
{
	static uint32_t hearbeat_tick = 0;
	if (hearbeat_tick < HAL_GetTick()){
		hearbeat_tick = HAL_GetTick() + 500; //500 milisegundos para tener una frecuencia de 2Hz
		HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin);
	}}
void turn_signal_right(void) //FUNCION ENCARGADA DEL ENCENDIDO Y APAGADO DEL LEDRIGHT
{
	static uint32_t hearbeat_tick = 0;
	if (hearbeat_tick < HAL_GetTick()){
		hearbeat_tick = HAL_GetTick() + 500;
		HAL_GPIO_TogglePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin);
	}}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_LEFT_Pin) {
         uint32_t currentTime = HAL_GetTick();
        if (currentTime - lastButtonPressTime > debounceTime) {
            // Detectar número de pulsaciones seguidas
            if (currentTime - lastButtonPressTime < doublePressTime) {
                buttonPressCount++;
            } else {
                buttonPressCount = 1; // Reiniciar si la pulsación no es seguida
            }
            lastButtonPressTime = currentTime;

            // Manejo de acciones según el número de pulsaciones
            if (buttonPressCount == 1) { //UNA PULSACION
            	BANDERALEFT1=1;
            	HAL_UART_Transmit(&huart2, "LEFT\r\n", 6, 10);
}

             if (buttonPressCount == 2) { //UNA PULSACION
            	BANDERALEFT2=1;
            	HAL_UART_Transmit(&huart2, "LEFT\r\n", 6, 10);

            }
        }
    }

    if (GPIO_Pin == BUTTON_RIGHT_Pin) {
        uint32_t currentTime = HAL_GetTick();
        if (currentTime - lastButtonPressTime > debounceTime) {
            // Detectar número de pulsaciones seguidas
            if (currentTime - lastButtonPressTime < doublePressTime) {
                buttonPressCount++;
            } else {
                buttonPressCount = 1; // Reiniciar si la pulsación no es seguida
            }
            lastButtonPressTime = currentTime;

            // Manejo de acciones según el número de pulsaciones
            if (buttonPressCount == 1) { //UNA PULSACION

            	HAL_UART_Transmit(&huart2, "RIGHT\r\n", 7, 10);
            	BANDERARIGHT1=1;

            } else if (buttonPressCount == 2) { //DOS PULSACIONES

            	HAL_UART_Transmit(&huart2, "RIGHT\r\n", 7, 10);
            	BANDERARIGHT2=1;

            }
        }
    }
}
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    /* USER CODE END WHILE */
	  if(BANDERALEFT1!=0){
		  turn_signal_left();
	  }

	  if(BANDERARIGHT1!=0){
		  turn_signal_right();

	  }

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}}

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_LEFT_Pin|LED_RIGHT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : BUTTON_LEFT_Pin BUTTON_RIGHT_Pin */
  GPIO_InitStruct.Pin = BUTTON_LEFT_Pin|BUTTON_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LEFT_Pin LED_RIGHT_Pin */
  GPIO_InitStruct.Pin = LED_LEFT_Pin|LED_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
