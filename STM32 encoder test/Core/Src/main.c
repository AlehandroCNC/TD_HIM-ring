/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t buffer[64]; //Буфер для приёма сообщений от компа
uint32_t count, lg, speed;
uint8_t message[8] = {'\0'};
uint32_t calBuffer[9]; //{0 Начало калибровки, 1 ES_1_B, 2 ES_1_E, 3 ES_2_B, 4 ES_2_E, 5 stop_start, 6 stop_end, 7 ES1_pos, 8 ES2_pos}
uint32_t deselerationDist;
uint32_t breakAt[2]; //Координата торможения, целевая координата
uint8_t move; //направление вращения n=стоит, f=в +, b=в -
union {
	int32_t u32;
	uint8_t u8[4];
} u;

uint32_t delta (uint32_t num_1, uint32_t num_2) {
	if (num_1 > num_2) {return num_1 - num_2;}
	else if (num_1 < num_2) {return num_2 - num_1;}
	else {return 0;}
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
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
  

  void sendData (uint8_t data[], int length) {
	CDC_Transmit_FS(data, length);
  }


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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

	void calibrate () {	  
	  
	  sendData("\nCalibration start", 18);
	  TIM4->ARR = 65535;
	  //Проверка статичности колеса
	  memset(calBuffer, 0, sizeof(calBuffer));
	  calBuffer[0] = TIM4->CNT;
	  HAL_Delay(1000);
	  if (TIM4->CNT != calBuffer[0]) {
		  sendData("\nSistem static test faild", 25);
	  } else {sendData("\nSistem static test complite", 28);}

	  //Калибровка
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  int i = 0; //Счётчик калибровки
	  while (i < 4) { //Запись срабатываней концевика
		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET && i==0) {calBuffer[1] = TIM4->CNT; i = 1; sendData("\nES_1_B", 7);} //ES_1_B
		  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET && i==1) {calBuffer[2] = TIM4->CNT; i = 2; sendData("\nES_1_E", 7);} //ES_1_E
		  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET && i==2) {calBuffer[3] = TIM4->CNT; i = 3; sendData("\nES_2_B", 7);} //ES_2_B
		  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET && i==3) {calBuffer[4] = TIM4->CNT; i = 4; sendData("\nES_2_E", 7);} //ES_2_E
		  } 
	  
	  //Остановка мотора, замер времени остановки
	  calBuffer[5] = TIM4->CNT; //Значение перед остановкой
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //остановка двигателя
	  int d[3] = {0, 0, 0}; //Буфер дельт
	  int e = calBuffer[5]; //Бувер значения энкодера
	  HAL_TIM_Base_Start_IT(&htim3); //Запуск таймера времени
	  while (d[2] < 1000) {
		  if (e != TIM4->CNT) {
			  e = TIM4->CNT; 
			  d[1] = d[0]; 
			  d[0] = TIM3->CNT;
			  if (d[1] <= d[0]) {d[2] = d[0] - d[1];}
			  else if (d[1] > d[0]) {d[2] = (65535 + d[0]) - d[1];}  		  
		  }
		  if (TIM3->CNT > 2000 + d[0]) {d[2] = 2000;}
	  }
	  HAL_TIM_Base_Stop_IT(&htim3); //Остановка таймера времени
	  TIM3->CNT = 0; //сброс таймера времени
	  calBuffer[6] = e;
	  
	  //Применение результатов калибровки
	  deselerationDist = delta(calBuffer[6], calBuffer[5]);
	  calBuffer[7] = calBuffer[1]; //delta(calBuffer[2], delta(calBuffer[2], calBuffer[1]) / 2);
	  calBuffer[8] = calBuffer[3]; //delta(calBuffer[4], delta(calBuffer[4], calBuffer[3]) / 2);
	  TIM4->CNT = deselerationDist;
	  TIM4->ARR = delta(calBuffer[8], calBuffer[7]);
	  lg = sprintf(message, "\nCalibration end, ARR=%d, CNT=%d", TIM4->ARR, TIM4->CNT);
	  sendData(message, lg);
	}

	void goToPos (int targetPos) {
		uint32_t disToMove[3] = {0, 0, deselerationDist}; //{по часовой, против часовой}	
		//breakAt = 0;
		
		if (targetPos < TIM4->ARR && targetPos > 0) {
			lg = sprintf(message, "\nMoving to %d", targetPos);
			sendData(message, lg);
		} else {
			lg = sprintf(message, "\nErr pos %d out of range", targetPos);
			sendData(message, lg);
			return;
		}
		

		//Вычисление дистанций движения в обе стороны
		if (TIM4->CNT < targetPos) {disToMove[0] = targetPos - TIM4->CNT; disToMove[1] = (TIM4->ARR - targetPos) + TIM4->CNT;}
		else if (TIM4->CNT > targetPos) {disToMove[0] = TIM4->ARR - TIM4->CNT + targetPos; disToMove[1] = TIM4->CNT - targetPos;}
		else {return;}

		//определение направления
		if (disToMove[0] > disToMove[1]) {move = 'b';} 
		else if (disToMove[0] == disToMove[1]) {move = 'n';} 
		else {move = 'f';}

		//Поправка торможения для коротких поездок
		if (disToMove[0] < disToMove[2]*2 && move == 'f') {
			disToMove[2] = disToMove[0]/2;
		}
		else if (disToMove[1] < disToMove[2] * 2 && move == 'b') {
			disToMove[2] = disToMove[1]/2;
		}

		//Определение координаты торможения
		if (move == 'f') {
			if (disToMove[2] > targetPos) {
				breakAt[0] = TIM4->ARR - (disToMove[2] - targetPos);
			} else {
				breakAt[0] = targetPos - disToMove[2];
			}
		} else if (move == 'b'){
			if (disToMove[2] > TIM4->ARR - targetPos) {
				breakAt[0] = disToMove[2] - (TIM4->ARR - targetPos);
			} else {
				breakAt[0] = targetPos + disToMove[2];
			}
		}

		breakAt[1] = targetPos;

		//Включение мотора
		if (move == 'f') {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);}
		else if (move == 'b') {HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);}

		//Дебаг сообщения
		lg = sprintf(message, "\nbreakeAt %d, move %c, DTM f=%d, DTM b=%d, break dist=%d", breakAt[0], move, disToMove[0], disToMove[1], disToMove[2]);
		sendData(message, lg);
		
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  count = TIM4->CNT; //получение данных энкодера

	  if (count > TIM4->ARR/2) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  } else {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  } //мигалка

	  //Обработка команд
	switch (buffer[0]) {
		case 'c':
			calibrate();
			break;
		case 'g':
			for (int i=1; i<=4; i++) {
				u.u8[i-1] = buffer[i];
			}
			goToPos(u.u32);
			break;
		case 'n':
			goToPos(0);
			break;
		case 'p':
			lg = sprintf(message, "\n%d", TIM4->CNT);
			sendData(message, lg);
			break;
		case 't':
			u.u32 = 2054781047;
			for (int i = 0; i <= 3; i++) {message[i] = u.u8[i];}
			sendData(message, 4);
			break;
		case '0':
			goToPos(1);
			break;
		case '1':
			goToPos(TIM4->ARR/8*1);
			break;
		case '2':
			goToPos(TIM4->ARR/8*2);
			break;
		case '3':
			goToPos(TIM4->ARR/8*3);
			break;
		case '4':
			goToPos(TIM4->ARR/8*4);
			break;
		case '5':
			goToPos(TIM4->ARR/8*5);
			break;
		case '6':
			goToPos(TIM4->ARR/8*6);
			break;
		case '7':
			goToPos(TIM4->ARR/8*7);
			break;
	  }
	  memset(buffer, 0, sizeof(buffer));

	  //Ожидание координаты остановки
	  if (move == 'f' && ((breakAt[0] < breakAt[1] && TIM4->CNT >= breakAt[0]) || (breakAt[0] > breakAt[1] && (TIM4->CNT >= breakAt[0] || TIM4->CNT < breakAt[1])))) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		  move = 'n';
		  sendData("\ndone F", 7);
	  } else if (move == 'b' && ((breakAt[0] > breakAt[1] && TIM4->CNT <= breakAt[0]) || (breakAt[0] < breakAt[1] && (TIM4->CNT <= breakAt[0] || TIM4->CNT > breakAt[1])))) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		  move = 'n';
		  sendData("\ndone B", 7);
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 26665;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Step_Pin|Dir_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Step_Pin Dir_Pin */
  GPIO_InitStruct.Pin = Step_Pin|Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Endstop_1_Pin Endstop_2_Pin */
  GPIO_InitStruct.Pin = Endstop_1_Pin|Endstop_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
