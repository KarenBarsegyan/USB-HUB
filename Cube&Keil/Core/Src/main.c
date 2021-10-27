/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_COUNT 10

#define TRESET 48   // not a joke, put at least 2240
#define HIGH 65
#define LOW 26

#define DATA_LENGTH TRESET + LED_COUNT*24 + 1

#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)

#define PassLength 15
#define ReportBuffLength PassLength + 2  //1 byte of pass lenght and 1 byte of report ID

#define I2C_ADDRESS 0x09
#define I2C_TIMEOUT 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch3;
DMA_HandleTypeDef hdma_tim4_ch3;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef  hUsbDeviceFS;

uint8_t password[ReportBuffLength];
uint8_t passNum = 2;

int TimOfBip = 0;
int CanISend = 0;
int pinPad = -1;
int pinPadPrev = -1;
uint8_t RGB[3];

uint8_t regData = 0;
uint8_t regAddress;
int Light = 0;
int LightSum = 0;
int LightAmountOfCheck = 0;
int PreviousLightSum = 0;
int WhiteOrNot = 0;

uint16_t ws2811_data [DATA_LENGTH] = {0};
uint16_t ws2811_data_1 [DATA_LENGTH] = {0};
uint16_t count = 0;
int decrement = 1;
uint16_t adcResult = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void ws2811_init(void);
void ws2811_setcolor(int pos, int R_value, int G_value, int B_value);
void ws2811_setcolor_1(int pos, int R_value, int G_value, int B_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void checkPinPad()
{
	pinPad = -1;
	for(int i = 0; i < 4; ++i)
	{
		switch (i)
    {
        case 0: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); break; 
    }
	
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
			pinPad = i+1;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)){
			pinPad = 5+i;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
			pinPad = 9+i;
		}else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
			pinPad = 13+i;
		}
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		
		if(pinPad != -1)
			break;
	}
	HAL_Delay(10);
	return;
}

int ConvertPass(int num)
{
	switch(num){
			case 8: return 0;
			case 1: return 1;
			case 5: return 2;
			case 9: return 3;
			case 2: return 4;
			case 6: return 5;
			case 10: return 6;
			case 3: return 7;
			case 7: return 8;
			case 11: return 9;
			case 13: return 14;
			case 14: return 15;
			case 15: return 16;
			default: return -1;
	}
	
}

void Bip(int time)
{
		TimOfBip = 0;
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_4);
		while(TimOfBip < time) 
		{
			HAL_Delay(1);
		}
		HAL_TIM_PWM_Stop_IT(&htim2,TIM_CHANNEL_4);
}

int checkLight()
{
	regAddress = 0x15;//0x11;
	HAL_I2C_Master_Transmit(&hi2c1, (I2C_ADDRESS << 1), &regAddress, 1,  I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2c1, (I2C_ADDRESS << 1), &regData, 1,  I2C_TIMEOUT);
	
	return regData;
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
  MX_USB_DEVICE_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	ws2811_init();
	//HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, (uint32_t*)&ws2811_data, DATA_LENGTH);
	//HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)&ws2811_data, DATA_LENGTH);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	
	for(int i = 0; i < ReportBuffLength; ++i){
		password[i] = 0;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(100);//INIT;
  while (1)
  {
		Light = checkLight();
		LightSum+=Light;
		LightAmountOfCheck++;
		if(LightAmountOfCheck >= 5){
			PreviousLightSum = LightSum/5;
			LightAmountOfCheck = 0;
			LightSum = 0;
		}
		
		if(PreviousLightSum > 90)
			WhiteOrNot = 1;
		else if (PreviousLightSum < 80)
			WhiteOrNot = 0;
		
		if(WhiteOrNot) 
		{
			for(int i = 0; i < LED_COUNT; ++i)
			{
				ws2811_setcolor(i, 100, 100, 100);
			}
		} else 
		{
			for(int i = 0; i < LED_COUNT; ++i)
			{
				ws2811_setcolor(i, 0, 0, 0);
			}
		}
		HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_3, (uint32_t*)&ws2811_data, DATA_LENGTH);
		
		for(int i = 0; i < LED_COUNT; ++i)
		{
			ws2811_setcolor_1(i, RGB[0], RGB[1], RGB[2]);
		}
		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t*)&ws2811_data_1, DATA_LENGTH);
		
		checkPinPad();
		pinPad = ConvertPass(pinPad);
		
		if(pinPad != pinPadPrev && pinPad != -1) {
			if(pinPad == 14 && passNum > 2) 
			{				
				Bip(300);
				HAL_Delay(100);
				Bip(300);
				
				passNum = 2;
				for(int i = 0; i < ReportBuffLength; ++i){
					password[i] = 0;
				}
				
			}else if(pinPad == 15 && passNum > 2) 
			{
				Bip(300);
				
				passNum--;
				password[passNum] = 0;
		
			}else if(pinPad == 16 && passNum > 2) 
			{
				Bip(300);
				
				password[0] = 3;
				password[1] = passNum-2;
				passNum = 2;
				
				uint8_t ToSend[ReportBuffLength];
				
				for(int i = 0; i < ReportBuffLength; ++i){
					ToSend[i] = password[i];
					password[i] = 0;
				}

				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, ToSend, ReportBuffLength);
					
			}else if(passNum < ReportBuffLength && pinPad != 14  && pinPad != 15  && pinPad != 16)
			{
				password[passNum] = pinPad;
				passNum++;
				
				Bip(100);
				
			}
		}
	
		pinPadPrev = pinPad;//*/
	
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 360-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PinPad1_1_Pin|PinPad1_2_Pin|PinPad1_3_Pin|PinPad1_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PinPad1_1_Pin PinPad1_2_Pin PinPad1_3_Pin PinPad1_4_Pin */
  GPIO_InitStruct.Pin = PinPad1_1_Pin|PinPad1_2_Pin|PinPad1_3_Pin|PinPad1_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PinPad2_1_Pin PinPad2_2_Pin PinPad2_3_Pin PinPad2_4_Pin */
  GPIO_InitStruct.Pin = PinPad2_1_Pin|PinPad2_2_Pin|PinPad2_3_Pin|PinPad2_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ws2811_init(void)
{
	for (int c = 0; c < TRESET; c++) {
		ws2811_data[c] = 0;
		ws2811_data_1[c] = 0;
	}
	for (int c = TRESET; c < DATA_LENGTH-1; c++) {
		ws2811_data[c] = LOW;
		ws2811_data_1[c] = 0;
	}
	
	ws2811_data[DATA_LENGTH-1] = 0;
	ws2811_data_1[DATA_LENGTH-1] = 0;
}
void ws2811_setcolor(int pos, int R_value, int G_value, int B_value) {
	
	int offset = TRESET + pos*24;
	int i = 0;
	
	for (i = 0; i < 8; i++) {
		if (BitIsSet(G_value, (7-i))) {
			ws2811_data[offset + i] = HIGH;
		} else {
			ws2811_data[offset + i] = LOW;
		}
	}

	for (i = 0; i < 8; i++) {
		if (BitIsSet(R_value, (7-i))) {
			ws2811_data[offset + 8 + i] = HIGH;
		} else {
			ws2811_data[offset + 8 + i] = LOW;
		}
	}

	for (i = 0; i < 8; i++) {
		if (BitIsSet(B_value, (7-i))) {
			ws2811_data[offset + 16 + i] = HIGH;
		} else {
			ws2811_data[offset + 16 + i] = LOW;
		}
	}
}

void ws2811_setcolor_1(int pos, int R_value, int G_value, int B_value) {
	
	int offset = TRESET + pos*24;
	int i = 0;
	
	for (i = 0; i < 8; i++) {
		if (BitIsSet(G_value, (7-i))) {
			ws2811_data_1[offset + i] = HIGH;
		} else {
			ws2811_data_1[offset + i] = LOW;
		}
	}

	for (i = 0; i < 8; i++) {
		if (BitIsSet(R_value, (7-i))) {
			ws2811_data_1[offset + 8 + i] = HIGH;
		} else {
			ws2811_data_1[offset + 8 + i] = LOW;
		}
	}

	for (i = 0; i < 8; i++) {
		if (BitIsSet(B_value, (7-i))) {
			ws2811_data_1[offset + 16 + i] = HIGH;
		} else {
			ws2811_data_1[offset + 16 + i] = LOW;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
