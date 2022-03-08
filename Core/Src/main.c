/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_pid.h"

void SystemClock_Config(void);
extern uint16_t adc_buffer[3];  //引用的时候，直接引用adc_buffer[x]   eg:value=2048
extern char rx_buffer[255];
extern char rx_wait[1];
extern y_pid_InitTypedef y_pid_InitStructure;
extern x_pid_InitTypedef x_pid_InitStructure;
int x_pid_result; 
int y_pid_result;
//extern char tx_buffer[255];  //???????????

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
	pid_init();
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_wait, 1); //为了使能接收中断
  //htim1.Instance->CCR1=500;  //0/500    测试用
  //htim1.Instance->CCR3=200;  //0/500
  //8.68v   6.47v    6.47v
  while (1)
  {
    x_pid_result=x_pid_calculator(&x_pid_InitStructure,adc_buffer[0]);
    y_pid_result=y_pid_calculator(&y_pid_InitStructure,adc_buffer[1]);
    /* USER CODE END WHILE */
		//printf("%d   ",adc_buffer[0]);
		//printf("%d   ",adc_buffer[1]);
    //printf("%d   ",x_pid_result);
    //printf("%d\n",y_pid_result);
    tim_rcc_config(x_pid_result,y_pid_result);   //这个是最核心的一行
		//HAL_Delay(1000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  RX_Interrupt(huart);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
/*|||||||||||||||||||||||||||||||||||||||||||||下面是自己写的|||||||||||||||||||||||||*/
void tim_rcc_config(int X_pid_result,int Y_pid_result)
{
  int CCCR1,CCCR2,CCCR3,CCCR4;
  if(X_pid_result>=0){CCCR1=X_pid_result;CCCR2=0;}if(X_pid_result<0){CCCR1=0;CCCR2=~X_pid_result;}
  if(Y_pid_result>=0){CCCR3=Y_pid_result;CCCR4=0;}if(Y_pid_result<0){CCCR3=0;CCCR4=~Y_pid_result;}
  htim1.Instance->CCR1=CCCR1;
  htim1.Instance->CCR2=CCCR2;
  htim1.Instance->CCR3=CCCR3;
  htim1.Instance->CCR4=CCCR4;
}
