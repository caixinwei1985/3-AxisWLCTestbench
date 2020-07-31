/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "app_rtthread.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "isp.h"
#include "button.h"
#include "comm_definition.h"
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

/* USER CODE BEGIN PV */
static Button_t  Buttons[12];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Button_Notify(uint16_t notify);
static void Buttons_Init_Static();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Peripheral_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->CFGR1 = (uint32_t)0x00000000;
  BootConfig_User();
  HAL_Init();
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
//  MX_IWDG_Init();

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


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  Buttons_Init_Static();
  HAL_PWR_EnableBkUpAccess();
  rt_kprintf("BAKUP3:%hu\n",RTC->BKP3R);
 
  RTC->BKP3R = 0xcc56;
  rt_kprintf("BAKUP3:%hu\n",RTC->BKP3R);
  MX_RT_Thread_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    MX_RT_Thread_Process();
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6, LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(48000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(48000000);
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

uint32_t Get_GPIOPinBit(GPIO_TypeDef* port,uint32_t pin)
{
  if((LL_GPIO_ReadInputPort(port)&pin)==0)
    return 0;
  return 1;
}

static void Buttons_Init_Static()
{
  Button_Init(&Buttons[BT_START_IDX],BT_START_GPIO_PORT,BT_START_Pin,NTF_START_PRESS,0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS1_IDX]  ,BT_JS1_GPIO_Port,  BT_JS1_Pin,NTF_JS1BT_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS2_IDX]  ,BT_JS2_GPIO_Port,  BT_JS2_Pin,NTF_JS2BT_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_RXC_IDX]  ,RX_TRIG_GPIO_Port, RX_TRIG_Pin,NTF_RXCOL_TRIG,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS1D0_IDX],JS1_D0_GPIO_Port,  JS1_D0_Pin,NTF_JS1D0_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS1D1_IDX],JS1_D1_GPIO_Port,  JS1_D1_Pin,NTF_JS1D1_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS1D2_IDX],JS1_D2_GPIO_Port,  JS1_D2_Pin,NTF_JS1D2_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS1D3_IDX],JS1_D3_GPIO_Port,  JS1_D3_Pin,NTF_JS1D3_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS2D0_IDX],JS2_D0_GPIO_Port,  JS2_D0_Pin,NTF_JS2D0_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS2D1_IDX],JS2_D1_GPIO_Port,  JS2_D1_Pin,NTF_JS2D1_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS2D2_IDX],JS2_D2_GPIO_Port,  JS2_D2_Pin,NTF_JS2D2_PRESS,  0,Button_Notify,NULL);
  Button_Init(&Buttons[BT_JS2D3_IDX],JS2_D3_GPIO_Port,  JS2_D3_Pin,NTF_JS2D3_PRESS,  0,Button_Notify,NULL);
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);
}

void Buttons_FreshState()
{
  uint32_t i = 0;
  while(i < 12)
  {
    Button_Refresh(&Buttons[i]);
    i++; 
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
