/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rtthread.h>
#include "moto.h"
#include "cmdlink.h"
#include "comm_definition.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Buttons_FreshState();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern struct rt_event evt_moto_pos;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0 */
    
    /* USER CODE END LL_EXTI_LINE_0 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1 */
    
    /* USER CODE END LL_EXTI_LINE_1 */
  }
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 2 and 3 interrupts.
  */
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */
    //YSW1
    if(MOTO_GetRunningAxis() == AxisY)
    {
      if((LL_GPIO_ReadInputPort(YSW1_GPIO_Port) & YSW1_Pin) == 0)// enter
      {
        log("Y near enter\n");
        MOTO_OnNearPointEnter(AxisY);
        if(MOTO_GetDirection(AxisY)!= MOTO_STATUS_POSITIVE)
        {
          rt_event_send(&evt_moto_pos,MOTO_EVENT_NEAR_ENTER);
          report_notify(NTF_MOTO_NEAR);
        }
      }
      else
      {
        log("y near exit\n");
        if(MOTO_OnNearPointExit(AxisY))
        {
            rt_event_send(&evt_moto_pos,MOTO_EVENT_OVERNEAR);
        }
        else
        {
          rt_event_send(&evt_moto_pos,MOTO_EVENT_NEAR_EXIT);
        }
      }
    }
    /* USER CODE END LL_EXTI_LINE_2 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */
    //YSW0
    if(MOTO_GetRunningAxis()!= AxisY)
      return;
    if((LL_GPIO_ReadInputPort(YSW0_GPIO_Port)&YSW0_Pin) == 0)
    {
      log("y zero enter\n");
      MOTO_OnZeroPointEnter(AxisY);
      rt_event_send(&evt_moto_pos,MOTO_EVENT_ZERO_ENTER);
      report_notify(NTF_MOTO_RESET);
    }
    else
    {
      MOTO_OnZeroPointExit(AxisY);
      log("y zero exit\n");
    }
    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    /* USER CODE BEGIN LL_EXTI_LINE_4 */
    //XSW2,
    if(MOTO_GetRunningAxis()!= AxisX)
      return;
    if((LL_GPIO_ReadInputPort(XSW2_GPIO_Port)&XSW2_Pin)==0) // enter
    {
      log("x far enter\n");
      MOTO_OnFarPointEnter(AxisX);
      if(MOTO_GetDirection(AxisX) == MOTO_STATUS_POSITIVE)
      {
        rt_event_send(&evt_moto_pos,MOTO_EVENT_FAR_ENTER);
        report_notify(NTF_MOTO_FAR);
      }
    }
    else
    {
      log("x far exit\n");
      if(MOTO_OnFarPointExit(AxisX))
        rt_event_send(&evt_moto_pos,MOTO_EVENT_OVERFAR);
      else
        rt_event_send(&evt_moto_pos,MOTO_EVENT_FAR_EXIT);
    }
    /* USER CODE END LL_EXTI_LINE_4 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    /* USER CODE BEGIN LL_EXTI_LINE_5 */
    //XSW1
    if(MOTO_GetRunningAxis()!=AxisX)
      return;
    if((LL_GPIO_ReadInputPort(XSW1_GPIO_Port)&XSW1_Pin) == 0)// enter
    {
      log("x near enter\n");
      MOTO_OnNearPointEnter(AxisX);
      if(MOTO_GetDirection(AxisX)!= MOTO_STATUS_POSITIVE)
      {
        rt_event_send(&evt_moto_pos,MOTO_EVENT_NEAR_ENTER);
        report_notify(NTF_MOTO_NEAR);
      }
    }
    else
    {
      log("x near exit\n");
      if(MOTO_OnNearPointExit(AxisX))
      {
          rt_event_send(&evt_moto_pos,MOTO_EVENT_OVERNEAR);
      }
      else
      {
        rt_event_send(&evt_moto_pos,MOTO_EVENT_NEAR_EXIT);
      }
    }
    /* USER CODE END LL_EXTI_LINE_5 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
    /* USER CODE BEGIN LL_EXTI_LINE_6 */
    if(MOTO_GetRunningAxis()!= AxisX)
      return;
    if((LL_GPIO_ReadInputPort(XSW0_GPIO_Port)&XSW0_Pin) == 0)
    {
      log("x zero enter\n");
      MOTO_OnZeroPointEnter(AxisX);
      rt_event_send(&evt_moto_pos,MOTO_EVENT_ZERO_ENTER);
      report_notify(NTF_MOTO_RESET);
    }
    else
    {
      MOTO_OnZeroPointExit(AxisX);
      log("x zero exit\n");
    }
    /* USER CODE END LL_EXTI_LINE_6 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    /* USER CODE BEGIN LL_EXTI_LINE_7 */
    
    /* USER CODE END LL_EXTI_LINE_7 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    /* USER CODE BEGIN LL_EXTI_LINE_10 */
    //ZSW1
    /* USER CODE END LL_EXTI_LINE_10 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    /* USER CODE BEGIN LL_EXTI_LINE_11 */
    //ZSW0
    /* USER CODE END LL_EXTI_LINE_11 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* USER CODE BEGIN LL_EXTI_LINE_12 */
    //YSW2
    //非对应轴向触发限位，忽略
    if(MOTO_GetRunningAxis()!= AxisY)
      return;
    if((LL_GPIO_ReadInputPort(YSW2_GPIO_Port)&YSW2_Pin)==0) // enter
    {
      log("y far enter\n");
      MOTO_OnFarPointEnter(AxisY);
      if(MOTO_GetDirection(AxisY) == MOTO_STATUS_POSITIVE)
      {
        rt_event_send(&evt_moto_pos,MOTO_EVENT_FAR_ENTER);
        report_notify(NTF_MOTO_FAR);
      }
    }
    else
    {
      log("y far exit\n");
      if(MOTO_OnFarPointExit(AxisY))
        rt_event_send(&evt_moto_pos,MOTO_EVENT_OVERFAR);
      else
        rt_event_send(&evt_moto_pos,MOTO_EVENT_FAR_EXIT);
    }
    /* USER CODE END LL_EXTI_LINE_12 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    /* USER CODE BEGIN LL_EXTI_LINE_13 */
    
    /* USER CODE END LL_EXTI_LINE_13 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14 */
    MOTO_Disable();
    /* USER CODE END LL_EXTI_LINE_14 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    /* USER CODE BEGIN LL_EXTI_LINE_15 */
    //ZSW2
    /* USER CODE END LL_EXTI_LINE_15 */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
  // 牢记 一定要清中断标志位，引发了一系列奇怪的问题
  if(TIM1->SR&TIM_SR_UIF)
  {
    TIM1->SR &= ~TIM_SR_UIF;
    Axis_t axis = MOTO_GetRunningAxis();
    uint16_t speed = MOTO_ISRHandler(axis);
    if(speed == 0)
    {
      MOTO_Stop(axis);
      rt_event_send(&evt_moto_pos,MOTO_EVENT_STEPOVER);
    }
    else
    {
      uint32_t arr = 48000000/speed;
      if(arr>0xffff)
        arr = 0xffff;
      TIM1->ARR = arr;
      switch(MOTO_GetRunningAxis())
      {
        case AxisX:
          TIM1->CCR4=TIM1->ARR/2;
          break;
        case AxisY:
          TIM1->CCR3=TIM1->ARR/2;
          break;
        case AxisZ:
          TIM1->CCR2=TIM1->ARR/2;
          break;
        default:
          break;
      }
    }
  }
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  if(TIM6->SR & TIM_SR_UIF)
  {
    TIM6->SR &= ~TIM_SR_UIF;
    HAL_IncTick();
    Buttons_FreshState();
  }
  /* USER CODE END TIM6_DAC_IRQn 0 */
  
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  if(LL_USART_IsActiveFlag_TC(USART2))
  {
    LL_USART_ClearFlag_TC(USART2);
    
  }
  if(LL_USART_IsActiveFlag_TXE(USART2))
  {
    hal_transfer_byte_IT(1);
  }
  if(LL_USART_IsActiveFlag_RXNE(USART2))
  {
    hal_receive_byte_IT(1);
  }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
