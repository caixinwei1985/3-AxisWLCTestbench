/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_hal_iwdg.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtthread.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
uint16_t ADC_Get_NTC_Volt(uint32_t ch);
uint16_t ADC_Get_VBAT(void);
/* USER CODE BEGIN EFP */
uint32_t Get_GPIOPinBit(GPIO_TypeDef* port,uint32_t pin);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW_24V_Pin LL_GPIO_PIN_15
#define SW_24V_GPIO_Port GPIOC
#define SW_5V_Pin LL_GPIO_PIN_0
#define SW_5V_GPIO_Port GPIOF
#define SW_LAMP_Pin LL_GPIO_PIN_1
#define SW_LAMP_GPIO_Port GPIOF
#define SW_LED0_Pin LL_GPIO_PIN_0
#define SW_LED0_GPIO_Port GPIOC
#define SW_BUZZER_Pin LL_GPIO_PIN_1
#define SW_BUZZER_GPIO_Port GPIOC
#define SW_LASER_Pin LL_GPIO_PIN_2
#define SW_LASER_GPIO_Port GPIOC
#define FAN1_Pin LL_GPIO_PIN_4
#define FAN1_GPIO_Port GPIOC
#define FAN2_Pin LL_GPIO_PIN_5
#define FAN2_GPIO_Port GPIOC
#define BT_JS2_Pin LL_GPIO_PIN_0
#define BT_JS2_GPIO_Port GPIOB
#define BT_JS2_EXTI_IRQn EXTI0_1_IRQn
#define BT_JS1_Pin LL_GPIO_PIN_1
#define BT_JS1_GPIO_Port GPIOB
#define BT_JS1_EXTI_IRQn EXTI0_1_IRQn
#define JS2_D3_Pin LL_GPIO_PIN_10
#define JS2_D3_GPIO_Port GPIOB
#define JS2_D2_Pin LL_GPIO_PIN_11
#define JS2_D2_GPIO_Port GPIOB
#define JS2_D1_Pin LL_GPIO_PIN_12
#define JS2_D1_GPIO_Port GPIOB
#define JS2_D0_Pin LL_GPIO_PIN_13
#define JS2_D0_GPIO_Port GPIOB
#define JS1_D3_Pin LL_GPIO_PIN_14
#define JS1_D3_GPIO_Port GPIOB
#define JS1_D2_Pin LL_GPIO_PIN_15
#define JS1_D2_GPIO_Port GPIOB
#define JS1_D1_Pin LL_GPIO_PIN_6
#define JS1_D1_GPIO_Port GPIOC
#define JS1_D0_Pin LL_GPIO_PIN_7
#define JS1_D0_GPIO_Port GPIOC
#define MOTOZDIR_Pin LL_GPIO_PIN_8
#define MOTOZDIR_GPIO_Port GPIOC
#define MOTOYDIR_Pin LL_GPIO_PIN_10
#define MOTOYDIR_GPIO_Port GPIOA
#define MOTOXDIR_Pin LL_GPIO_PIN_8
#define MOTOXDIR_GPIO_Port GPIOA
#define MOTOZPUL_Pin LL_GPIO_PIN_9
#define MOTOZPUL_GPIO_Port GPIOA
#define MOTOYPUL_Pin LL_GPIO_PIN_9
#define MOTOYPUL_GPIO_Port GPIOC
#define MOTOXPUL_Pin LL_GPIO_PIN_11
#define MOTOXPUL_GPIO_Port GPIOA
#define MOTOEN_Pin LL_GPIO_PIN_12
#define MOTOEN_GPIO_Port GPIOA
#define ZSW2_Pin LL_GPIO_PIN_15
#define ZSW2_GPIO_Port GPIOA
#define ZSW2_EXTI_IRQn EXTI4_15_IRQn
#define ZSW1_Pin LL_GPIO_PIN_10
#define ZSW1_GPIO_Port GPIOC
#define ZSW1_EXTI_IRQn EXTI4_15_IRQn
#define ZSW0_Pin LL_GPIO_PIN_11
#define ZSW0_GPIO_Port GPIOC
#define ZSW0_EXTI_IRQn EXTI4_15_IRQn
#define YSW2_Pin LL_GPIO_PIN_12
#define YSW2_GPIO_Port GPIOC
#define YSW2_EXTI_IRQn EXTI4_15_IRQn
#define YSW1_Pin LL_GPIO_PIN_2
#define YSW1_GPIO_Port GPIOD
#define YSW1_EXTI_IRQn EXTI2_3_IRQn
#define YSW0_Pin LL_GPIO_PIN_3
#define YSW0_GPIO_Port GPIOB
#define YSW0_EXTI_IRQn EXTI2_3_IRQn
#define XSW2_Pin LL_GPIO_PIN_4
#define XSW2_GPIO_Port GPIOB
#define XSW2_EXTI_IRQn EXTI4_15_IRQn
#define XSW1_Pin LL_GPIO_PIN_5
#define XSW1_GPIO_Port GPIOB
#define XSW1_EXTI_IRQn EXTI4_15_IRQn
#define XSW0_Pin LL_GPIO_PIN_6
#define XSW0_GPIO_Port GPIOB
#define XSW0_EXTI_IRQn EXTI4_15_IRQn
#define RX_TRIG_Pin LL_GPIO_PIN_13
#define RX_TRIG_GPIO_Port GPIOC
#define RX_TRIG_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
#define PROT0_GPIO_Port GPIOC
#define PROT0_Pin  LL_GPIO_PIN_7
#define PROT1_GPIO_Port GPIOC
#define PROT1_Pin  LL_GPIO_PIN_8
#define PROT2_GPIO_Port GPIOC
#define PROT2_Pin  LL_GPIO_PIN_9
#define BT_START_GPIO_PORT GPIOC
#define BT_START_Pin  LL_GPIO_PIN_14
#define HPOUT1_GPIO_Port GPIOA
#define HPOUT1_Pin  LL_GPIO_PIN_6
#define HPOUT2_GPIO_Port GPIOA
#define HPOUT2_Pin  LL_GPIO_PIN_7
#define MOTO_EVENT_ZERO_ENTER     0x00000001
#define MOTO_EVENT_ZERO_EXIT      0x00000002
#define MOTO_EVENT_NEAR_ENTER     0x00000004
#define MOTO_EVENT_NEAR_EXIT      0x00000008
#define MOTO_EVENT_FAR_ENTER      0x00000010
#define MOTO_EVENT_FAR_EXIT       0x00000020
#define MOTO_EVENT_OVERNEAR       0x00000040
#define MOTO_EVENT_OVERFAR        0x00000080
#define MOTO_EVENT_STEPOVER       0x00000100
#define MOTO_EVENT_RX_COLLID      0x10000000

#define MOTO_EVENT_ZERO_ENTER_X     0x00000001
#define MOTO_EVENT_ZERO_EXIT_X      0x00000002
#define MOTO_EVENT_NEAR_ENTER_X     0x00000004
#define MOTO_EVENT_NEAR_EXIT_X      0x00000008
#define MOTO_EVENT_FAR_ENTER_X      0x00000010
#define MOTO_EVENT_FAR_EXIT_X       0x00000020
#define MOTO_EVENT_OVERNEAR_X       0x00000040
#define MOTO_EVENT_OVERFAR_X        0x00000080
#define MOTO_EVENT_STEPOVER_X       0x00000100
#define MOTO_EVENT_STEPOVER_0       0x00000100

#define MOTO_EVENT_ZERO_ENTER_Y     (0x00000001<<9)
#define MOTO_EVENT_ZERO_EXIT_Y      (0x00000002<<9)
#define MOTO_EVENT_NEAR_ENTER_Y     (0x00000004<<9)
#define MOTO_EVENT_NEAR_EXIT_Y      (0x00000008<<9)
#define MOTO_EVENT_FAR_ENTER_Y      (0x00000010<<9)
#define MOTO_EVENT_FAR_EXIT_Y       (0x00000020<<9)
#define MOTO_EVENT_OVERNEAR_Y       (0x00000040<<9)
#define MOTO_EVENT_OVERFAR_Y        (0x00000080<<9)
#define MOTO_EVENT_STEPOVER_Y       (0x00000100<<9)
#define MOTO_EVENT_STEPOVER_1       (0x00000100<<9)
#define MOTO_EVENT_ZERO_ENTER_Z     (0x00000001<<18)
#define MOTO_EVENT_ZERO_EXIT_Z      (0x00000002<<18)
#define MOTO_EVENT_NEAR_ENTER_Z     (0x00000004<<18)
#define MOTO_EVENT_NEAR_EXIT_Z      (0x00000008<<18)
#define MOTO_EVENT_FAR_ENTER_Z      (0x00000010<<18)
#define MOTO_EVENT_FAR_EXIT_Z       (0x00000020<<18)
#define MOTO_EVENT_OVERNEAR_Z       (0x00000040<<18)
#define MOTO_EVENT_OVERFAR_Z        (0x00000080<<18)
#define MOTO_EVENT_STEPOVER_Z       (0x00000100<<18)
#define MOTO_EVENT_STEPOVER_2       (0x00000100<<18)
#define ADC_CHANNEL_NTC1          LL_ADC_CHANNEL_0
#define ADC_CHANNEL_NTC2          LL_ADC_CHANNEL_1
#define ADC_CHANNEL_NTC3          LL_ADC_CHANNEL_4  

__STATIC_INLINE void log(const char* s)
{
  rt_kputs(s);
}
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
