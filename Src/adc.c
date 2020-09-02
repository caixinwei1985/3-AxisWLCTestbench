/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "rtthread.h"
/* USER CODE END 0 */

/* ADC init function */
void MX_ADC_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration  
  PA0   ------> ADC_IN0
  PA1   ------> ADC_IN1
  PA4   ------> ADC_IN4
  PA5   ------> ADC_IN5
  PA6   ------> ADC_IN6
  PA7   ------> ADC_IN7 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_4);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VREFINT);
//  /** Configure Regular Channel 
//  */
//  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_VBAT);
  /** Configure Internal Channel 
  */
  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_VREFINT|LL_ADC_PATH_INTERNAL_TEMPSENSOR|LL_ADC_PATH_INTERNAL_VBAT);
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1); 
  
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADEN) != 0);
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN; /* (3) */
  ADC1->CR |= ADC_CR_ADCAL; /* (4) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (5) */
  {
  /* For robust implementation, add here time-out management */
  }
  LL_ADC_Enable(ADC1);
  ADC1_COMMON->CCR |= ADC_CCR_VBATEN;

}

/* USER CODE BEGIN 1 */
uint16_t ADC_Get_NTC_Volt(uint32_t ch)
{
  int32_t dr;
  LL_ADC_REG_SetSequencerChAdd(ADC1,ch);
  LL_ADC_REG_StartConversion(ADC1);
  while(LL_ADC_REG_IsConversionOngoing(ADC1));
  dr = LL_ADC_REG_ReadConversionData12(ADC1);
  LL_ADC_REG_SetSequencerChRem(ADC1,ch);
  dr = dr*3300/4095;
  return dr;
}

/** 
  * @brife  Get VBAT voltage unit in mV
  * @Note   CR2032 Li-battery used externally,volt range between 1.62~3.0
  */
uint16_t ADC_Get_VBAT(void)
{
  int32_t dr;
  // 选定通道
  
  LL_ADC_REG_SetSequencerChAdd(ADC1,LL_ADC_CHANNEL_VBAT);
  // 启动转换
  LL_ADC_REG_StartConversion(ADC1);
  // 等待结果
  while(LL_ADC_REG_IsConversionOngoing(ADC1));
  dr = LL_ADC_REG_ReadConversionData12(ADC1)&0x0fff;
  LL_ADC_REG_SetSequencerChRem(ADC1,LL_ADC_CHANNEL_VBAT);
  dr = dr*3300/4095*2;
  return (uint16_t)dr;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
