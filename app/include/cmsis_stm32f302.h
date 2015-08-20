/* Copyright (c) 2009 - 2014 ARM LIMITED

All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
- Neither the name of ARM nor the names of its contributors may be used
 to endorse or promote products derived from this software without
 specific prior written permission.
*
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------------*/

#pragma once

#define __CM4_REV                 0x0001
#define __MPU_PRESENT             1
#define __NVIC_PRIO_BITS          4
#define __Vendor_SysTickConfig    0
#define __FPU_PRESENT             1

typedef enum IRQn
{
  NonMaskableInt_IRQn         = -14,
  MemoryManagement_IRQn       = -12,
  BusFault_IRQn               = -11,
  UsageFault_IRQn             = -10,
  SVCall_IRQn                 = -5,
  DebugMonitor_IRQn           = -4,
  PendSV_IRQn                 = -2,
  SysTick_IRQn                = -1,
  WWDG_IRQn                   = 0,
  PVD_IRQn                    = 1,
  TAMPER_STAMP_IRQn           = 2,
  RTC_WKUP_IRQn               = 3,
  FLASH_IRQn                  = 4,
  RCC_IRQn                    = 5,
  EXTI0_IRQn                  = 6,
  EXTI1_IRQn                  = 7,
  EXTI2_TS_IRQn               = 8,
  EXTI3_IRQn                  = 9,
  EXTI4_IRQn                  = 10,
  DMA1_Channel1_IRQn          = 11,
  DMA1_Channel2_IRQn          = 12,
  DMA1_Channel3_IRQn          = 13,
  DMA1_Channel4_IRQn          = 14,
  DMA1_Channel5_IRQn          = 15,
  DMA1_Channel6_IRQn          = 16,
  DMA1_Channel7_IRQn          = 17,
  ADC1_2_IRQn                 = 18,
  USB_HP_CAN1_TX_IRQn         = 19,
  USB_LP_CAN1_RX0_IRQn        = 20,
  CAN1_RX1_IRQn               = 21,
  CAN1_SCE_IRQn               = 22,
  EXTI9_5_IRQn                = 23,
  TIM1_BRK_TIM15_IRQn         = 24,
  TIM1_UP_TIM16_IRQn          = 25,
  TIM1_TRG_COM_TIM17_IRQn     = 26,
  TIM1_CC_IRQn                = 27,
  TIM2_IRQn                   = 28,
  TIM3_IRQn                   = 29,
  TIM4_IRQn                   = 30,
  I2C1_EV_IRQn                = 31,
  I2C1_ER_IRQn                = 32,
  I2C2_EV_IRQn                = 33,
  I2C2_ER_IRQn                = 34,
  SPI1_IRQn                   = 35,
  SPI2_IRQn                   = 36,
  USART1_IRQn                 = 37,
  USART2_IRQn                 = 38,
  USART3_IRQn                 = 39,
  EXTI15_10_IRQn              = 40,
  RTC_Alarm_IRQn              = 41,
  USBWakeUp_IRQn              = 42,
  TIM8_BRK_IRQn               = 43,
  TIM8_UP_IRQn                = 44,
  TIM8_TRG_COM_IRQn           = 45,
  TIM8_CC_IRQn                = 46,
  ADC3_IRQn                   = 47,
  SPI3_IRQn                   = 51,
  UART4_IRQn                  = 52,
  UART5_IRQn                  = 53,
  TIM6_DAC_IRQn               = 54,
  TIM7_IRQn                   = 55,
  DMA2_Channel1_IRQn          = 56,
  DMA2_Channel2_IRQn          = 57,
  DMA2_Channel3_IRQn          = 58,
  DMA2_Channel4_IRQn          = 59,
  DMA2_Channel5_IRQn          = 60,
  ADC4_IRQn                   = 61,
  COMP1_2_3_IRQn              = 64,
  COMP4_5_6_IRQn              = 65,
  COMP7_IRQn                  = 66,
  USB_HP_IRQn                 = 74,
  USB_LP_IRQn                 = 75,
  USBWakeUp_RMP_IRQn          = 76,
  FPU_IRQn                    = 81
} IRQn_Type;
