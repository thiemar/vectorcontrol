/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file config.h
 *
 * bootloader definitions that configures the behavior and options
 * of the Boot loader
 * This file is relies on the parent folder's boot_config.h file and defines
 * different usages of the hardware for bootloading
 */


#pragma once

/************************************************************************************
 * Global platform definitions
 ************************************************************************************/

#define CONFIG_ARCH_CORTEXM4
#define CONFIG_ARCH_FPU
#define CONFIG_ARCH_IRQPRIO
#define CONFIG_ARCH_CHIP_STM32F446R
#define CONFIG_STM32_STM32F446
#define CONFIG_USEC_PER_TICK 1000
#define CONFIG_IDLETHREAD_STACKSIZE 8192
#define CONFIG_STM32_NOEXT_VECTORS
#define CONFIG_STM32_FLASH_PREFETCH
#define STM32_ADC1
#define STM32_ADC2
#define STM32_ADC3
#define PX4ESC_1_6


/************************************************************************************
 * Included Files
 ************************************************************************************/



#include <stm32.h>

#ifndef __ASSEMBLY__
#include <assert.h>
#include <string.h>
#endif

/************************************************************************************
* Pre-processor Definitions
************************************************************************************/

/* Clocking *************************************************************************/
/* The PX4ESC uses a 8MHz crystal connected to the HSE.
 *
 * This is the "standard" configuration as set up by arch/arm/src/stm32f40xx_rcc.c:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 4            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 180          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 2            (STM32_PLLCFG_PPQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Use PLLSA1M
 */

//TODO(Need to define and add the PLLSAIM );

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 24MHz
 * LSE - not installed
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 4) * 180
 *         = 360,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 360,000,000/ 2 = 180,000,000
 * USB OTG FS will use PLLSA1M
 *
 */
//

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(180)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define CONFIG_STM32_USE_ PLLSAI
#define CONFIG_STM32_USE_PLLI2S

/* Configure factors for  PLLSAI clock */

#define STM32_RCC_PLLSAICFGR_PLLSAIM    RCC_PLLSAICFGR_PLLSAIM(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(96)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR_PLLI2SDIVQ RCC_DCKCFGR_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR_PLLSAIDIVQ RCC_DCKCFGR_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR_SAI1SRC       RCC_DCKCFGR_SAI1SRC(RCC_SAI1CLKSRC_PLLSAI)
#define STM32_RCC_DCKCFGR_SAI2SRC       RCC_DCKCFGR_SAI2SRC(RCC_SAI2CLKSRC_PLLSAI)
#define STM32_RCC_DCKCFGR_TIMPRE        0
#define STM32_RCC_DCKCFGR_I2S1SRC       RCC_DCKCFGR_I2S1SRC(RCC_SAI1CLKSRC_PLLR)
#define STM32_RCC_DCKCFGR_I2S2SRC       RCC_DCKCFGR_I2S2SRC(RCC_SAI1CLKSRC_PLLR)



/* Configure factors for  PLLI2S clock */


#define STM32_RCC_PLLI2SCFGR_PLLI2SM   RCC_PLLI2SCFGR_PLLI2SM(16)
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_FMPI2C1SEL RCC_DCKCFGR2_FMPI2C1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSEL     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSEL   RCC_DCKCFGR2_CK48MSEL_PLLSAI
#define STM32_RCC_DCKCFGR2_SDIOCSEL      RCC_DCKCFGR2_SDIOCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SPDIFRXEL  RCC_DCKCFGR2_SPDIFRXEL_48MHZ

#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (180MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (45MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (90MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define STM32_TIM18_FREQUENCY   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM27_FREQUENCY   (2*STM32_PCLK1_FREQUENCY)

/* Leds *************************************************************************/

/* LED index values for use with board_setled() */

#define BOARD_LED1                0
#define BOARD_LED_RED             BOARD_LED1
#define BOARD_LED2                1
#define BOARD_LED_GREEN           BOARD_LED2
#define BOARD_LED3                2
#define BOARD_LED_BLUE            BOARD_LED3
#define BOARD_NLEDS               3

/* LED bits for use with board_setleds() */

#define BOARD_LED_RED_BIT     (1 << BOARD_LED_RED)
#define BOARD_LED_GREEN_BIT   (1 << BOARD_LED_GREEN)
#define BOARD_LED_BLUE_BIT    (1 << BOARD_LED_BLUE)

/* TODO:define these
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 *
 * defined.  In that case, the usage by the board port is as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                         Red   Green Blue
 *   ------------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED          0 /* NuttX has been started   OFF    OFF   OFF */
#define LED_HEAPALLOCATE     1 /* Heap has been allocated  OFF    OFF   ON  */
#define LED_IRQSENABLED      2 /* Interrupts enabled       OFF    ON    OFF */
#define LED_STACKCREATED     3 /* Idle stack created       OFF    ON    ON  */
#define LED_INIRQ            4 /* In an interrupt          N/C    GLOW  N/C */
#define LED_SIGNAL           5 /* In a signal handler      N/C    GLOW  N/C */
#define LED_ASSERTION        6 /* An assertion failed      GLOW   GLOW  N/C */
#define LED_PANIC            7 /* The system has crashed   Blk    OFF   N/C */
#define LED_IDLE             8  /* MCU is is sleep mode    ON     OFF   OFF */

/*
 * Thus if the blue is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 *
 */

/* Probes unused */
#define PROBE_INIT(mask)
#define PROBE(n,s)
#define PROBE_MARK(n)


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OPT_PREFERRED_NODE_ID ANY_NODE_ID

//todo:wrap OPT_x in in ifdefs for command line definitions
#define OPT_TBOOT_MS            2000
#define OPT_NODE_STATUS_RATE_MS 800
#define OPT_NODE_INFO_RATE_MS   50
#define OPT_BL_NUMBER_TIMERS    7

/*
 *  This Option set is set to 1 ensure a provider of firmware has an
 *  opportunity update the node's firmware.
 *  This Option is the default policy and can be overridden by
 *  a jumper
 *  When this Policy is set, the node will ignore tboot and
 *  wait indefinitely for a GetNodeInfo request before booting.
 *
 *  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT is used to allow
 *  the polarity of the jumper to be True Active
 *
 *  wait  OPT_WAIT_FOR_GETNODEINFO  OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO
 *                                                 Jumper
 *   yes           1                       0         x
 *   yes           1                       1       Active
 *   no            1                       1       Not Active
 *   no            0                       0         X
 *   yes           0                       1       Active
 *   no            0                       1       Not Active
 *
 */
#define OPT_WAIT_FOR_GETNODEINFO                    0
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO        1
#define OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO_INVERT 0

#define OPT_ENABLE_WD           1

#define OPT_RESTART_TIMEOUT_MS 20000u

/* Reserved for the Booloader */
#define OPT_BOOTLOADER_SIZE_IN_K (1024*16)

/* Reserved for the application out of the total
 * system flash minus the BOOTLOADER_SIZE_IN_K
 */
#define OPT_APPLICATION_RESERVER_IN_K    (32*1024) /* Parameters will use the 2nd, 3rd 16KiB sectors*/

#define OPT_APPLICATION_IMAGE_OFFSET    (OPT_BOOTLOADER_SIZE_IN_K + OPT_APPLICATION_RESERVER_IN_K)
#define OPT_APPLICATION_IMAGE_LENGTH    (FLASH_SIZE-(OPT_BOOTLOADER_SIZE_IN_K+OPT_APPLICATION_RESERVER_IN_K))

#define FLASH_BASE              STM32_FLASH_BASE
#define FLASH_NUMBER_PAGES      STM32_FLASH_NPAGES
#define FLASH_PAGE_SIZE         STM32_FLASH_PAGESIZE
#define FLASH_SIZE              ((4*16*1024) + (1*64*1024) + (3*128*1024))

/*
 *
512 KiB of flash:

* 16 KiB bootloader region (stubbed out in default image) at 0x08000000
* 32 KiB parameter area starting at 0x08004000
* Up to 464 KiB main application area starting at 0x0800C000
*/
#define FLASH_PARAM_ADDRESS (FLASH_BASE + OPT_BOOTLOADER_SIZE_IN_K)
#define FLASH_PARAM_LENGTH  OPT_APPLICATION_RESERVER_IN_K

#define APPLICATION_LOAD_ADDRESS (FLASH_BASE + OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_SIZE (FLASH_SIZE-OPT_APPLICATION_IMAGE_OFFSET)
#define APPLICATION_LAST_8BIT_ADDRRESS  ((uint8_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint8_t)))
#define APPLICATION_LAST_32BIT_ADDRRESS ((uint32_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint32_t)))
#define APPLICATION_LAST_64BIT_ADDRRESS ((uint64_t *)((APPLICATION_LOAD_ADDRESS+APPLICATION_SIZE)-sizeof(uint64_t)))

/* If this board uses big flash that have large sectors */

#if defined(CONFIG_STM32_FLASH_CONFIG_E) || \
   defined(CONFIG_STM32_FLASH_CONFIG_F) || \
   defined(CONFIG_STM32_FLASH_CONFIG_G) || \
   defined(CONFIG_STM32_FLASH_CONFIG_I)
#define OPT_USE_YIELD
#endif

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* High-resolution timer
 */
#define HRT_TIMER               1       /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1       /* use capture/compare channel */
#define HRT_PPM_CHANNEL         3       /* use capture/compare channel 3 */
#define GPIO_PPM_IN             (GPIO_ALT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN12)

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PA[03] PA3/TIM2_CH4/TIM5_CH4/TIM9_CH2/USART2_RX           17       RC_PWM
 *  PA[04] PA4/SPI1_NSS                                       20       OC_ADJ
 *  PA[05] PA5/TIM2_CH1/TIM2_ETR/TIM8_CH1                     21       EN_GATE
 *  PA[06] PA6/TIM1_BKIN/TIM3_CH1/TIM8_BKIN/SPI1_MISO         22       DC_CAL
 *
 */

#define GPIO_RC_PWM   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OC_ADJ   GPIO_DAC1_OUT
#define GPIO_EN_GATE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR)
#define GPIO_DC_CAL   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTA | GPIO_PIN6 | GPIO_OUTPUT_CLEAR)


/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PB[02] PB2/TIM2_CH4/SPI3_MOSI                             28       GAIN
 *  PB[03] PB3/TIM2_CH2/I2C2_SDA/SPI1_SCK                     55       TEST2
 *  PB[04] PB4/TIM3_CH1/I2C3_SDA/SPI1_MISO/SPI3_MISO          56       TEST3
 *
 */

#define GPIO_GAIN     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTB | GPIO_PIN2 | GPIO_OUTPUT_CLEAR)
#define GPIO_TEST2     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTB | GPIO_PIN3 | GPIO_OUTPUT_CLEAR)
#define GPIO_TEST3     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTB | GPIO_PIN4 | GPIO_OUTPUT_CLEAR)

/* CAN ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PB[05] PB5/TIM3_CH2/SPI1_MOSI/CAN2_RX                     57       CAN2_RX
 *  PB[06] PB6/TIM4_CH1/USART1_TX/CAN2_TX                     58       CAN2_TX
 *  PB[07] PB7/TIM2_CH2/TIM4_CH4/TIM11_CH1/I2C1_SDA           59       WAIT_GETNODEINFO
 *  PB[08] PB8/TIM4_CH3/I2C1_SCL/CAN1_RX                      61       CAN1_RX
 *  PB[09] PB9/TIM4_CH4/I2C1_SDA/CAN1_TX                      62       CAN1_TX
 *
 */

#define GPIO_WAIT_GETNODEINFO   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTB | GPIO_PIN7)

/* UART3 ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PB[10] PB10/TIM2_CH3/I2C2_SCL/SPI2_SCK/I2S2_CK/USART3_TX  29       DBG_TX
 *  PC[05] PC5/USART3_RX/SPDIFRX_IN3/FMC_SDCKE0               25       DBG_RX
 *
 */


/* Analog ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[00] PC0/ADC123_IN10                                   8       TEMP_SENS
 *  PC[01] PC1/ADC123_IN11/SPI3_MOSI/SPI2_MOSI               9       VBAT_SENS
 *  PC[02] PC2/ADC123_IN12/SPI2_MISO,                       10       CURR_SENS2
 *  PC[03] PC3/ADC123_IN13/SPI2_MOSI                        11       CURR_SENS1
 *
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[06] TIM3_CH1/TIM8_CH1/USART6_TX                      37         RPM
 *
 */

#define GPIO_RPM   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)

/* LEDs ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[07] PC7/TIM3_CH2/TIM8_CH2                            38       LED_RED
 *  PC[08] PC8/TIM3_CH3/TIM8_CH3                            39       LED_GREEN
 *  PC[09] PC9/TIM3_CH4/TIM8_CH4                            40       LED_BLUE
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTC | GPIO_PIN7 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_RED    GPIO_LED1

#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTC | GPIO_PIN8 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_GREEN  GPIO_LED2

#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTC | GPIO_PIN9 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_BLUE  GPIO_LED3


/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[10] PC10/SPI3_SCK/USART3_TX/UART4_TX/                51       HWID0
 *  PC[11] PC11/SPI3_MISO/USART3_RX/UART4_RX                52       HWID1
 *  PC[12] PC12/I2C2_SDA/SPI3_MOSI/USART3_CK/UART5_TX       53       TEST4
 *  PC[13] PC13/TAMP_1/WKUP1                                 2       PWRGD
 *  PC[14] PC14/OSC32_IN                                     3       OCTW
 *  PC[15] PC15/OSC32_OUT                                    4       FAULT
 *
 */

#define GPIO_HWID0   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN10)
#define GPIO_HWID1   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN11)
#define GPIO_TEST4   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTC | GPIO_PIN12 | GPIO_OUTPUT_CLEAR)

#define GPIO_PWRGD   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN13)
#define GPIO_OCTW    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN14)
#define GPIO_FAULT   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN15)

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PD[02] PD2/TIM3_ETR/UART5_RX/SDIO_CMD                   54       TEST1
 */
#define GPIO_TEST1   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_PORTD | GPIO_PIN2 | GPIO_OUTPUT_CLEAR)

/************************************************************************************
 * Parameter estimation configuration
 ************************************************************************************/

#define PE_TEST_CYCLES 64.0f
#define PE_START_FREQ_HZ 2812.5f
#define PE_MIN_V_V float(1.0/64.0)
#define PE_START_V_V float(1.0/4.0)
#define PE_MAX_V_V float(2.0/1.0)
#define PE_MIN_I_A float(1.0/1.0)
#define PE_MAX_I_A float(4.0)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

inline static void board_initialize(void)
{
    putreg32(getreg32(STM32_RCC_APB1ENR) | RCC_APB1ENR_CAN1EN |
             RCC_APB1ENR_DACEN,
             STM32_RCC_APB1ENR);
    putreg32(getreg32(STM32_RCC_APB2ENR) | RCC_APB2ENR_TIM1EN |
             RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN,
             STM32_RCC_APB2ENR);

    putreg32(getreg32(STM32_RCC_APB1RSTR) | RCC_APB1RSTR_CAN1RST,
             STM32_RCC_APB1RSTR);
    putreg32(getreg32(STM32_RCC_APB1RSTR) & ~RCC_APB1RSTR_CAN1RST,
             STM32_RCC_APB1RSTR);

    stm32_configgpio(GPIO_CAN1_RX_2);
    stm32_configgpio(GPIO_CAN1_TX_2);

    /* Phase A, B and C PWM */
    stm32_configgpio(GPIO_TIM1_CH1OUT_1);
    stm32_configgpio(GPIO_TIM1_CH2OUT_1);
    stm32_configgpio(GPIO_TIM1_CH3OUT_1);
    stm32_configgpio(GPIO_TIM1_CH1N_1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL);
    stm32_configgpio(GPIO_TIM1_CH2N_1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL);
    stm32_configgpio(GPIO_TIM1_CH3N_1 | GPIO_SPEED_50MHz | GPIO_PUSHPULL);

    /* Phase A, B, C voltage */
    stm32_configgpio(GPIO_ADC1_IN0);
    stm32_configgpio(GPIO_ADC2_IN1);
    stm32_configgpio(GPIO_ADC3_IN2);

    /* Phase A, B current; Vbus */
    stm32_configgpio(GPIO_ADC1_IN13); /* PC3 = IA */
    stm32_configgpio(GPIO_ADC2_IN12); /* PC2 = IB */
    stm32_configgpio(GPIO_ADC3_IN11); /* PC1 = Vbus */

    /* Temperature sensor */
    stm32_configgpio(GPIO_ADC3_IN10); /* PC0 = Temp */

    /* Inputs */
    stm32_configgpio(GPIO_PWRGD);
    stm32_configgpio(GPIO_OCTW);
    stm32_configgpio(GPIO_FAULT);

    /* Outputs */
    stm32_configgpio(GPIO_OC_ADJ);
    stm32_configgpio(GPIO_EN_GATE);
    stm32_configgpio(GPIO_DC_CAL);
    stm32_configgpio(GPIO_GAIN);
}

/************************************************************************************
 * Name: board_deinitialize
 *
 * Description:
 *   This function is called by the bootloader code prior to booting
 *   the application. Is should place the HW into an benign initialized state.
 *
 ************************************************************************************/

inline static void board_deinitialize(void)
{
}

/****************************************************************************
 * Name: board_get_product_name
 *
 * Description:
 *   Called to retrieve the product name. The returned value is a assumed
 *   to be written to a pascal style string that will be length prefixed
 *   and not null terminated
 *
 * Input Parameters:
 *    product_name - A pointer to a buffer to write the name.
 *    maxlen       - The maximum number of charter that can be written
 *
 * Returned Value:
 *   The length of characters written to the buffer.
 *
 ****************************************************************************/

inline static uint8_t board_get_product_name(uint8_t *product_name, size_t maxlen)
{
    assert(maxlen > sizeof(HW_UAVCAN_NAME)-1);
    memcpy(product_name, HW_UAVCAN_NAME, sizeof(HW_UAVCAN_NAME)-1);
    return sizeof(HW_UAVCAN_NAME)-1;
}

/****************************************************************************
 * Name: board_get_hardware_version
 *
 * Description:
 *   Called to retrieve the hardware version information.
 *
 * Input Parameters:
 *    major - A pointer to the major hardware version field.
 *    minor - A pointer to the minor hardware version field.
 *    unique_id - A pointer to the 16-byte unique ID field
 *    coa_length - A pointer to the 8-bit length field for the COA
 *    coa - A pointer to the COA field, with a maximum length of 255 bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

inline static void board_get_hardware_version(uint8_t *major, uint8_t *minor,
    uint8_t unique_id[16], uint8_t *coa_length, uint8_t *coa)
{
    *major = HW_VERSION_MAJOR;
    *minor = HW_VERSION_MINOR;
    *coa_length = 0u;

    memset(unique_id, 0, 16u);
    memcpy(unique_id, (void *)STM32_SYSMEM_UID, 12u);
}

/****************************************************************************
 * Name: board_indicate
 *
 * Description:
 *   Provides User feedback to indicate the state of the bootloader
 *   on board specific  hardware.
 *
 * Input Parameters:
 *    indication - A member of the uiindication_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

typedef enum {
    off,
    reset,
    autobaud_start,
    autobaud_end,
    allocation_start,
    allocation_end,
    fw_update_start,
    fw_update_erase_fail,
    fw_update_invalid_response,
    fw_update_timeout,
    fw_update_invalid_crc,
    jump_to_app,
} uiindication_t;

inline static void board_indicate(uiindication_t indication)
{
}

/****************************************************************************
 * Name: board_should_wait_for_getnodeinfo
 *
 * Description:
 *   Returns 1 if the board should wait for a GetNodeInfo request before
 *   booting, or 0 if boot should proceed anyway.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *    1 if the board should wait for GetNodeInfo, 0 otherwise
 *
 ****************************************************************************/

inline static uint8_t board_should_wait_for_getnodeinfo(void)
{
    return 1u;
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

