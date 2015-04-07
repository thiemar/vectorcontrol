/*
Copyright (c) 2014 - 2015 by Thiemar Pty Ltd

This file is part of vectorcontrol.

vectorcontrol is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

vectorcontrol is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
vectorcontrol. If not, see <http://www.gnu.org/licenses/>.
*/

#include <cstdlib>
#include <cstdint>
#include <cassert>
#include "stm32f30x.h"
#include "hal.h"
#include "svm.h"
#include "perf.h"


/*
ESC revision 2 board pin definitions - STM32F302

PIN   PORT   NUMBER    FUNCTION
  1      -        -    VDD1
  2      F        0    OSC_IN
  3      F        1    -
  4      -        -    NRST - debug access
  5      -        -    VDDA
  6      -        -    VSSA
  7      A        0    ADC1_IN1 - Phase A current sense
  8      A        1    ADC1_IN2 - Phase B current sense
  9      A        2    ADC1_IN3 - Phase C current sense
 10      A        3    ADC1_IN4 - VSENSE
 11      A        4    ADC1_IN5 - TSENSE
 12      A        5    -
 13      A        6    -
 14      A        7    -
 15      B        0    -
 16      -        -    VSS2
 17      -        -    VDD2
 18      A        8    TIM1_CH1 - Phase A PWM
 19      A        9    TIM1_CH2 - Phase B PWM
 20      A       10    TIM1_CH3 - Phase C PWM
 21      A       11    CAN_RX
 22      A       12    CAN_TX
 23      A       13    SWDIO - debug access
 24      A       14    SWCLK - debug access
 25      A       15    -
 26      B        3    TRACESWO - debug access
 27      B        4    -
 28      B        5    -
 29      B        6    /CAN_SILENT
 30      B        7    -
 31      -        -    BOOT0
 32      -        -    VSS1
*/

/* These two are externally useful */
extern const uint32_t hal_core_frequency_hz = 72000000u;
extern const uint32_t hal_pwm_frequency_hz = 62500u;

const uint32_t hal_adc_frequency_hz = hal_core_frequency_hz;

const uint32_t hal_pwm_period_ticks = hal_core_frequency_hz /
                                      hal_pwm_frequency_hz;
const uint32_t hal_pwm_half_period_ticks = hal_pwm_period_ticks / 2;
const uint32_t hal_pwm_quarter_period_ticks = hal_pwm_period_ticks / 4;


/*
Set the TIM1 update event rate based on the selected PWM frequency. Must
always be odd; 1 results in an update event every PWM cycle, and 3 results
in an update event every 2 PWM cycles.
*/
const uint32_t hal_pwm_control_rate_div =
    hal_pwm_frequency_hz < 40000 ? 1u : 3u;


/* Work out the ADC sampling time in nanoseconds */
const float hal_adc_sample_periods = 7.5f;
const float hal_adc_sample_time_ns =
    1e3f * (3.0f * hal_adc_sample_periods + 2.0f * 13.5f) /
    hal_adc_frequency_hz;
const float hal_adc_shunt_settling_time_ns = 200.0f;
const uint32_t hal_adc_sample_time_ticks =
    (uint32_t)(hal_adc_sample_time_ns * hal_core_frequency_hz * 1e-9f + 0.5f);
const uint32_t hal_adc_settling_time_ticks =
    (uint32_t)(hal_adc_shunt_settling_time_ns *
               hal_core_frequency_hz * 1e-9f + 0.5f);


/* Board parameters */
const float hal_nominal_mcu_vdd_v = 3.3f;
const float hal_current_sense_gain_v_per_v = 20.0f;
const float hal_current_sense_r = 0.001f;
const float hal_vbus_gain_v_per_v = 1.0f / (20.0f + 1.0f);
const uint32_t hal_adc_full_scale_lsb = 1u << 12u;
const float hal_adc_v_per_lsb =
    hal_nominal_mcu_vdd_v / (float)hal_adc_full_scale_lsb;


/* This is a signed quantity so can go +/- 1.65 V of the 1.65 V reference */
extern const float hal_full_scale_current_a =
    0.5f * hal_adc_v_per_lsb * (float)hal_adc_full_scale_lsb /
    (hal_current_sense_r * hal_current_sense_gain_v_per_v);
/* Vbus input is 0-3.3 V */
extern const float hal_full_scale_voltage_v =
    hal_adc_v_per_lsb * (float)hal_adc_full_scale_lsb / hal_vbus_gain_v_per_v;

/* Control timestemp in seconds */
extern const float hal_control_t_s =
    (float)((1 + hal_pwm_control_rate_div) / 2) / (float)hal_pwm_frequency_hz;


#define HAL_ADC_PHASE_A_CHANNEL 1u
#define HAL_ADC_PHASE_B_CHANNEL 2u
#define HAL_ADC_PHASE_C_CHANNEL 3u
#define HAL_ADC_VBUS_CHANNEL 4u


/* CAN bit rate in bits per second */
#define HAL_CAN_RATE_BPS 1000000u


/*
Minimum number of core cycles for a pulse on CAN RX to be considered a PWM
pulse, rather than part of a CAN message -- corresponds to 10 us.
*/
#define HAL_PWM_MIN_CYCLES 720u


/* Interrupt priorities */
#define ADC_PRE_EMPTION_PRIORITY 1u
#define ADC_SUB_PRIORITY 0u

#define CAN_RX_PRE_EMPTION_PRIORITY 2u
#define CAN_RX_SUB_PRIORITY 0u

#define SYSTICK_PRE_EMPTION_PRIORITY 3u
#define SYSTICK_SUB_PRIORITY 0u

/* Do not modify (NVIC_PriorityGroup_3 is assumed to be set) */
#define SYSTICK_PRIORITY (((SYSTICK_PRE_EMPTION_PRIORITY & 0x07) << 1) | \
                          (SYSTICK_SUB_PRIORITY & 0x01))


/*
Phase A/B/C MOSFET driver switch from TIM1. Output, alternate function |
tri-state depending on enable mode.
H = top switch on
L = bottom switch on
Z = both switches off
*/
#define PORT_PHASE_SW GPIOA
static GPIO_InitTypeDef PIN_PHASE_SW_EN = {
    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10,
    GPIO_Mode_AF,
    GPIO_Speed_Level_3,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};
static GPIO_InitTypeDef PIN_PHASE_SW_DIS = {
    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10,
    GPIO_Mode_IN,
    GPIO_Speed_Level_3,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};
static GPIO_InitTypeDef PIN_PHASE_SW_LOW = {
    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10,
    GPIO_Mode_OUT,
    GPIO_Speed_Level_3,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* ADC inputs, analog, 0–3.3 V. */
#define PORT_ADC GPIOA
static GPIO_InitTypeDef PIN_ADC = {
    GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3,
    GPIO_Mode_AN,
    GPIO_Speed_Level_3,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* DAC output for ADC voltage reference, 1.65 V */
#define PORT_DAC GPIOA
static GPIO_InitTypeDef PIN_DAC = {
    GPIO_Pin_4,
    GPIO_Mode_AN,
    GPIO_Speed_Level_3,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* CAN RX from transceiver. Input, digital. */
#define PORT_CAN_RX GPIOA
static GPIO_InitTypeDef PIN_CAN_RX = {
    GPIO_Pin_11,
    GPIO_Mode_AF,
    GPIO_Speed_Level_2,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* CAN TX to transceiver. Output, alternate function. */
#define PORT_CAN_TX GPIOA
static GPIO_InitTypeDef PIN_CAN_TX = {
    GPIO_Pin_12,
    GPIO_Mode_AF,
    GPIO_Speed_Level_2,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/*
CAN SILENT signal to transceiver. Output, push-pull.
H = CAN transceiver is in silent mode (forced recessive)
L = CAN transceiver is in normal mode
*/
#define PORT_CAN_SILENT GPIOB
static GPIO_InitTypeDef PIN_CAN_SILENT = {
    GPIO_Pin_6,
    GPIO_Mode_OUT,
    GPIO_Speed_Level_1,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* PWM input -- tied to CAN RX but times pulse widths to detect PWM signals */
#define PORT_PWM_INPUT GPIOA
static GPIO_InitTypeDef PIN_PWM_INPUT = {
    GPIO_Pin_15,
    GPIO_Mode_AF,
    GPIO_Speed_Level_1,
    GPIO_OType_PP,
    GPIO_PuPd_NOPULL
};


/* Hardware states */
static volatile enum hal_pwm_state_t pwm_state_;


/* Low-frequency (SysTick) control task callback */
static volatile hal_callback_t low_frequency_task_;
/* High-frequency (ADC update) control task callback */
static volatile hal_control_callback_t high_frequency_task_;
/* CAN message RX callback */
static volatile hal_can_callback_t can_receive_task_;
/* PWM pulse callback */
static volatile hal_pwm_callback_t pwm_receive_task_;


/* ADC conversion result destination */
static volatile uint16_t adc_conversion_results_[2];


/* Vbus in raw ADC units (1/LSB) */
static uint32_t board_vbus_lsb_;
static volatile float vbus_v_;
static volatile float vbus_inv_;


/* Board temperature in raw ADC units (1/LSB) */
static uint32_t board_temp_lsb_;
static volatile float temp_degc_;


extern "C" void SysTick_Handler(void);
extern "C" void ADC1_2_IRQHandler(void);
extern "C" void USB_LP_CAN1_RX0_IRQHandler(void);
extern "C" void CAN1_RX1_IRQHandler(void);
extern "C" void Fault_Handler(void);


static void hal_init_sys_();
static void hal_init_io_();
static void hal_init_tim1_();
static void hal_init_adc_();
static void hal_init_dma_();
static void hal_init_can_();
static void hal_run_calibration_();
static bool hal_read_last_vbus_temp_();


inline void __attribute__((optimize("O3")))
hal_read_phase_shunts_(
    int16_t phase_shunt_signal_lsb[3],
    uint8_t phase_pwm_sector
) {
    /* Direct reading of phase A current */
    phase_shunt_signal_lsb[0] = (int16_t)ADC1->JDR1;

    /* Direct reading of phase B current */
    phase_shunt_signal_lsb[1] = (int16_t)ADC1->JDR2;

    /* Direct reading of phase C current */
    phase_shunt_signal_lsb[2] = (int16_t)ADC1->JDR3;

    switch (phase_pwm_sector) {
        case 4:
        case 5:
            /*
            A and B are observable, no need to reconstruct C as it's not used
            */
            break;

        case 6:
        case 1:
            /*
            Phase A current is not observable -- reconstruct:
            IA = -IC - IB
            */
            phase_shunt_signal_lsb[0] = (int16_t)(-phase_shunt_signal_lsb[2] -
                                                   phase_shunt_signal_lsb[1]);
            break;

        case 2:
        case 3:
            /*
            Phase B current is not observable -- reconstruct:
            IB = -IC - IA
            */
            phase_shunt_signal_lsb[1] = (int16_t)(-phase_shunt_signal_lsb[2] -
                                                   phase_shunt_signal_lsb[0]);
            break;

        default:
            break;
    }
}


inline void __attribute__((optimize("O3")))
hal_update_timer_(
    uint8_t phase_pwm_sector,
    const uint16_t phase_on_ticks[3]
) {
    const uint8_t sector_phases[6][3] = {
        /* primary, secondary, tertiary */
        { 0, 1, 2 },
        { 1, 0, 2 },
        { 1, 2, 0 },
        { 2, 1, 0 },
        { 2, 0, 1 },
        { 0, 2, 1 }
    };
    uint16_t sample_ticks, duty_delta_ticks, phase1_ticks, phase2_ticks,
             period_ticks;

    phase1_ticks = phase_on_ticks[sector_phases[phase_pwm_sector - 1][0]];
    phase2_ticks = phase_on_ticks[sector_phases[phase_pwm_sector - 1][1]];
    period_ticks = hal_pwm_half_period_ticks - 1u;

    /* Polarity of CC4 is active high */
    TIM1->CCER &= 0xDFFFu;

    if ((uint16_t)(period_ticks - phase1_ticks) >
            hal_adc_settling_time_ticks) {
        sample_ticks = (uint16_t)(period_ticks - 1u);
    } else {
        duty_delta_ticks = (uint16_t)(phase1_ticks - phase2_ticks);
        sample_ticks = (uint16_t)phase1_ticks;

        /* Check which side of the crossing point we should sample */
        if (duty_delta_ticks > (uint16_t)(period_ticks - phase1_ticks) * 2u) {
            sample_ticks =
                (uint16_t)(sample_ticks - hal_adc_sample_time_ticks);
        } else {
            sample_ticks =
                (uint16_t)(sample_ticks + hal_adc_settling_time_ticks);

            if (sample_ticks >= period_ticks) {
                /* Make polarity of CC4 active low */
                TIM1->CCER |= 0x2000u;

                sample_ticks =
                    (uint16_t)((2u * period_ticks) - sample_ticks - 1u);
            }
        }
    }

    /*
    Update the on times for the PWM channels as well as the ADC trigger point
    */
    TIM1->CCR1 = phase_on_ticks[0];
    TIM1->CCR2 = phase_on_ticks[1];
    TIM1->CCR3 = phase_on_ticks[2];
    TIM1->CCR4 = sample_ticks;
}


void
__attribute__((noreturn))
__esc_assert_func (
    const char __attribute__((unused)) *file,
    int __attribute__((unused)) line,
    const char __attribute__((unused)) *func,
    const char __attribute__((unused)) *failedexpr
) {
    hal_set_pwm_state(HAL_PWM_STATE_OFF);
    abort();
}


void Fault_Handler(void) {
    esc_assert(false);
}


void NMIVector(void) __attribute__((alias("Fault_Handler")));
void HardFaultVector(void) __attribute__((alias("Fault_Handler")));
void MemManageVector(void) __attribute__((alias("Fault_Handler")));
void BusFaultVector(void) __attribute__((alias("Fault_Handler")));
void UsageFaultVector(void) __attribute__((alias("Fault_Handler")));


void SysTick_Handler(void) {
    /*
    Get the latest bus voltage or temperature, depending on what was requested
    */
    hal_read_last_vbus_temp_();

    /* If we got a PWM pulse, call the PWM receive callback. */
    if (TIM2->SR & TIM_SR_CC2IF) {
        if (pwm_receive_task_) {
            pwm_receive_task_(TIM2->CCR2, TIM2->CCR1);
        }

        TIM2->SR &= ~(TIM_SR_CC2IF | TIM_SR_CC1IF);
    }

    /* Run the user task, if defined */
    if (low_frequency_task_) {
        low_frequency_task_();
    }
}


void __attribute__((optimize("O3")))
ADC1_2_IRQHandler(void) {
PERF_COUNT_START
    static float last_v_ab[2];
    static float prev_v_ab[2];
    static uint8_t last_pwm_sector = 1u;
    static uint8_t prev_pwm_sector = 1u;

    int16_t phase_current_lsb[3];
    uint16_t phase_oc[3];
    float out_v_ab[2], i_ab[2];
    float temp;

    /* Clear TIM1 update flag so failure to meet deadline can be detected */
    TIM1->SR &= ~(uint16_t)TIM_FLAG_Update;

    hal_read_phase_shunts_(phase_current_lsb, prev_pwm_sector);

    /*
    Clarke transformation for balanced systems

    i_alpha = i_a,
    i_beta = (2 * i_b + i_a) / sqrt(3)

    Multiply by 8 because the phase current readings are right-aligned.
    */
    i_ab[0] = (float)phase_current_lsb[0] *
              (hal_full_scale_current_a * 8.0f / 32768.0f);
    temp = (float)phase_current_lsb[1] *
           (hal_full_scale_current_a * 8.0f / 32768.0f);
    i_ab[1] = (0.57735026919f * i_ab[0] + 1.15470053838f * temp);

    if (high_frequency_task_) {
        high_frequency_task_(out_v_ab, prev_v_ab, i_ab, vbus_v_);
    } else {
        out_v_ab[0] = out_v_ab[1] = 0.0f;
    }

    prev_v_ab[0] = last_v_ab[0];
    prev_v_ab[1] = last_v_ab[1];

    last_v_ab[0] = out_v_ab[0];
    last_v_ab[1] = out_v_ab[1];

    prev_pwm_sector = last_pwm_sector;

    /*
    Convert alpha-beta frame voltage fractions to SVM output compare values
    for each phase.
    */
    last_pwm_sector = svm_duty_cycle_from_v_alpha_beta(
        phase_oc,
        (int16_t)__SSAT((int32_t)(vbus_inv_ * out_v_ab[0]), 16),
        (int16_t)__SSAT((int32_t)(vbus_inv_ * out_v_ab[1]), 16),
        hal_pwm_period_ticks);

    /* Update the timer */
    hal_update_timer_(last_pwm_sector, phase_oc);

    /*
    Clear the JEOS event, and prepare for the next hardware trigger
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOS);
    */
    ADC1->ISR = ADC_FLAG_JEOS;
    /*
    Allow the next ADC conversion to happen based on the TIM1 CC4 event
    ADC_StartInjectedConversion(ADC1);
    */
    ADC1->CR |= ADC_CR_JADSTART;

    /* FIXME -- check return code for task deadline miss */
PERF_COUNT_END
}


void USB_LP_CAN1_RX0_IRQHandler(void) {
    uint8_t num_messages, i;
    CANMessage message;
    CanRxMsg rx_message;

    num_messages = CAN_MessagePending(CAN1, 0);
    for (i = 0; i < num_messages; i++) {
        CAN_Receive(CAN1, 0, &rx_message);
        message.set_id(rx_message.IDE == CAN_Id_Standard ?
                       rx_message.StdId : rx_message.ExtId);
        message.set_data(rx_message.DLC, rx_message.Data);
        if (can_receive_task_) {
            can_receive_task_(message);
        }
    }
}


void CAN1_RX1_IRQHandler(void) {
    uint8_t num_messages, i;
    CANMessage message;
    CanRxMsg rx_message;

    num_messages = CAN_MessagePending(CAN1, 1);
    for (i = 0; i < num_messages; i++) {
        CAN_Receive(CAN1, 1, &rx_message);
        message.set_id(rx_message.IDE == CAN_Id_Standard ?
                       rx_message.StdId : rx_message.ExtId);
        message.set_data(rx_message.DLC, rx_message.Data);
            if (can_receive_task_) {
            can_receive_task_(message);
        }
    }
}


static void hal_init_sys_() {
    /* Enable the required peripherals */
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div1);

    /* AHB */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* APB2 peripherals (high-speed) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* APB1 peripherals (low-speed) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* Configure the interrupt controller */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    /* Enable DWT count-down */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


static void hal_init_io_() {
    /* Reset I/O ports */
    GPIO_DeInit(GPIOA);
    GPIO_DeInit(GPIOB);

    /* Turn CAN to silent to avoid glitches */
    GPIO_SetBits(PORT_CAN_SILENT, (uint16_t)PIN_CAN_SILENT.GPIO_Pin);

    /*
    Don't lock GPIO config for these as we need to switch between tri-state
    and alternate function push-pull depending on drive status.
    */
    GPIO_Init(PORT_PHASE_SW, &PIN_PHASE_SW_DIS);

    /* Configure TIM1 alternate function remaps for PWM out */
    GPIO_PinAFConfig(PORT_PHASE_SW, GPIO_PinSource8, GPIO_AF_6);
    GPIO_PinAFConfig(PORT_PHASE_SW, GPIO_PinSource9, GPIO_AF_6);
    GPIO_PinAFConfig(PORT_PHASE_SW, GPIO_PinSource10, GPIO_AF_6);

    GPIO_Init(PORT_ADC, &PIN_ADC);
    GPIO_PinLockConfig(PORT_ADC, (uint16_t)PIN_ADC.GPIO_Pin);

    GPIO_Init(PORT_DAC, &PIN_DAC);
    GPIO_PinLockConfig(PORT_DAC, (uint16_t)PIN_DAC.GPIO_Pin);

    GPIO_Init(PORT_CAN_SILENT, &PIN_CAN_SILENT);
    GPIO_PinLockConfig(PORT_CAN_SILENT, (uint16_t)PIN_CAN_SILENT.GPIO_Pin);

    /* Configure CAN alternate function */
    GPIO_PinAFConfig(PORT_CAN_RX, GPIO_PinSource11, GPIO_AF_9);
    GPIO_PinAFConfig(PORT_CAN_TX, GPIO_PinSource12, GPIO_AF_9);

    GPIO_Init(PORT_CAN_TX, &PIN_CAN_TX);
    GPIO_Init(PORT_CAN_RX, &PIN_CAN_RX);
    GPIO_PinLockConfig(PORT_CAN_TX, (uint16_t)(PIN_CAN_TX.GPIO_Pin |
                                               PIN_CAN_RX.GPIO_Pin));

    /* Turn CAN on */
    GPIO_ResetBits(PORT_CAN_SILENT, (uint16_t)PIN_CAN_SILENT.GPIO_Pin);

    /* Configure PWM alternate function -- TIM2_CH1 */
    GPIO_PinAFConfig(PORT_PWM_INPUT, GPIO_PinSource15, GPIO_AF_1);
    GPIO_Init(PORT_PWM_INPUT, &PIN_PWM_INPUT);
    GPIO_PinLockConfig(PORT_PWM_INPUT, (uint16_t)PIN_PWM_INPUT.GPIO_Pin);
}


static void hal_init_tim1_() {
    TIM_TimeBaseInitTypeDef timebase_config = {
        .TIM_Prescaler = 0,
        .TIM_CounterMode = TIM_CounterMode_CenterAligned1,
        .TIM_Period = (uint16_t)(hal_pwm_half_period_ticks - 1u),
        .TIM_ClockDivision = TIM_CKD_DIV2,
        .TIM_RepetitionCounter = hal_pwm_control_rate_div
    };
    TIM_OCInitTypeDef oc_config = {
        .TIM_OCMode = TIM_OCMode_PWM1,
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_OutputNState = TIM_OutputNState_Disable,
        .TIM_Pulse = (uint16_t)hal_pwm_quarter_period_ticks,
        .TIM_OCPolarity = TIM_OCPolarity_High,
        .TIM_OCNPolarity = TIM_OCNPolarity_High,
        .TIM_OCIdleState = TIM_OCIdleState_Reset,
        .TIM_OCNIdleState = TIM_OCNIdleState_Reset
    };
    TIM_BDTRInitTypeDef bdtr_config = {
        .TIM_OSSRState = TIM_OSSRState_Enable,
        .TIM_OSSIState = TIM_OSSIState_Enable,
        .TIM_LOCKLevel = TIM_LOCKLevel_1,
        .TIM_DeadTime = 0u,
        .TIM_Break = TIM_Break_Disable,
        .TIM_BreakPolarity = TIM_BreakPolarity_High,
        .TIM_AutomaticOutput = TIM_AutomaticOutput_Disable
    };

    /* Reset TIM1 */
    TIM_DeInit(TIM1);

    /* Configure the timebase and OC channels */
    TIM_TimeBaseInit(TIM1, &timebase_config);

    /* OC1-3 are PWM channels */
    TIM_OC1Init(TIM1, &oc_config);
    TIM_OC2Init(TIM1, &oc_config);
    TIM_OC3Init(TIM1, &oc_config);

    /*
    OC4 is the ADC conversion timer, so configuration is a little different
    */
    oc_config.TIM_OCMode = TIM_OCMode_PWM2;
    oc_config.TIM_Pulse = (uint16_t)(hal_pwm_half_period_ticks - 5u);
    TIM_OC4Init(TIM1, &oc_config);

    /* Enable preload */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* Dead Time */
    TIM_BDTRConfig(TIM1, &bdtr_config);

    /* Output trigger is update event */
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    /*
    Enable TIM1 and resynchronize so update event happens during underflow
    */
    TIM_Cmd(TIM1, ENABLE);
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);

    /* Stop TIM1 when execution is paused */
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
}


static void hal_init_pwm_() {
    TIM_TimeBaseInitTypeDef tim2_config = {
        .TIM_Prescaler = 72u - 1u, /* 1 us resolution -- TIM2 is on APB1,
                                      which runs at 36 MHz */
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 65535u, /* 0.065 s maximum period */
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0
    };
    TIM_ICInitTypeDef capture_config = {
        .TIM_Channel = TIM_Channel_1,
        .TIM_ICPolarity = TIM_ICPolarity_Falling,
        .TIM_ICSelection = TIM_ICSelection_DirectTI,
        .TIM_ICPrescaler = TIM_ICPSC_DIV1,
        .TIM_ICFilter = 0
    };

    TIM_DeInit(TIM2);
    TIM_TimeBaseInit(TIM2, &tim2_config);

    TIM_SelectInputTrigger(TIM2, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);
    TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

    TIM_PWMIConfig(TIM2, &capture_config);

    TIM2->SR &= ~(TIM_SR_CC2IF | TIM_SR_CC1IF);

    TIM_Cmd(TIM2, ENABLE);
}


static void hal_init_adc_() {
    DAC_InitTypeDef dac_config = {
        .DAC_Trigger = DAC_Trigger_None,
        .DAC_WaveGeneration = DAC_WaveGeneration_None,
        .DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0,
        .DAC_OutputBuffer = DAC_OutputBuffer_Disable
    };
    ADC_InitTypeDef config = {
        .ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable,
        .ADC_Resolution = ADC_Resolution_12b,
        .ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0,
        .ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None,
        .ADC_DataAlign = ADC_DataAlign_Right,
        .ADC_OverrunMode = ADC_OverrunMode_Disable,
        .ADC_AutoInjMode = ADC_AutoInjec_Disable,
        .ADC_NbrOfRegChannel = 2u
    };
    ADC_CommonInitTypeDef common_config = {
        .ADC_Mode = ADC_Mode_Independent,
        .ADC_Clock = ADC_Clock_SynClkModeDiv1,
        .ADC_DMAAccessMode = ADC_DMAAccessMode_1,
        .ADC_DMAMode = ADC_DMAMode_OneShot,
        .ADC_TwoSamplingDelay = 0u
    };

    /* Init DAC for the current sense voltage reference (1.65 V) */
    DAC_DeInit();
    DAC_Init(DAC_Channel_1, &dac_config);
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_SetChannel1Data(DAC_Align_12b_R, 0x800u);

    /* Init ADC1 */
    ADC_DeInit(ADC1);
    ADC_VoltageRegulatorCmd(ADC1, ENABLE);

    /* Worst-case regulator delay is 10 us */
    for (volatile uint32_t x = 0;
         x < 10 * hal_core_frequency_hz / 1000000; x++);

    ADC_CommonInit(ADC1, &common_config);
    ADC_Init(ADC1, &config);

    /* Calibrate the ADCs */
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    /*
    Delay after calibration done -- see STM32F302x6/x8 Silicon Limitations
    (DM00109012) item 2.2.4.
    */
    for (volatile uint32_t x = 0; x < 4; x++);

    /* Enable the ADC and temperature sensor */
    ADC_Cmd(ADC1, ENABLE);
    ADC_TempSensorCmd(ADC1, ENABLE);

    while (!(ADC1_2->CSR & ADC12_CSR_ADRDY_MST));

    /* Configure VBUS and temperature sense channels */
    ADC_RegularChannelConfig(ADC1, HAL_ADC_VBUS_CHANNEL, 1u,
                             ADC_SampleTime_61Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 2u,
                             ADC_SampleTime_181Cycles5);
}


static void hal_init_dma_() {
    DMA_InitTypeDef ch1_config = {
        .DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR),
        .DMA_MemoryBaseAddr = (uint32_t)(&adc_conversion_results_),
        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_BufferSize = 2u,
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
        .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
        .DMA_Mode = DMA_Mode_Circular,
        .DMA_Priority = DMA_Priority_Low,
        .DMA_M2M = DMA_M2M_Disable
    };
    NVIC_InitTypeDef nvic_config = {
        .NVIC_IRQChannel = (uint8_t)ADC1_2_IRQn,
        .NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY,
        .NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY,
        .NVIC_IRQChannelCmd = ENABLE
    };

    /*
    DMA channel 1 (temperature and VBUS measurement) is linked to ADC1 end of
    conversion
    */
    DMA_DeInit(DMA1_Channel1);
    DMA_Init(DMA1_Channel1, &ch1_config);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC interrupts */
    NVIC_Init(&nvic_config);
}


static void hal_init_can_() {
    CAN_InitTypeDef config = {
        .CAN_Prescaler = 0,
        .CAN_Mode = CAN_Mode_Normal, /* Or CAN_Mode_LoopBack */
        .CAN_SJW = 0,
        .CAN_BS1 = 0,
        .CAN_BS2 = 0,
        .CAN_TTCM = DISABLE, /* Enable or disable the time triggered
                                communication mode. */
        .CAN_ABOM = ENABLE,  /* Enable or disable the automatic bus-off
                                management. */
        .CAN_AWUM = ENABLE,  /* Enable or disable the automatic wake-up
                                mode. */
        .CAN_NART = ENABLE,  /* Enable or disable the no-automatic
                                retransmission mode. */
        .CAN_RFLM = DISABLE,  /* Enable or disable the Receive FIFO Locked
                                 mode. */
        .CAN_TXFP = ENABLE    /* Enable or disable the transmit FIFO
                                priority. */
    };
    CAN_FilterInitTypeDef filter_config = {
        .CAN_FilterIdHigh = 0,         /* Specifies the filter identification
                                          number (MSBs for a 32-bit
                                          configuration, first one for a
                                          16-bit configuration). */
        .CAN_FilterIdLow = 0,          /* Specifies the filter identification
                                          number (LSBs for a 32-bit
                                          configuration, second one for a
                                          16-bit configuration). */
        .CAN_FilterMaskIdHigh = 0,     /* Specifies the filter mask number or
                                          identification number, according to
                                          the mode (MSBs for a 32-bit
                                          configuration, first one for a
                                          16-bit configuration). */
        .CAN_FilterMaskIdLow = 0,      /* Specifies the filter mask number or
                                          identification number, according to
                                          the mode (LSBs for a 32-bit
                                          configuration, second one for a
                                          16-bit configuration).*/
        .CAN_FilterFIFOAssignment = 0,
        .CAN_FilterNumber = 0,          /* Specifies the filter which will be
                                           initialized. It ranges from 0 to
                                           13. */
        .CAN_FilterMode = 0,            /* Specifies the filter mode to be
                                           initialized. */
        .CAN_FilterScale = 0,           /* Specifies the filter scale. */
        .CAN_FilterActivation = ENABLE
    };
    NVIC_InitTypeDef nvic_config = {
        .NVIC_IRQChannel = 0,
        .NVIC_IRQChannelPreemptionPriority = CAN_RX_PRE_EMPTION_PRIORITY,
        .NVIC_IRQChannelSubPriority = CAN_RX_SUB_PRIORITY,
        .NVIC_IRQChannelCmd = ENABLE
    };

#if (HAL_CAN_RATE_BPS == 1000000)
    /* 1 Mbaud from 72 MHz system clock */
    config.CAN_Prescaler = 4;
    config.CAN_BS1 = 6;
    config.CAN_BS2 = 0;
#elif (HAL_CAN_RATE_BPS == 500000)
    /* 500 Kbaud from 72 MHz system clock */
    config.CAN_Prescaler = 9;
    config.CAN_BS1 = 5;
    config.CAN_BS2 = 0;
#elif (HAL_CAN_RATE_BPS == 250000)
    /* 250 Kbaud from 72 MHz system clock */
    config.CAN_Prescaler = 9;
    config.CAN_BS1 = 12;
    config.CAN_BS2 = 1;
#else
#pragma error("Invalid CAN_RATE_BPS")
#endif

    CAN_DeInit(CAN1);

    CAN_Init(CAN1, &config);
    CAN_FilterInit(&filter_config);

    /* Configure interrupts for both RX FIFOs */
    nvic_config.NVIC_IRQChannel = (uint8_t)USB_LP_CAN1_RX0_IRQn;
    NVIC_Init(&nvic_config);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    nvic_config.NVIC_IRQChannel = (uint8_t)CAN1_RX1_IRQn;
    NVIC_Init(&nvic_config);
    CAN_ITConfig(CAN1, CAN_IT_FMP1, ENABLE);
}


static void hal_run_calibration_() {
    ADC_InjectedInitTypeDef injected_config = {
        .ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_1,
        .ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_None,
        .ADC_NbrOfInjecChannel = 3u,
        .ADC_InjecSequence1 = HAL_ADC_PHASE_A_CHANNEL,
        .ADC_InjecSequence2 = HAL_ADC_PHASE_B_CHANNEL,
        .ADC_InjecSequence3 = HAL_ADC_PHASE_C_CHANNEL,
        .ADC_InjecSequence4 = 1 /* ADC_InjectedInit requires all four channels
                                   be valid, even if not used */
    };

    size_t i;
    uint32_t offset[3] = {0, 0, 0};

    /* Ensure MOSFETs are turned off before we start */
    hal_set_pwm_state(HAL_PWM_STATE_LOW);

    /* Don't need end-of-conversion interrupts */
    ADC_ITConfig(ADC1, ADC_IT_JEOS, DISABLE);
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOS);

    /* Set up a conversion sequence to read from each of the shunts */
    ADC_InjectedInit(ADC1, &injected_config);

    ADC_InjectedChannelSampleTimeConfig(
        ADC1, HAL_ADC_PHASE_A_CHANNEL, ADC_SampleTime_7Cycles5);
    ADC_InjectedChannelSampleTimeConfig(
        ADC1, HAL_ADC_PHASE_B_CHANNEL, ADC_SampleTime_7Cycles5);
    ADC_InjectedChannelSampleTimeConfig(
        ADC1, HAL_ADC_PHASE_C_CHANNEL, ADC_SampleTime_7Cycles5);

    /* Sample the three shunts 16384 times each (takes ~6 ms) */
    for (i = 0; i < 16384u; i++) {
        ADC_StartInjectedConversion(ADC1);
        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOS));
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOS);

        offset[0] +=
            ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
        offset[1] +=
            ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
        offset[2] +=
            ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
    }

    /*
    Get the mean shunt offset and set the channels up to subtract that offset
    value from the readings. This makes the output values signed 12-bit packed
    into the LSBs of a 16-bit register, with sign extension.
    */
    ADC_SetChannelOffset1(ADC1, HAL_ADC_PHASE_A_CHANNEL,
                          (uint16_t)(offset[0] / 16384u));
    ADC_ChannelOffset1Cmd(ADC1, ENABLE);

    ADC_SetChannelOffset2(ADC1, HAL_ADC_PHASE_B_CHANNEL,
                          (uint16_t)(offset[1] / 16384u));
    ADC_ChannelOffset2Cmd(ADC1, ENABLE);

    ADC_SetChannelOffset3(ADC1, HAL_ADC_PHASE_C_CHANNEL,
                          (uint16_t)(offset[2] / 16384u));
    ADC_ChannelOffset3Cmd(ADC1, ENABLE);

    /* Read VBUS and temperature */
    ADC_StartConversion(ADC1);
    while (!hal_read_last_vbus_temp_());

    /*
    Enable end-of-conversion interrupts and hardware triggering, then start
    converting
    */
    ADC_ITConfig(ADC1, ADC_IT_JEOS, ENABLE);
    injected_config.ADC_ExternalTrigInjecEventEdge =
        ADC_ExternalTrigInjecEventEdge_RisingEdge;
    ADC_InjectedInit(ADC1, &injected_config);
}


static bool hal_read_last_vbus_temp_() {
    /* TS_CAL_1 is the temperature sensor reading at 30 deg C */
    static uint16_t ts_cal_1 = *((volatile uint16_t*)(0x1FFFF7B8));
    /* TS_CAL_2 is the temperature sensor reading at 110 deg C */
    static uint16_t ts_cal_2 = *((volatile uint16_t*)(0x1FFFF7C2));
    static float ts_deg_c_per_lsb = (110.0f - 30.0f) /
                                    (float)(ts_cal_2 - ts_cal_1);
    float temp;

    /* Check if the last VBUS and temperature conversion is done */
    if (!ADC_GetStartConversionStatus(ADC1)) {
        /*
        Low-pass the readings if we've already taken them, otherwise seed
        the filter with the current value.
        */
        if (board_vbus_lsb_ > 0) {
            board_vbus_lsb_ = (board_vbus_lsb_ * 63 +
                               (adc_conversion_results_[0] << 7)) >> 6;
        } else {
            board_vbus_lsb_ = adc_conversion_results_[0] << 7;
        }
        temp = (float)(board_vbus_lsb_ >> 4) * hal_full_scale_voltage_v *
               (1.0f / 32768.0f);
        vbus_v_ = temp;
        if (temp > 6.0f) {
            vbus_inv_ = 32768.0f / temp;
        } else {
            vbus_inv_ = 0.0f;
        }

        if (board_temp_lsb_ > 0) {
            board_temp_lsb_ = (board_temp_lsb_ * 63 +
                               (adc_conversion_results_[1] << 7)) >> 6;
        } else {
            board_temp_lsb_ = adc_conversion_results_[1] << 7;
        }
        temp = 30.0f +
               (float)((int32_t)(board_temp_lsb_ >> 7) - (int32_t)ts_cal_1) *
               ts_deg_c_per_lsb;
        temp_degc_ = temp;

        /* Start a new conversion */
        ADC_StartConversion(ADC1);

        return true;
    } else {
        return false;
    }
}


void hal_restart(void) {
    hal_set_pwm_state(HAL_PWM_STATE_OFF);

    /* NVIC_SystemReset(); */
    __DSB();
    SCB->AIRCR = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) |
                 (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                 SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    for (volatile bool cond = true; cond;);
}


void hal_reset(void) {
    /* Clear out state */
    pwm_state_ = HAL_PWM_STATE_OFF;
    board_vbus_lsb_ = 0;

    hal_init_sys_();
    hal_init_io_();
    hal_init_tim1_();
    hal_init_adc_();
    hal_init_dma_();
    hal_init_can_();
    hal_init_pwm_();

    /* Calibrate the current shunt sensor offsets */
    hal_run_calibration_();

    /* Trigger an ADC conversion to start the high-frequency task */
    ADC_StartInjectedConversion(ADC1);

    /* Enable PWM generation from TIM1 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* Enable SysTick 1 kHz periodic task */
    SysTick_Config(hal_core_frequency_hz / 1000u);
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
}


void hal_set_pwm_state(enum hal_pwm_state_t state) {
    if (state == pwm_state_) {
        /* No change -- ignore */
    } else if (state == HAL_PWM_STATE_OFF) {
        /* Tri-state the drive pins to force the MOSFET drivers off */
        GPIO_Init(PORT_PHASE_SW, &PIN_PHASE_SW_DIS);
    } else if (state == HAL_PWM_STATE_LOW) {
        /* Force all outputs low */
        GPIO_ResetBits(PORT_PHASE_SW, (uint16_t)PIN_PHASE_SW_LOW.GPIO_Pin);
        GPIO_Init(PORT_PHASE_SW, &PIN_PHASE_SW_LOW);
    } else if (state == HAL_PWM_STATE_RUNNING) {
        /* Take PWM output pins out of tri-state */
        GPIO_Init(PORT_PHASE_SW, &PIN_PHASE_SW_EN);
    }

    pwm_state_ = state;
}


enum hal_status_t hal_transmit_can_message(const CANMessage& message) {
    CanTxMsg msg;
    uint8_t result;

    msg.RTR = CAN_RTR_Data;

    if (message.has_extended_id()) {
        msg.ExtId = message.get_id();
        msg.IDE = CAN_Id_Extended;
    } else {
        msg.StdId = message.get_id();
        msg.IDE = CAN_Id_Standard;
    }

    msg.DLC = (uint8_t)message.get_data(msg.Data);

    result = CAN_Transmit(CAN1, &msg);

    if (result == CAN_TxStatus_NoMailBox) {
        return HAL_STATUS_ERROR;
    } else {
        return HAL_STATUS_OK;
    }
}


float hal_get_temperature_degc(void) {
    return temp_degc_;
}


void hal_set_low_frequency_callback(hal_callback_t callback) {
    esc_assert(!callback || !low_frequency_task_);

    low_frequency_task_ = callback;
}


void hal_set_high_frequency_callback(hal_control_callback_t callback) {
    esc_assert(!callback || !high_frequency_task_);

    high_frequency_task_ = callback;
}


void hal_set_can_receive_callback(hal_can_callback_t callback) {
    esc_assert(!callback || !can_receive_task_);

    can_receive_task_ = callback;
}


void hal_set_rc_pwm_callback(hal_pwm_callback_t callback) {
    esc_assert(!callback || !pwm_receive_task_);

    pwm_receive_task_ = callback;
}

void hal_flash_protect(bool __attribute__((unused)) readonly) {
    /* TODO */
}


void hal_flash_erase(uint8_t *addr, size_t len) {
    FLASH_Status result;

    hal_set_pwm_state(HAL_PWM_STATE_OFF);

    /* Pages and lengths must be a multiple of 2048 */
    esc_assert(((uint32_t)addr & 0x7FFu) == 0);
    esc_assert(((uint32_t)len & 0x7FFu) == 0);

    FLASH_Unlock();
    while (len) {
        result = FLASH_ErasePage((uint32_t)addr);
        esc_assert(result == FLASH_COMPLETE);

        addr += 2048u;
        len -= 2048u;
    }
    FLASH_Lock();
}


void hal_flash_write(uint8_t *addr, size_t len, const uint8_t *data) {
    uint32_t word;
    FLASH_Status result;

    hal_set_pwm_state(HAL_PWM_STATE_OFF);

    /* Addresses and lengths must be 4-byte aligned */
    esc_assert(((uint32_t)addr & 0x3u) == 0);
    esc_assert(((uint32_t)len & 0x3u) == 0);

    FLASH_Unlock();
    while (len) {
        memcpy(&word, data, sizeof(uint32_t));
        result = FLASH_ProgramWord((uint32_t)addr, word);

        /*
        Ensure the operation completed successfully and the correct word was
        written.
        */
        esc_assert(result == FLASH_COMPLETE);
        esc_assert(memcmp(addr, data, sizeof(uint32_t)) == 0);

        addr += 4u;
        data += 4u;
        len -= 4u;
    }
    FLASH_Lock();
}

void hal_disable_can_transmit(void) {
    GPIO_SetBits(PORT_CAN_SILENT, (uint16_t)PIN_CAN_SILENT.GPIO_Pin);
}


void hal_enable_can_transmit(void) {
    GPIO_ResetBits(PORT_CAN_SILENT, (uint16_t)PIN_CAN_SILENT.GPIO_Pin);
}

