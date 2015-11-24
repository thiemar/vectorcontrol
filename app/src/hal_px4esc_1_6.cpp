/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/*
Note: this file is NOT built directly. Instead, it is conditionally included
by hal.cpp
*/

#include <algorithm>
#include <stm32_dac.h>
#include <stm32_adc.h>

/*
PX4ESC 1.6 board pin definitions - STM32F446RE

PIN   PORT   NUMBER    FUNCTION
  1      -        -    VBAT
  2      C       13    GPIN - PWRGD
  3      C       14    GPIN - OCTW
  4      C       15    GPIN - FAULT
  5      H        0    OSC_IN
  6      H        1    OSC_OUT
  7      -        -    NRST
  8      C        0    ADC - TEMP_SENS
  9      C        1    ADC - VBAT_SENS
 10      C        2    ADC - CURR_SENS2
 11      C        3    ADC - CURR_SENS1
 12      -        -    VSSA
 13      -        -    VDDA
 14      A        0    ADC - SENS_A
 15      A        1    ADC - SENS_B
 16      A        2    ADC - SENS_C
 17      A        3    GPIN - RC_PWM
 18      -        -    VSS
 19      -        -    VDD
 20      A        4    GPOUT - OC_ADJ
 21      A        5    GPOUT - EN_GATE
 22      A        6    GPOUT - DC_CAL
 23      A        7    TIM1_CH1N
 24      C        4    -
 25      C        5    GPIN - DBG_RX
 26      B        0    TIM1_CH2N
 27      B        1    TIM1_CH3N
 28      B        2    GPOUT - GAIN
 29      B       10    GPOUT - DBG_TX
 30      -        -    VCAP1
 31      -        -    VSS
 32      -        -    VDD
 33      B       12    -
 34      B       13    -
 35      B       14    -
 36      B       15    -
 37      C        6    GPOUT - RPM
 38      C        7    LED_RED
 39      C        8    LED_GREEN
 40      C        9    LED_BLUE
 41      A        8    TIM1_CH1
 42      A        9    TIM1_CH2
 43      A       10    TIM1_CH3
 44      A       11    USB_DM
 45      A       12    USB_DP
 46      A       13    SWDIO - debug access
 47      -        -    VSS
 48      -        -    VDD
 49      A       14    SWCLK - debug access
 50      A       15    -
 51      C       10    -
 52      C       11    -
 53      C       12    GPIO - TP4
 54      D        2    GPIO - TP1
 55      B        3    GPIO - TP2
 56      B        4    GPIO - TP3
 57      B        5    CAN2_RX
 58      B        6    CAN2_TX
 59      B        7    -
 60      -        -    BOOT0
 61      B        8    CAN1_RX
 62      B        9    CAN1_TX
 63      -        -    VSS
 64      -        -    VDD
*/

/* These two are externally useful */
extern const uint32_t hal_core_frequency_hz = 180000000u;
extern const uint32_t hal_pwm_frequency_hz = 22500u;

const uint32_t hal_tim1_frequency_hz = hal_core_frequency_hz / 4u;
const uint32_t hal_adc_frequency_hz = hal_core_frequency_hz / 8.0f;

const uint32_t hal_pwm_period_ticks = hal_tim1_frequency_hz /
                                      hal_pwm_frequency_hz;
const uint32_t hal_pwm_half_period_ticks = hal_pwm_period_ticks / 2;
const uint32_t hal_pwm_quarter_period_ticks = hal_pwm_period_ticks / 4;

const float hal_pwm_dead_time_ns_ = 800.0f;
const uint32_t hal_pwm_dead_time_ticks_ =
    (uint32_t)(hal_pwm_dead_time_ns_ * hal_tim1_frequency_hz / 2.0f * 1e-9f);


/*
Set the TIM1 update event rate based on the selected PWM frequency. Must
always be odd; 1 results in an update event every PWM cycle, and 3 results
in an update event every 2 PWM cycles.
*/
const uint32_t hal_pwm_control_rate_div =
    hal_pwm_frequency_hz < 40000 ? 1u : 3u;


/* Work out the ADC sampling time in nanoseconds */
const float hal_adc_sample_periods = 15.0f;
const float hal_adc_sample_time_ns = 1e9f * (hal_adc_sample_periods + 12.0f) /
                                     hal_adc_frequency_hz;
const float hal_adc_shunt_settling_time_ns = 1500.0f;
const uint32_t hal_adc_sample_time_ticks =
    (uint32_t)(hal_adc_sample_time_ns * hal_tim1_frequency_hz * 1e-9f + 0.5f);
const uint32_t hal_adc_settling_time_ticks =
    (uint32_t)(hal_adc_shunt_settling_time_ns *
               hal_tim1_frequency_hz * 1e-9f + 0.5f);


/* Board parameters */
const float hal_nominal_mcu_vdd_v = 3.3f;
const float hal_current_sense_gain_v_per_v = 10.0f;
const float hal_current_sense_r = 0.001f;
const float hal_vbus_gain_v_per_v = 660.0f / (10400.0f + 660.0f);
const uint32_t hal_adc_full_scale_lsb = 1u << 12u;
const float hal_adc_v_per_lsb =
    hal_nominal_mcu_vdd_v / (float)hal_adc_full_scale_lsb;
const float hal_current_limit_a = 100.0f; /* FIXME! */
const float hal_mosfet_rdson_r = 0.001f;
const float hal_oc_adj_v = hal_current_limit_a * hal_mosfet_rdson_r;


/* This is a signed quantity so can go +/- 1.65 V of the 1.65 V reference */
extern const float hal_full_scale_current_a =
    hal_adc_v_per_lsb * (float)hal_adc_full_scale_lsb /
    (hal_current_sense_r * hal_current_sense_gain_v_per_v);
/* Vbus input is 0-3.3 V */
extern const float hal_full_scale_voltage_v =
    hal_adc_v_per_lsb * (float)hal_adc_full_scale_lsb / hal_vbus_gain_v_per_v;

/* Control timestemp in seconds */
extern const float hal_control_t_s =
    (float)((1 + hal_pwm_control_rate_div) / 2) / (float)hal_pwm_frequency_hz;


#define HAL_ADC_PHASE_A_V_CHANNEL 0u
#define HAL_ADC_PHASE_B_V_CHANNEL 1u
#define HAL_ADC_PHASE_C_V_CHANNEL 2u
#define HAL_ADC_PHASE_A_I_CHANNEL 13u
#define HAL_ADC_PHASE_B_I_CHANNEL 12u
#define HAL_ADC_VBUS_CHANNEL 11u
#define HAL_ADC_TEMP_CHANNEL 10u


/* Vbus in raw ADC units (1/LSB) */
static uint32_t board_vbus_lsb_;


/* Board temperature in raw ADC units (1/LSB) */
static uint32_t board_temp_lsb_;


inline void hal_read_phase_shunts_(int16_t phase_shunt_signal_lsb[2]) {
    /* Direct reading of phase A and B current */
    if (phase_reverse_) {
        phase_shunt_signal_lsb[1] = (int16_t)getreg32(STM32_ADC1_JDR1);
        phase_shunt_signal_lsb[0] = (int16_t)getreg32(STM32_ADC2_JDR1);
    } else {
        phase_shunt_signal_lsb[0] = (int16_t)getreg32(STM32_ADC1_JDR1);
        phase_shunt_signal_lsb[1] = (int16_t)getreg32(STM32_ADC2_JDR1);
    }
}


inline void hal_update_timer_(const uint16_t phase_on_ticks[3]) {
    uint16_t sample_ticks;

    sample_ticks = hal_pwm_half_period_ticks - 2u;
                   //std::max(phase_on_ticks[0], phase_on_ticks[1]) +
                   //hal_adc_settling_time_ticks; // -
                   //hal_adc_sample_time_ticks - 2u;

    /*
    Update the on times for the PWM channels as well as the ADC trigger point
    */
    if (phase_reverse_) {
        putreg16(phase_on_ticks[1], STM32_TIM1_CCR1);
        putreg16(phase_on_ticks[0], STM32_TIM1_CCR2);
        putreg16(phase_on_ticks[2], STM32_TIM1_CCR3);
        putreg16(sample_ticks, STM32_TIM1_CCR4);
    } else {
        putreg16(phase_on_ticks[0], STM32_TIM1_CCR1);
        putreg16(phase_on_ticks[1], STM32_TIM1_CCR2);
        putreg16(phase_on_ticks[2], STM32_TIM1_CCR3);
        putreg16(sample_ticks, STM32_TIM1_CCR4);
    }
}


extern "C" void stm32_adc(void) {
PERF_COUNT_START
    static float last_v_ab[2];
    static float prev_v_ab[2];

    int16_t phase_current_lsb[2];
    uint16_t phase_oc[3];
    float out_v_ab[2], i_ab[2];
    float temp, vbus_inv;

    /* Disable JEXTEN to prevent re-triggering */
    putreg32(getreg32(STM32_ADC1_CR2) & ~ADC_CR2_JEXTEN_MASK, STM32_ADC1_CR2);

    /* Read and low-pass bus voltage */
    if (board_vbus_lsb_ > 0) {
        board_vbus_lsb_ = (board_vbus_lsb_ * 63 +
                           (getreg16(STM32_ADC3_JDR1) << 7)) >> 6;
    } else {
        board_vbus_lsb_ = getreg16(STM32_ADC3_JDR1) << 7;
    }
    temp = float(board_vbus_lsb_ >> 4) * hal_full_scale_voltage_v *
           float(1.0 / 32768.0);
    vbus_v_ = temp;
    if (temp > 6.0f) {
        vbus_inv = 32768.0f / temp;
    } else {
        vbus_inv = 0.0f;
    }

    hal_read_phase_shunts_(phase_current_lsb);

    /*
    Clarke transformation for balanced systems

    i_alpha = i_a,
    i_beta = (2 * i_b + i_a) / sqrt(3)

    Multiply by 8 because the phase current readings are right-aligned.
    */
    i_ab[0] = float(phase_current_lsb[0]) *
              float(hal_full_scale_current_a * 8.0 / 32768.0);
    temp = float(phase_current_lsb[1]) *
           float(hal_full_scale_current_a * 8.0 / 32768.0);
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

    /*
    Convert alpha-beta frame voltage fractions to SVM output compare values
    for each phase.
    */
    (void)svm_duty_cycle_from_v_alpha_beta(
        phase_oc,
        int16_t(__SSAT(int32_t(vbus_inv * out_v_ab[0]), 16u)),
        int16_t(__SSAT(int32_t(vbus_inv * out_v_ab[1]), 16u)),
        hal_pwm_period_ticks);

    /* Update the timer */
    hal_update_timer_(phase_oc);

    /* Clear all JEOC events, and prepare for the next hardware trigger */
    putreg32(getreg32(STM32_ADC1_SR) & ~ADC_SR_JEOC, STM32_ADC1_SR);
    /*
    Allow the next ADC conversion to happen based on the TIM1 CC4 event
    */
    putreg32(getreg32(STM32_ADC1_CR2) | ADC_CR2_JEXTEN_RISING,
             STM32_ADC1_CR2);

PERF_COUNT_END
}


static void hal_init_tim_() {
    /*
    Set deadtime clock division to 2; center aligned 1 counter mode;
    prescaler /4
    */
    putreg16(ATIM_CR1_CENTER1 | ATIM_CR1_2TCKINT, STM32_TIM1_CR1);
    putreg16((hal_core_frequency_hz / hal_tim1_frequency_hz) - 1u,
             STM32_TIM1_PSC);

    /* Set the period and repetition counter */
    putreg16(hal_pwm_half_period_ticks - 1u, STM32_TIM1_ARR);
    putreg16(hal_pwm_control_rate_div, STM32_TIM1_RCR);

    /*
    OC1-3 are PWM channels; thease are configured with preload enabled,
    high output polarity, low idle polarity, inverted negative outputs,
    and PWM1 mode. (High-side active while CNT < CCR.)

    OC4 is the ADC conversion timer; it is configured as above, but in PWM2
    mode. (Active when CNT > CCR.)
    */
    putreg32(ATIM_CCMR1_OC1PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) |
             ATIM_CCMR1_OC2PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT),
             STM32_TIM1_CCMR1);
    putreg32(ATIM_CCMR2_OC3PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) |
             ATIM_CCMR2_OC4PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC4M_SHIFT),
             STM32_TIM1_CCMR2);

    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR1);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR2);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR3);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR4);

    putreg32(ATIM_CCER_CC1E | ATIM_CCER_CC2E | ATIM_CCER_CC3E |
             ATIM_CCER_CC4E | ATIM_CCER_CC1NE | ATIM_CCER_CC2NE |
             ATIM_CCER_CC3NE /*| ATIM_CCER_CC4P*/,
             STM32_TIM1_CCER);

    /* Configure dead time register to reset value */
    putreg32(std::min((uint32_t)127u, hal_pwm_dead_time_ticks_),
             STM32_TIM1_BDTR);

    /* Output trigger is update event; all output idle states are 0 */
    putreg16(ATIM_CR2_MMS_UPDATE, STM32_TIM1_CR2);

    /*
    Enable TIM1 and resynchronize so update event happens during underflow
    */
    putreg16(ATIM_EGR_UG, STM32_TIM1_EGR);
    putreg16(getreg16(STM32_TIM1_CR1) | ATIM_CR1_CEN, STM32_TIM1_CR1);
}


static void hal_init_adc_() {
    uint32_t smpr_config;

    /* Enable DAC_OUT1 to set the DRV8302 overcurrent threshold */
    putreg32(DAC_CR_EN1, STM32_DAC_CR);
    putreg32((uint16_t)(hal_oc_adj_v / hal_nominal_mcu_vdd_v * 4096u),
             STM32_DAC_DHR12RD);

    /* Enable clk/4 prescaler */
    putreg32(ADC_CCR_ADCPRE_DIV4, STM32_ADC_CCR);

    /* Give some time for the clock to settle or something */
    for (volatile uint32_t x = 0;
         x < 10u * hal_core_frequency_hz / 1000000u; x++);

    /*
    ADC1, ADC2, ADC3 config: no continuous conversion mode,
    12 bit right-aligned, no external trigger, no auto-inject, no overrun mode

    ADC2 and ADC3 will be auto-triggered by ADC1 so no need to configure their
    triggers.
    */
    putreg32(ADC_CR1_RES_12BIT, STM32_ADC1_CR1);
    putreg32(ADC_CR1_RES_12BIT, STM32_ADC2_CR1);
    putreg32(ADC_CR1_RES_12BIT, STM32_ADC3_CR1);

    putreg32(ADC_CR2_JEXTSEL_T1CC4 | ADC_CR2_JEXTEN_NONE, STM32_ADC1_CR2);

    /*
    Configure temperature sense channel, and set the ADC SQR sequence length
    to 1.

    The temperature sensor has a sample time of 144 cycles.

    The injected phase current sense channels and the Vbus sense channel each
    have a sample time of 15 cycles.
    */
    smpr_config = (ADC_SMPR_144 << ADC_SMPR1_SMP10_SHIFT) |
                  (ADC_SMPR_15 << ADC_SMPR1_SMP11_SHIFT) |
                  (ADC_SMPR_15 << ADC_SMPR1_SMP12_SHIFT) |
                  (ADC_SMPR_15 << ADC_SMPR1_SMP13_SHIFT);

    putreg32(0u, STM32_ADC3_SQR1);
    putreg32(HAL_ADC_TEMP_CHANNEL << ADC_SQR3_SQ1_SHIFT, STM32_ADC3_SQR3);

    putreg32(smpr_config, STM32_ADC1_SMPR1);
    putreg32(smpr_config, STM32_ADC2_SMPR1);
    putreg32(smpr_config, STM32_ADC3_SMPR1);

    /* Set up a conversion sequence to read from each of the shunts. */
    putreg32(HAL_ADC_PHASE_A_I_CHANNEL << ADC_JSQR_JSQ4_SHIFT,
             STM32_ADC1_JSQR);
    putreg32(HAL_ADC_PHASE_B_I_CHANNEL << ADC_JSQR_JSQ4_SHIFT,
             STM32_ADC2_JSQR);
    putreg32(HAL_ADC_VBUS_CHANNEL << ADC_JSQR_JSQ4_SHIFT, STM32_ADC3_JSQR);

    /* Enable the ADCs */
    putreg32(getreg32(STM32_ADC1_CR2) | ADC_CR2_ADON, STM32_ADC1_CR2);
    putreg32(getreg32(STM32_ADC2_CR2) | ADC_CR2_ADON, STM32_ADC2_CR2);
    putreg32(getreg32(STM32_ADC3_CR2) | ADC_CR2_ADON, STM32_ADC3_CR2);

    /* Worst-case start-up delay is 3 us; allow 100-200 us */
    for (volatile uint32_t x = 0;
         x < 1000u * (hal_core_frequency_hz / 1000000u); x++);

    /* Now that config is done, enable triple injected simultaneous mode */
    putreg32(ADC_CCR_MULTI_ISM3, STM32_ADC_CCR);
}


static void hal_run_calibration_() {
    size_t i;
    uint32_t offset[2] = {0, 0};

    /* Ensure MOSFETs are turned off before we start. */
    hal_set_pwm_state(HAL_PWM_STATE_LOW);

    /* Wait for DRV8302 to become ready -- around 100 ms */
    for (volatile uint32_t x = 0;
         x < 100000u * (hal_core_frequency_hz / 1000000u); x++);

    /* Sample the three shunts 1M times each (takes ~6 ms) */
    for (i = 0; i < 1048576u; i++) {
        /* Trigger the injected conversion sequence */
        putreg32(getreg32(STM32_ADC1_CR2) | ADC_CR2_JSWSTART, STM32_ADC1_CR2);

        /*
        Wait for the JEOC bit to go high, marking the end of the injected
        sequence
        */
        while (!(getreg32(STM32_ADC1_SR) & ADC_SR_JEOC));

        /* Clear JEOC */
        putreg32(getreg32(STM32_ADC1_SR) & ~ADC_SR_JEOC, STM32_ADC1_SR);

        /* Get injected conversion values */
        offset[0] += (uint16_t)getreg16(STM32_ADC1_JDR1);
        offset[1] += (uint16_t)getreg16(STM32_ADC2_JDR1);
    }

    /*
    Get the mean shunt offset and set the channels up to subtract that offset
    value from the readings. This makes the output values signed 12-bit packed
    into the LSBs of a 16-bit register, with sign extension.
    */
    putreg16(offset[0] / 1048576u, STM32_ADC1_JOFR1);
    putreg16(offset[1] / 1048576u, STM32_ADC2_JOFR1);

    /* Read temperature */
    putreg32(getreg32(STM32_ADC3_CR2) | ADC_CR2_SWSTART, STM32_ADC3_CR2);
    while (!hal_adc_periodic_());

    /*
    Enable end-of-conversion interrupts and hardware triggering, then start
    converting
    */
    putreg32(getreg32(STM32_ADC1_CR1) | ADC_CR1_JEOCIE, STM32_ADC1_CR1);
    putreg32(getreg32(STM32_ADC1_CR2) | ADC_CR2_JEXTEN_RISING,
             STM32_ADC1_CR2);
}


static bool hal_adc_periodic_() {
    /* TS_CAL_1 is the temperature sensor reading at 30 deg C */
    static uint16_t ts_cal_1 = *((volatile uint16_t*)(0x1FFFF7B8));
    /* TS_CAL_2 is the temperature sensor reading at 110 deg C */
    static uint16_t ts_cal_2 = *((volatile uint16_t*)(0x1FFFF7C2));
    static float ts_deg_c_per_lsb = (110.0f - 30.0f) /
                                    float(ts_cal_2 - ts_cal_1);
    float temp;

    /* Check if the last temperature conversion is done */
    if (!(getreg32(STM32_ADC3_SR) & ADC_SR_EOC)) {
        /*
        Low-pass the readings if we've already taken them, otherwise seed
        the filter with the current value.
        */
        if (board_temp_lsb_ > 0) {
            board_temp_lsb_ = (board_temp_lsb_ * 63 +
                               (getreg32(STM32_ADC3_DR) << 7)) >> 6;
        } else {
            board_temp_lsb_ = getreg32(STM32_ADC3_DR) << 7;
        }
        temp = 30.0f +
               float(int32_t(board_temp_lsb_ >> 7) - int32_t(ts_cal_1)) *
               ts_deg_c_per_lsb;
        temp_degc_ = temp;

        /* Start a new conversion */
        putreg32(getreg32(STM32_ADC3_CR2) | ADC_CR2_SWSTART, STM32_ADC3_CR2);

        return true;
    } else {
        return false;
    }
}


void hal_reset(void) {
    /* Clear out state */
    pwm_state_ = HAL_PWM_STATE_OFF;
    board_vbus_lsb_ = 0;

    up_cxxinitialize();
    board_initialize();

    /* GPIO_GAIN = 1 means 40 V/V gain on the current shunts */
    // stm32_gpiowrite(GPIO_GAIN, 1u);
    stm32_gpiowrite(GPIO_GAIN, 0u);
    stm32_gpiowrite(GPIO_DC_CAL, 0u);

    hal_init_sys_();

    /*
    Read bootloader auto-baud and node ID values, then set up the node. We've
    just come from the bootloader, so CAN must already be configured
    correctly.
    */
    if (!bootloader_read(&bootloader_app_shared_)) {
        up_systemreset();
    }
    can_init(bootloader_app_shared_.bus_speed);

    /* Initialize timer and ADC */
    hal_init_tim_();
    hal_init_adc_();

    /* Calibrate the current shunt sensor offsets */
    hal_run_calibration_();

    up_prioritize_irq(STM32_IRQ_ADC, NVIC_SYSH_PRIORITY_MAX);
    up_enable_irq(STM32_IRQ_ADC);
    /* Perform timer initialization -- moved from NuttX __start */
    up_timer_initialize();

    /* Enable IRQs to make sure we get the next interrupt */
    irqenable();

    /* Trigger an ADC conversion to start the high-frequency task */
    putreg32(getreg32(STM32_ADC1_CR2) | ADC_CR2_JSWSTART, STM32_ADC1_CR2);
}


void hal_set_pwm_state(enum hal_pwm_state_t state) {
    if (state == pwm_state_) {
        /* No change -- ignore */
    } else if (state == HAL_PWM_STATE_OFF) {
        /* Turn gate drives and timers off */
        putreg32(getreg32(STM32_TIM1_BDTR) & ~ATIM_BDTR_MOE, STM32_TIM1_BDTR);
        stm32_gpiowrite(GPIO_EN_GATE, 0u);
    } else if (state == HAL_PWM_STATE_LOW) {
        /* Force all outputs low */
        putreg32(getreg32(STM32_TIM1_BDTR) & ~ATIM_BDTR_MOE, STM32_TIM1_BDTR);
        stm32_gpiowrite(GPIO_EN_GATE, 1u);
    } else if (state == HAL_PWM_STATE_RUNNING) {
        /* Drive EN_GATE high */
        putreg32(getreg32(STM32_TIM1_BDTR) | ATIM_BDTR_MOE, STM32_TIM1_BDTR);
        stm32_gpiowrite(GPIO_EN_GATE, 1u);
    }

    pwm_state_ = state;
}
