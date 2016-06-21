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

#include <stm32f10xxx_dma.h>
#include <stm32f30xxx_adc.h>
#include "drv8305.h"


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
    hal_pwm_frequency_hz < 32000u ? 1u : 3u;


/* Work out the ADC sampling time in nanoseconds */
const float hal_adc_sample_periods = 7.5f;
const float hal_adc_sample_time_ns =
    1e9f * (3.0f * hal_adc_sample_periods + 2.0f * 13.5f) /
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
const uint32_t hal_adc_calibration_sample_count_log2 = 20u;
const uint32_t hal_adc_calibration_sample_count =
    (1u << hal_adc_calibration_sample_count_log2);
const uint32_t hal_adc_calibration_sample_round =
    hal_adc_calibration_sample_count >> 1u;


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


#define HAL_ADC_PHASE_A_I_CHANNEL 1u
#define HAL_ADC_PHASE_B_I_CHANNEL 2u
#define HAL_ADC_PHASE_C_I_CHANNEL 3u
#define HAL_ADC_VBUS_CHANNEL 4u
#define HAL_ADC_PHASE_A_V_CHANNEL 5u
#define HAL_ADC_PHASE_B_V_CHANNEL 10u
#define HAL_ADC_PHASE_C_V_CHANNEL 15u
#define HAL_ADC_TEMP_CHANNEL 16u


/* ADC conversion result destination */
static volatile uint16_t adc_conversion_results_[2];


/* Vbus in raw ADC units (1/LSB) */
static uint32_t board_vbus_lsb_;


/* Cached inverse vbus value */
static volatile float vbus_inv_;


/* Board temperature in raw ADC units (1/LSB) */
static uint32_t board_temp_lsb_;


static void hal_init_dma_();


inline void
hal_read_phase_shunts_(
    int16_t phase_shunt_signal_lsb[3],
    uint8_t phase_pwm_sector
) {
    if (phase_reverse_) {
        /* Direct reading of phase A, B and C current */
        phase_shunt_signal_lsb[1] = (int16_t)getreg32(STM32_ADC1_JDR1);
        phase_shunt_signal_lsb[0] = (int16_t)getreg32(STM32_ADC1_JDR2);
    } else {
        /* Direct reading of phase A, B and C current */
        phase_shunt_signal_lsb[0] = (int16_t)getreg32(STM32_ADC1_JDR1);
        phase_shunt_signal_lsb[1] = (int16_t)getreg32(STM32_ADC1_JDR2);
    }
    phase_shunt_signal_lsb[2] = (int16_t)getreg32(STM32_ADC1_JDR3);

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
            phase_shunt_signal_lsb[0] = int16_t(-phase_shunt_signal_lsb[2] -
                                                phase_shunt_signal_lsb[1]);
            break;

        case 2:
        case 3:
            /*
            Phase B current is not observable -- reconstruct:
            IB = -IC - IA
            */
            phase_shunt_signal_lsb[1] = int16_t(-phase_shunt_signal_lsb[2] -
                                                phase_shunt_signal_lsb[0]);
            break;

        default:
            break;
    }
}


inline void hal_update_timer_(
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
    putreg32(getreg32(STM32_TIM1_CCER) & ~ATIM_CCER_CC4P, STM32_TIM1_CCER);

    if (uint16_t(period_ticks - phase1_ticks) > hal_adc_settling_time_ticks) {
        sample_ticks = uint16_t(period_ticks - 1u);
    } else {
        duty_delta_ticks = uint16_t(phase1_ticks - phase2_ticks);
        sample_ticks = uint16_t(phase1_ticks);

        /* Check which side of the crossing point we should sample */
        if (duty_delta_ticks > uint16_t(period_ticks - phase1_ticks) << 1u) {
            sample_ticks = uint16_t(sample_ticks - hal_adc_sample_time_ticks);
        } else {
            sample_ticks =
                uint16_t(sample_ticks + hal_adc_settling_time_ticks);

            if (sample_ticks >= period_ticks) {
                /* Make polarity of CC4 active low */
                putreg32(getreg32(STM32_TIM1_CCER) | ATIM_CCER_CC4P,
                         STM32_TIM1_CCER);

                sample_ticks =
                    uint16_t(hal_pwm_period_ticks - 3u - sample_ticks);
            }
        }
    }

    /*
    Update the on times for the PWM channels as well as the ADC trigger point
    */
    if (phase_reverse_) {
        putreg16(phase_on_ticks[1], STM32_TIM1_CCR1);
        putreg16(phase_on_ticks[0], STM32_TIM1_CCR2);
    } else {
        putreg16(phase_on_ticks[0], STM32_TIM1_CCR1);
        putreg16(phase_on_ticks[1], STM32_TIM1_CCR2);
    }
    putreg16(phase_on_ticks[2], STM32_TIM1_CCR3);
    putreg16(sample_ticks, STM32_TIM1_CCR4);
}


extern "C" void stm32_adc12(void) {
PERF_COUNT_START
    static float last_v_ab[2];
    static float prev_v_ab[2];
    static uint8_t last_pwm_sector = 1u;
    static uint8_t prev_pwm_sector = 1u;

    int16_t phase_current_lsb[3];
    uint16_t phase_oc[3];
    float out_v_ab[2], i_ab[2];
    float temp;

    hal_read_phase_shunts_(phase_current_lsb, prev_pwm_sector);

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

    out_v_ab[0] = out_v_ab[1] = 0.0f;

    if (high_frequency_task_) {
        high_frequency_task_(out_v_ab, prev_v_ab, i_ab, vbus_v_);
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
    temp = vbus_inv_;
    last_pwm_sector = svm_duty_cycle_from_v_alpha_beta(
        phase_oc,
        int16_t(__SSAT(int32_t(temp * out_v_ab[0]), 16u)),
        int16_t(__SSAT(int32_t(temp * out_v_ab[1]), 16u)),
        hal_pwm_period_ticks);

    /* Update the timer */
    hal_update_timer_(last_pwm_sector, phase_oc);

    /*
    Clear the JEOS event, and prepare for the next hardware trigger
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOS);
    */
    putreg32(ADC_INT_JEOS, STM32_ADC1_ISR);
    /*
    Allow the next ADC conversion to happen based on the TIM1 CC4 event
    ADC_StartInjectedConversion(ADC1);
    */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_JADSTART, STM32_ADC1_CR);

PERF_COUNT_END
}


static void hal_init_tim_() {
    /* Set clock division to 1; center aligned 1 counter mode; prescaler 0 */
    putreg16(ATIM_CR1_CENTER1 | ATIM_CR1_2TCKINT, STM32_TIM1_CR1);
    putreg16(0u, STM32_TIM1_PSC);

    /* Set the period and repetition counter */
    putreg16(hal_pwm_half_period_ticks - 1u, STM32_TIM1_ARR);
    putreg16(hal_pwm_control_rate_div, STM32_TIM1_RCR);

    /*
    OC1-3 are PWM channels; thease are configured with preload enabled,
    high output polarity, low idle polarity, no negative outputs,
    and PWM1 mode.

    OC4 is the ADC conversion timer; it is configured as above, but in PWM2
    mode.
    */
    putreg32(ATIM_CCMR1_OC1PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) |
             ATIM_CCMR1_OC2PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT),
             STM32_TIM1_CCMR1);
    putreg32(ATIM_CCMR2_OC3PE | (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) |
             ATIM_CCMR2_OC4PE | (ATIM_CCMR_MODE_PWM2 << ATIM_CCMR2_OC4M_SHIFT),
             STM32_TIM1_CCMR2);

    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR1);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR2);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR3);
    putreg16(hal_pwm_quarter_period_ticks, STM32_TIM1_CCR4);

    putreg32(ATIM_CCER_CC1E | ATIM_CCER_CC2E | ATIM_CCER_CC3E |
             ATIM_CCER_CC4E,
             STM32_TIM1_CCER);

    /* Configure dead time register to reset value*/
    putreg32(0u, STM32_TIM1_BDTR);

    /* Output trigger is update event; all output idle states are 0 */
    putreg16(ATIM_CR2_MMS_UPDATE, STM32_TIM1_CR2);

    /*
    Enable TIM1 and resynchronize so update event happens during underflow
    */
    putreg16(getreg16(STM32_TIM1_CR1) | ATIM_CR1_CEN, STM32_TIM1_CR1);
    putreg16(ATIM_EGR_UG, STM32_TIM1_EGR);

    /* Initialize TIM2 for PWM input*/

    /*
    TIM2 is on APB1, at 36 MHz; divide clock by 36 for 1 us resolution, and
    set maximum count time to 65 ms.
    */
    putreg16(35u, STM32_TIM2_PSC);
    putreg16(65535u, STM32_TIM2_ARR);

    /* Enable PWM input capture mode -- see 19.3.6 of RM0365 */
    putreg32((GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC1S_SHIFT) |
             (GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC2S_SHIFT),
             STM32_TIM2_CCMR1);
    putreg32(GTIM_CCER_CC2P, STM32_TIM2_CCER);

    putreg16(GTIM_SMCR_TI1FP1, STM32_TIM2_SMCR);
    putreg16(GTIM_SMCR_RESET, STM32_TIM2_SMCR);

    putreg32(getreg32(STM32_TIM2_CCER) | GTIM_CCER_CC1E | GTIM_CCER_CC2E,
             STM32_TIM2_CCER);

    /* Start the counter */
    putreg16(getreg16(STM32_TIM2_CR1) | GTIM_CR1_CEN, STM32_TIM2_CR1);
}


static void hal_init_adc_() {
    /* Enable the ADC voltage regulator */
    putreg32(getreg32(STM32_ADC1_CR) & ~ADC_CR_ADVREGEN_MASK, STM32_ADC1_CR);
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_ADVREGEN_ENABLED,
             STM32_ADC1_CR);

    /* Worst-case regulator delay is 10 us */
    for (volatile uint32_t x = hal_core_frequency_hz / 100000u; x--;);

    /* ADC12 common config: independent, sync clock/1, DMA mode 1, one shot */
    putreg32(ADC_CCR_DUAL_IND | ADC_CCR_MDMA_10_12 | ADC_CCR_CKMODE_SYNCH_DIV1,
             STM32_ADC12_CCR);

    /*
    ADC1 config: no continuous conversion mode, 12 bit right-aligned, no
    external trigger, no auto-inject, no overrun mode
    */
    putreg32(ADC_CFGR_RES_12BIT, STM32_ADC1_CFGR);

    /* Calibrate the ADCs */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_ADCAL, STM32_ADC1_CR);
    while (getreg32(STM32_ADC1_CR) & ADC_CR_ADCAL);

    /*
    Delay after calibration done -- see STM32F302x6/x8 Silicon Limitations
    (DM00109012) item 2.2.4.
    */
    for (volatile uint32_t x = 4; x--;);

    /* Enable the ADC and temperature sensor */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_ADEN, STM32_ADC1_CR);
    putreg32(getreg32(STM32_ADC12_CCR) | ADC_CCR_TSEN, STM32_ADC12_CCR);

    while (!(getreg32(STM32_ADC12_CSR) & ADC_CSR_ADRDY_MST));

    /*
    Configure VBUS and temperature sense channels, and set the ADC SQR
    sequence length to 2.

    The Vbus channel has a sample time of 61.5 cycles, while the temperature
    sensors has a sample time of 181.5 cycles.

    The injected phase current sense channels each have a sample time of
    7.5 cycles.
    */
    putreg32((1u << ADC_SQR1_L_SHIFT) |
             (HAL_ADC_VBUS_CHANNEL << ADC_SQR1_SQ1_SHIFT) |
             (HAL_ADC_TEMP_CHANNEL << ADC_SQR1_SQ2_SHIFT),
             STM32_ADC1_SQR1);

    putreg32((ADC_SMPR_7p5 << (HAL_ADC_PHASE_A_I_CHANNEL * 3u)) |
             (ADC_SMPR_7p5 << (HAL_ADC_PHASE_B_I_CHANNEL * 3u)) |
             (ADC_SMPR_7p5 << (HAL_ADC_PHASE_C_I_CHANNEL * 3u)) |
             (ADC_SMPR_61p5 << (HAL_ADC_VBUS_CHANNEL * 3u)),
             STM32_ADC1_SMPR1);
    putreg32(ADC_SMPR_181p5 << ((HAL_ADC_TEMP_CHANNEL - 10u) * 3u),
             STM32_ADC1_SMPR2);

    /* Enable DMA for the Vbus and temperature conversion sequence */
    putreg32(getreg32(STM32_ADC1_CFGR) | ADC_CFGR_DMAEN, STM32_ADC1_CFGR);

    /*
    Set up a conversion sequence to read from each of the shunts. The trigger
    event is 1; for now, the event edge is disabled; there are 3 injected
    channels A, B and C.
    */
    putreg32((2u << ADC_JSQR_JL_SHIFT) |
             ADC_JSQR_JEXTSEL(1u) | ADC_JSQR_JEXTEN_NONE |
             ADC_JSQR_JSQ1(HAL_ADC_PHASE_A_I_CHANNEL) |
             ADC_JSQR_JSQ2(HAL_ADC_PHASE_B_I_CHANNEL) |
             ADC_JSQR_JSQ3(HAL_ADC_PHASE_C_I_CHANNEL),
             STM32_ADC1_JSQR);
}


static void hal_init_dma_() {
    /*
    DMA channel 1 (temperature and VBUS measurement) is linked to ADC1 end of
    conversion. Configure with:
    * peripheral base address STM32_ADC1_DR;
    * memory base adress adc_conversion_results_;
    * direction is peripheral->memory;
    * buffer size is 2;
    * peripheral increment is disabled, memory increment is enabled;
    * data size is 16 bits;
    * mode is circular.
    */
    putreg32(2u, STM32_DMA1_CNDTR(DMACHAN_ADC1));
    putreg32(STM32_ADC1_DR, STM32_DMA1_CPAR(DMACHAN_ADC1));
    putreg32((uint32_t)adc_conversion_results_,
             STM32_DMA1_CMAR(DMACHAN_ADC1));
    putreg32(DMA_CCR_EN | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BITS |
             DMA_CCR_MSIZE_16BITS | DMA_CCR_PRILO,
             STM32_DMA1_CCR(DMACHAN_ADC1));
}


static uint16_t hal_read_drv8305_(uint8_t reg_addr) {
    volatile uint16_t result;


    /* Enable SPI and drive NSS low */
    putreg16(getreg16(STM32_SPI3_CR1) | SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_DRV_CS, 0u);

    /* Wait a bit */
    for (volatile uint32_t x = 8u; x != 0; x--);

    /* Write address and read flag to the data register */
    putreg16(0x8000u | ((reg_addr & 0xFu) << 11u), STM32_SPI3_DR);

    /* Wait for TXE and RXNE to go high, and BSY to go low */
    const uint16_t done_status = SPI_SR_TXE | SPI_SR_RXNE;
    const uint16_t done_mask = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_BSY;
    while ((getreg16(STM32_SPI3_SR) ^ done_status) & done_mask);

    /* Collect the data */
    result = getreg16(STM32_SPI3_DR);

    /* Disable SPI and drive NSS high */
    putreg16(getreg16(STM32_SPI3_CR1) & ~SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_DRV_CS, 1u);

    /* Ignore the first 5 bits of the result */
    return (result & 0x7FFu);
}


static void hal_write_drv8305_(uint8_t reg_addr, uint16_t reg_val) {
    volatile uint16_t __attribute__((unused)) result;

    /* Enable SPI and drive NSS low */
    putreg16(getreg16(STM32_SPI3_CR1) | SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_DRV_CS, 0u);

    /* Wait a bit */
    for (volatile uint32_t x = 8u; x != 0; x--);

    /* Write 16 bits to the data register */
    putreg16(((reg_addr & 0xFu) << 11) | (reg_val & 0x7FFu), STM32_SPI3_DR);

    /* Wait for TXE and RXNE to go high, and BSY to go low */
    const uint16_t done_status = SPI_SR_TXE | SPI_SR_RXNE;
    const uint16_t done_mask = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_BSY;
    while ((getreg16(STM32_SPI3_SR) ^ done_status) & done_mask);

    /* Collect the data */
    result = getreg16(STM32_SPI3_DR);

    /* Disable SPI and drive NSS high */
    putreg16(getreg16(STM32_SPI3_CR1) & ~SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_DRV_CS, 1u);
}


static void hal_init_drv8305_(void) {
    /*
    SPI device configuration. Refer to section 7.5.1 of:
    "DRV8305 Three Phase Gate Driver With Current Shunt Amplifiers and Voltage
    Regulator", doc SLVSCX2A

    Clock is low when idle (CPOL=0); data is latched on the negative-going
    clock edge (CPHA=1). Frequency must <= 10 MHz for the DRV8305.

    Here, set f = PCLK / 4 (i.e. 9 MHz) and MSTR mode.
    */
    putreg16(SPI_CR1_FPCLCKd4 | SPI_CR1_MSTR | SPI_CR1_CPHA, STM32_SPI3_CR1);

    /*
    Set FRXTH to trigger RXNE when 16 bits have been received, DS to
    1111 for 16-bit, and SSOE to enable the NSS pin.
    */
    putreg16(SPI_CR2_FRXTH | SPI_CR1_DS_16BIT | SPI_CR2_SSOE, STM32_SPI3_CR2);

    /*
    The command format is:
    bit 15: R/nW
    bit 14-11: ADDR
    bit 10-0: DATA
    */

    uint16_t status;
    status = hal_read_drv8305_(DRV8305_WARN_ADDR);
    if (status) {
        up_systemreset();
    }
    status = hal_read_drv8305_(DRV8305_OV_VDS_FAULT_ADDR);
    if (status) {
        up_systemreset();
    }
    status = hal_read_drv8305_(DRV8305_IC_FAULT_ADDR);
    if (status) {
        up_systemreset();
    }
    status = hal_read_drv8305_(DRV8305_DRV_FAULT_ADDR);
    if (status) {
        up_systemreset();
    }

    /* Configure DRV8305 registers*/

    /*
    Drive strength 500 mA for 250 ns max, both high and low side -- can be
    more aggressive depending on switching transients
    */
    hal_write_drv8305_(DRV8305_HS_DRV_CTL_ADDR,
                       DRV8305_TDRIVE_250ns | DRV8305_IDRIVEN_500mA |
                       DRV8305_IDRIVEP_500mA);
    hal_write_drv8305_(DRV8305_LS_DRV_CTL_ADDR,
                       DRV8305_TDRIVE_250ns | DRV8305_IDRIVEN_500mA |
                       DRV8305_IDRIVEP_500mA);
    /*
    3-in PWM mode; dead time is 500 ns (could be more aggressive), and VDS
    comparator blanking and deglitch both set to 2 us to avoid false
    triggering -- we shut down on overcurrent so best to be sure
    */
    hal_write_drv8305_(DRV8305_DRV_CTL_ADDR,
                       DRV8305_PWM_3IN | DRV8305_DEADTIME_500ns |
                       DRV8305_TBLANK_2us | DRV8305_TVDS_2us);

    /* Clamp current sense lines to 3V3 */
    hal_write_drv8305_(DRV8305_IC_OP_ADDR, DRV8305_SNS_CLAMP_EN);

    /*
    Blank current shunts for 500 ns around switching; set gains to 20 V/V,
    for operation up to 80 A peak.
    */
    hal_write_drv8305_(DRV8305_SHUNT_CTL_ADDR,
                       DRV8305_CS_BLANK_500ns | DRV8305_GAIN_CS3_20VpV |
                       DRV8305_GAIN_CS2_20VpV | DRV8305_GAIN_CS1_20VpV);

    /* Don't need to make any changes to the VREF/VREG settings. */

    /*
    Set VDS threshold to 822 mV, which is about 80 A through 10 milliohms.
    In the event of a fault, the FET is likely to heat up and cross that
    resistance threshold fairly quickly so this should be enough to catch it,
    while being far enough from the normal operating range to avoid false
    triggering.

    The overcurrent response is immediate, latched shutdown.
    */
    hal_write_drv8305_(DRV8305_IC_OP_ADDR,
                       DRV8305_VDS_THRES_822mV | DRV8305_VDS_MODE_SHDN);
}


static void hal_run_calibration_() {
    size_t i;
    uint32_t offset[3] = {0, 0, 0};

    /* Don't need end-of-conversion interrupts */
    putreg32(getreg32(STM32_ADC1_IER) & ~ADC_INT_JEOS, STM32_ADC1_IER);

    /* ADC_ClearFlag(ADC1, ADC_FLAG_JEOS); */
    putreg32(ADC_INT_JEOS, STM32_ADC1_ISR);

    /* Sample the three shunts 65536 times each (takes ~24 ms) */
    for (i = 0; i < hal_adc_calibration_sample_count; i++) {
        /* Trigger the injected conversion sequence */
        putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_JADSTART, STM32_ADC1_CR);

        /*
        Wait for the JEOS bit to go high, marking the end of the injected
        sequence
        */
        while (!(getreg32(STM32_ADC1_ISR) & ADC_INT_JEOS));

        /* Write 1 to the JEOS bit to clear it (!) */
        putreg32(ADC_INT_JEOS, STM32_ADC1_ISR);

        /* Get injected conversion values */
        offset[0] += getreg16(STM32_ADC1_JDR1);
        offset[1] += getreg16(STM32_ADC1_JDR2);
        offset[2] += getreg16(STM32_ADC1_JDR3);
    }

    /*
    Get the mean shunt offset and set the channels up to subtract that offset
    value from the readings. This makes the output values signed 12-bit packed
    into the LSBs of a 16-bit register, with sign extension.
    */
    putreg32(ADC_OFR_OFFSETY((offset[0] + hal_adc_calibration_sample_round) >>
                             hal_adc_calibration_sample_count_log2) |
             ADC_OFR_OFFSETY_CH(HAL_ADC_PHASE_A_I_CHANNEL) | ADC_OFR_OFFSETY_EN,
             STM32_ADC1_OFR1);

    putreg32(ADC_OFR_OFFSETY((offset[1] + hal_adc_calibration_sample_round) >>
                             hal_adc_calibration_sample_count_log2) |
             ADC_OFR_OFFSETY_CH(HAL_ADC_PHASE_B_I_CHANNEL) | ADC_OFR_OFFSETY_EN,
             STM32_ADC1_OFR2);

    putreg32(ADC_OFR_OFFSETY((offset[2] + hal_adc_calibration_sample_round) >>
                             hal_adc_calibration_sample_count_log2) |
             ADC_OFR_OFFSETY_CH(HAL_ADC_PHASE_C_I_CHANNEL) | ADC_OFR_OFFSETY_EN,
             STM32_ADC1_OFR3);

    /* Read VBUS and temperature */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_ADSTART, STM32_ADC1_CR);
    while (!hal_adc_periodic_());

    /*
    Enable end-of-conversion interrupts and hardware triggering, then start
    converting
    */
    putreg32(getreg32(STM32_ADC1_IER) | ADC_INT_JEOS, STM32_ADC1_IER);
    putreg32(getreg32(STM32_ADC1_JSQR) | ADC_JSQR_JEXTEN_RISING,
             STM32_ADC1_JSQR);
}


static void hal_pwm_periodic_() {
    if (getreg16(STM32_TIM2_SR) & GTIM_SR_CC1IF) {
        // if (pwm_receive_task_) {
        //     pwm_receive_task_(getreg16(STM32_TIM2_CCR2),
        //                       getreg16(STM32_TIM2_CCR1));
        // }
    }
}


static bool hal_adc_periodic_() {
    hal_pwm_periodic_();

    /* TS_CAL_1 is the temperature sensor reading at 30 deg C */
    static uint16_t ts_cal_1 = *((volatile uint16_t*)(0x1FFFF7B8));
    /* TS_CAL_2 is the temperature sensor reading at 110 deg C */
    static uint16_t ts_cal_2 = *((volatile uint16_t*)(0x1FFFF7C2));
    static float ts_deg_c_per_lsb = (110.0f - 30.0f) /
                                    float(ts_cal_2 - ts_cal_1);
    float temp;

    /* Check if the last VBUS and temperature conversion is done */
    if (getreg32(STM32_ADC1_CR) & ADC_CR_ADSTART) {
        return false;
    }

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
    temp = float(board_vbus_lsb_) *
           float(hal_full_scale_voltage_v / (32768.0 * 16.0));
    vbus_v_ = temp;
    vbus_inv_ = 0.0f;
    if (temp > 6.0f) {
        vbus_inv_ = 32768.0f / temp;
    }

    if (board_temp_lsb_ > 0) {
        board_temp_lsb_ = (board_temp_lsb_ * 63 +
                           (adc_conversion_results_[1] << 7)) >> 6;
    } else {
        board_temp_lsb_ = adc_conversion_results_[1] << 7;
    }
    temp = 30.0f +
           float(int32_t(board_temp_lsb_ >> 7) - int32_t(ts_cal_1)) *
           ts_deg_c_per_lsb;
    temp_degc_ = temp;

    /* Start a new conversion */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_ADSTART, STM32_ADC1_CR);

    return true;
}


void hal_reset(void) {
    /* Clear out state */
    pwm_state_ = HAL_PWM_STATE_OFF;
    board_vbus_lsb_ = 0;

    up_cxxinitialize();
    board_initialize();

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

    /* Initialize timer, DMA and ADC */
    hal_init_tim_();
    hal_init_dma_();
    hal_init_adc_();

    /* Set up the PWM outputs -- TIM1 MOE is disabled */
    stm32_configgpio(GPIO_PWMA_ENABLED);
    stm32_configgpio(GPIO_PWMB_ENABLED);
    stm32_configgpio(GPIO_PWMC_ENABLED);

    hal_init_drv8305_();

    /* Calibrate the current shunt sensor offsets */
    hal_run_calibration_();

    up_prioritize_irq(STM32_IRQ_ADC12, NVIC_SYSH_PRIORITY_MAX);
    up_enable_irq(STM32_IRQ_ADC12);
    /* Perform timer initialization -- moved from NuttX __start */
    up_timer_initialize();

    /* Enable IRQs to make sure we get the next interrupt */
    irqenable();

    /* Trigger an ADC conversion to start the high-frequency task */
    putreg32(getreg32(STM32_ADC1_CR) | ADC_CR_JADSTART, STM32_ADC1_CR);
}


void hal_set_pwm_state(enum hal_pwm_state_t state) {
    if (state == pwm_state_) {
        /* No change -- ignore */
    } else if (state == HAL_PWM_STATE_OFF || state == HAL_PWM_STATE_LOW) {
        /* Clear BDTR to force all outputs low */
        putreg32(0u, STM32_TIM1_BDTR);
    } else if (state == HAL_PWM_STATE_RUNNING) {
        /* Enable main timer output */
        putreg32(ATIM_BDTR_MOE, STM32_TIM1_BDTR);
    }

    pwm_state_ = state;
}
