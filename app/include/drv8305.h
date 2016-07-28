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

#pragma once

#define DRV8305_WARN_ADDR 0x1u
#define DRV8305_OV_VDS_FAULT_ADDR 0x2u
#define DRV8305_IC_FAULT_ADDR 0x3u
#define DRV8305_DRV_FAULT_ADDR 0x4u
#define DRV8305_HS_DRV_CTL_ADDR 0x5u
#define DRV8305_LS_DRV_CTL_ADDR 0x6u
#define DRV8305_DRV_CTL_ADDR 0x7u
#define DRV8305_IC_OP_ADDR 0x9u
#define DRV8305_SHUNT_CTL_ADDR 0xAu
#define DRV8305_VREG_CTL_ADDR 0xBu
#define DRV8305_VDS_CTL_ADDR 0xCu


#define DRV8305_TDRIVE_220ns     ( 0u <<  8u)
#define DRV8305_TDRIVE_440ns     ( 1u <<  8u)
#define DRV8305_TDRIVE_880ns     ( 2u <<  8u)
#define DRV8305_TDRIVE_1780ns    ( 3u <<  8u)

#define DRV8305_IDRIVEN_20mA     ( 0u <<  4u)
#define DRV8305_IDRIVEN_30mA     ( 1u <<  4u)
#define DRV8305_IDRIVEN_40mA     ( 2u <<  4u)
#define DRV8305_IDRIVEN_50mA     ( 3u <<  4u)
#define DRV8305_IDRIVEN_60mA     ( 4u <<  4u)
#define DRV8305_IDRIVEN_70mA     ( 5u <<  4u)
#define DRV8305_IDRIVEN_80mA     ( 6u <<  4u)
#define DRV8305_IDRIVEN_250mA    ( 7u <<  4u)
#define DRV8305_IDRIVEN_500mA    ( 8u <<  4u)
#define DRV8305_IDRIVEN_750mA    ( 9u <<  4u)
#define DRV8305_IDRIVEN_1000mA   (10u <<  4u)
#define DRV8305_IDRIVEN_1250mA   (11u <<  4u)

#define DRV8305_IDRIVEP_10mA     ( 0u <<  0u)
#define DRV8305_IDRIVEP_20mA     ( 1u <<  0u)
#define DRV8305_IDRIVEP_30mA     ( 2u <<  0u)
#define DRV8305_IDRIVEP_40mA     ( 3u <<  0u)
#define DRV8305_IDRIVEP_50mA     ( 4u <<  0u)
#define DRV8305_IDRIVEP_60mA     ( 5u <<  0u)
#define DRV8305_IDRIVEP_70mA     ( 6u <<  0u)
#define DRV8305_IDRIVEP_125mA    ( 7u <<  0u)
#define DRV8305_IDRIVEP_250mA    ( 8u <<  0u)
#define DRV8305_IDRIVEP_500mA    ( 9u <<  0u)
#define DRV8305_IDRIVEP_750mA    (10u <<  0u)
#define DRV8305_IDRIVEP_1000mA   (11u <<  0u)

#define DRV8305_FREEWHEEL_DIODE  ( 0u <<  9u)
#define DRV8305_FREEWHEEL_ACTIVE ( 0u <<  9u)
#define DRV8305_PWM_6IN          ( 0u <<  7u)
#define DRV8305_PWM_3IN          ( 1u <<  7u)
#define DRV8305_PWM_1IN          ( 2u <<  7u)
#define DRV8305_DEADTIME_35ns    ( 0u <<  4u)
#define DRV8305_DEADTIME_52ns    ( 1u <<  4u)
#define DRV8305_DEADTIME_88ns    ( 2u <<  4u)
#define DRV8305_DEADTIME_440ns   ( 3u <<  4u)
#define DRV8305_DEADTIME_880ns   ( 4u <<  4u)
#define DRV8305_DEADTIME_1760ns  ( 5u <<  4u)
#define DRV8305_DEADTIME_3520ns  ( 6u <<  4u)
#define DRV8305_DEADTIME_5280ns  ( 7u <<  4u)
#define DRV8305_TBLANK_0us       ( 0u <<  2u)
#define DRV8305_TBLANK_1p75us    ( 1u <<  2u)
#define DRV8305_TBLANK_3p5us     ( 2u <<  2u)
#define DRV8305_TBLANK_7us       ( 3u <<  2u)
#define DRV8305_TVDS_0us         ( 0u <<  0u)
#define DRV8305_TVDS_1p75us      ( 1u <<  0u)
#define DRV8305_TVDS_3p5us       ( 2u <<  0u)
#define DRV8305_TVDS_7us         ( 3u <<  0u)

#define DRV8305_FLIP_OTS_EN      ( 1u << 10u)
#define DRV8305_VPVDD_UVLO2_DIS  ( 1u <<  9u)
#define DRV8305_GDRV_FAULT_DIS   ( 1u <<  8u)
#define DRV8305_SNS_CLAMP_EN     ( 1u <<  7u)
#define DRV8305_WD_DLY_10ms      ( 0u <<  5u)
#define DRV8305_WD_DLY_20ms      ( 1u <<  5u)
#define DRV8305_WD_DLY_50ms      ( 2u <<  5u)
#define DRV8305_WD_DLY_100ms     ( 3u <<  5u)
#define DRV8305_SNS_OCP_DIS      ( 1u <<  4u)
#define DRV8305_WD_EN            ( 1u <<  3u)
#define DRV8305_SLEEP            ( 1u <<  2u)
#define DRV8305_CLR_FLTS         ( 1u <<  1u)
#define DRV8305_SET_VCPH_UV_4V6  ( 1u <<  0u)

#define DRV8305_DC_CAL_CH3       ( 1u << 10u)
#define DRV8305_DC_CAL_CH2       ( 1u <<  9u)
#define DRV8305_DC_CAL_CH1       ( 1u <<  8u)
#define DRV8305_CS_BLANK_0ns     ( 0u <<  6u)
#define DRV8305_CS_BLANK_500ns   ( 1u <<  6u)
#define DRV8305_CS_BLANK_2500ns  ( 2u <<  6u)
#define DRV8305_CS_BLANK_10000ns ( 3u <<  6u)
#define DRV8305_GAIN_CS3_10VpV   ( 0u <<  4u)
#define DRV8305_GAIN_CS3_20VpV   ( 1u <<  4u)
#define DRV8305_GAIN_CS3_40VpV   ( 2u <<  4u)
#define DRV8305_GAIN_CS3_80VpV   ( 3u <<  4u)
#define DRV8305_GAIN_CS2_10VpV   ( 0u <<  2u)
#define DRV8305_GAIN_CS2_20VpV   ( 1u <<  2u)
#define DRV8305_GAIN_CS2_40VpV   ( 2u <<  2u)
#define DRV8305_GAIN_CS2_80VpV   ( 3u <<  2u)
#define DRV8305_GAIN_CS1_10VpV   ( 0u <<  0u)
#define DRV8305_GAIN_CS1_20VpV   ( 1u <<  0u)
#define DRV8305_GAIN_CS1_40VpV   ( 2u <<  0u)
#define DRV8305_GAIN_CS1_80VpV   ( 3u <<  0u)

#define DRV8305_VREF_SCALE_2     ( 1u <<  8u)
#define DRV8305_VREF_SCALE_4     ( 2u <<  8u)
#define DRV8305_SLEEP_DLY_0us    ( 0u <<  3u)
#define DRV8305_SLEEP_DLY_10us   ( 1u <<  3u)
#define DRV8305_SLEEP_DLY_50us   ( 2u <<  3u)
#define DRV8305_SLEEP_DLY_1000us ( 3u <<  3u)
#define DRV8305_VREG_PWRGD_DIS   ( 1u <<  2u)
#define DRV8305_VREG_UV_10pct    ( 0u <<  0u)
#define DRV8305_VREG_UV_20pct    ( 1u <<  0u)
#define DRV8305_VREG_UV_30pct    ( 2u <<  0u)

#define DRV8305_VDS_THRES_60mV   ( 0u <<  3u)
#define DRV8305_VDS_THRES_68mV   ( 1u <<  3u)
#define DRV8305_VDS_THRES_76mV   ( 2u <<  3u)
#define DRV8305_VDS_THRES_86mV   ( 3u <<  3u)
#define DRV8305_VDS_THRES_97mV   ( 4u <<  3u)
#define DRV8305_VDS_THRES_109mV  ( 5u <<  3u)
#define DRV8305_VDS_THRES_123mV  ( 6u <<  3u)
#define DRV8305_VDS_THRES_138mV  ( 7u <<  3u)
#define DRV8305_VDS_THRES_155mV  ( 8u <<  3u)
#define DRV8305_VDS_THRES_175mV  ( 9u <<  3u)
#define DRV8305_VDS_THRES_197mV  (10u <<  3u)
#define DRV8305_VDS_THRES_222mV  (11u <<  3u)
#define DRV8305_VDS_THRES_250mV  (12u <<  3u)
#define DRV8305_VDS_THRES_282mV  (13u <<  3u)
#define DRV8305_VDS_THRES_317mV  (14u <<  3u)
#define DRV8305_VDS_THRES_358mV  (15u <<  3u)
#define DRV8305_VDS_THRES_403mV  (16u <<  3u)
#define DRV8305_VDS_THRES_454mV  (17u <<  3u)
#define DRV8305_VDS_THRES_511mV  (18u <<  3u)
#define DRV8305_VDS_THRES_576mV  (19u <<  3u)
#define DRV8305_VDS_THRES_648mV  (20u <<  3u)
#define DRV8305_VDS_THRES_730mV  (21u <<  3u)
#define DRV8305_VDS_THRES_822mV  (22u <<  3u)
#define DRV8305_VDS_THRES_926mV  (23u <<  3u)
#define DRV8305_VDS_THRES_1043mV (24u <<  3u)
#define DRV8305_VDS_THRES_1175mV (25u <<  3u)
#define DRV8305_VDS_THRES_1324mV (26u <<  3u)
#define DRV8305_VDS_THRES_1491mV (27u <<  3u)
#define DRV8305_VDS_THRES_1679mV (28u <<  3u)
#define DRV8305_VDS_THRES_1892mV (29u <<  3u)
#define DRV8305_VDS_THRES_2131mV (30u <<  3u)
#define DRV8305_VDS_MODE_SHDN    ( 0u <<  0u)
#define DRV8305_VDS_MODE_REPORT  ( 1u <<  0u)
#define DRV8305_VDS_MODE_IGNORE  ( 2u <<  0u)
