/******************************************************************************

 @file mac_user_config_cc13x0_subg_us_euro.h

 @brief override and power table

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc13x0_sdk_1_40_00_10"
 Release Date: 2017-06-27 19:06:01
 *****************************************************************************/
#ifndef _CONFIG_CC13X0_SUBG_US_EURO_H
#define _CONFIG_CC13X0_SUBG_US_EURO_H

/******************************************************************************
 Includes
 *****************************************************************************/

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/******************************************************************************
 External Variables
 *****************************************************************************/

/******************************************************************************
 Global Variables
 *****************************************************************************/

#ifndef SL_LRM /* regular mode */
//
// RF patch pointers
//
const RF_Mode rfPropTable =
{
#ifdef RF_MULTI_MODE
  .rfMode = RF_MODE_MULTIPLE,
#else
  .rfMode = RF_MODE_PROPRIETARY_SUB_1,
#endif
  .cpePatchFxn = &rf_patch_cpe_genfsk,
  .mcePatchFxn = 0,
  .rfePatchFxn = &rf_patch_rfe_genfsk,
};
// Overrides for CMD_PROP_RADIO_DIV_SETUP
#if !CONFIG_RANGE_EXT_MODE
// US-EU Band, GFSK, no CC1190
static uint32_t pOverridesPG21[] =
{
    // override_use_patch_prop_genfsk.xml
    // PHY: Use MCE ROM bank 4, RFE RAM patch
    MCE_RFE_OVERRIDE(0,4,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 3
    (uint32_t)0x00038883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x1a.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x1A.
    HW_REG_OVERRIDE(0x6088,0x411A),
    // Tx: Configure PA ramping setting and RSSI measurement period of 34us. This may change for different data rate.
    HW_REG_OVERRIDE(0x608C,0x8223),  // Edited for TIMAC
    // override_phy_rx_rssi_offset_5db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +5 dB
    (uint32_t)0x00FB88A3,
    // TX power override
#if CCFG_FORCE_VDDR_HH
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
#endif
    (uint32_t)0x00000943, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0x00000963, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0xFFFFFFFF,
};
#else
// US-EU Band, GFSK, CC1190
static uint32_t pOverridesPG21[] =
{
    // override_use_patch_prop_genfsk.xml
    // PHY: Use MCE ROM bank 4, RFE RAM patch
    MCE_RFE_OVERRIDE(0,4,0,1,0,0),
    // override_synth_prop_863_930_div5.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x0A480583,
    // Synth: Set loop bandwidth after lock to 20 kHz
    (uint32_t)0x7AB80603,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 3
    (uint32_t)0x00038883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x1a.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x1A.
    HW_REG_OVERRIDE(0x6088,0x411A),
    // Tx: Configure PA ramping setting and RSSI measurement period of 34us. This may change for different data rate.
    HW_REG_OVERRIDE(0x608C,0x8223),  // Edited for TIMAC
    // override_phy_rx_rssi_offset_27db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +27 dB
    (uint32_t)0x000388A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    (uint32_t)0x00000943, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0x00000963, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0xFFFFFFFF,
};
#endif
#else /* long range mode */
//
// RF patch pointers
//
const RF_Mode rfPropTable =
{
#ifdef RF_MULTI_MODE
  .rfMode = RF_MODE_MULTIPLE,
#else
  .rfMode = RF_MODE_PROPRIETARY_SUB_1,
#endif
  .cpePatchFxn = &rf_patch_cpe_sl_longrange,
  .mcePatchFxn = &rf_patch_mce_sl_longrange,
  .rfePatchFxn = &rf_patch_rfe_sl_longrange,
};

// Overrides for CMD_PROP_RADIO_DIV_SETUP
#if !CONFIG_RANGE_EXT_MODE
// US-EU Band, SLRM, no CC1190
static uint32_t pOverridesPG21[] =
{
    // override_use_patch_simplelink_long_range.xml
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_863_930_div5_lbw60k.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x40410583,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x32CC0603,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x00010623,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_synth_disable_bias_div5.xml
    // Synth: Set divider bias to disabled
    //HW32_ARRAY_OVERRIDE(0x405C,1),
    // Synth: Set divider bias to disabled (specific for loDivider=5)
    //(uint32_t)0x18000200,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    //ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 3
    (uint32_t)0x00038883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x14.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x14.
    HW_REG_OVERRIDE(0x6088,0x4114),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_long_range_dsss2.xml
    // PHY: Configure DSSS SF=2
    HW_REG_OVERRIDE(0x505C,0x0100),
    // override_phy_rx_rssi_offset_5db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +5 dB
    (uint32_t)0x00FB88A3,
    // TX power override
#if CCFG_FORCE_VDDR_HH
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
#endif
    (uint32_t)0x00000943, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0x00000963, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0xFFFFFFFF,
};
#else
// US-EU Band, SLRM, CC1190
static uint32_t pOverridesPG21[] =
{
    // override_use_patch_simplelink_long_range.xml
    // PHY: Use MCE RAM patch, RFE RAM patch
    MCE_RFE_OVERRIDE(1,0,0,1,0,0),
    // override_synth_prop_863_930_div5_lbw60k.xml
    // Synth: Set recommended RTRIM to 7
    HW_REG_OVERRIDE(0x4038,0x0037),
    // Synth: Set Fref to 4 MHz
    (uint32_t)0x000684A3,
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4020,0x7F00),
    // Synth: Configure fine calibration setting
    HW_REG_OVERRIDE(0x4064,0x0040),
    // Synth: Configure fine calibration setting
    (uint32_t)0xB1070503,
    // Synth: Configure fine calibration setting
    (uint32_t)0x05330523,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x40410583,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x32CC0603,
    // Synth: Set loop bandwidth after lock to 60 kHz
    (uint32_t)0x00010623,
    // Synth: Configure VCO LDO (in ADI1, set VCOLDOCFG=0x9F to use voltage input reference)
    ADI_REG_OVERRIDE(1,4,0x9F),
    // Synth: Configure synth LDO (in ADI1, set SLDOCTL0.COMP_CAP=1)
    ADI_HALFREG_OVERRIDE(1,7,0x4,0x4),
    // Synth: Use 24 MHz XOSC as synth clock, enable extra PLL filtering
    (uint32_t)0x02010403,
    // Synth: Configure extra PLL filtering
    (uint32_t)0x00108463,
    // Synth: Increase synth programming timeout (0x04B0 RAT ticks = 300 us)
    (uint32_t)0x04B00243,
    // override_synth_disable_bias_div5.xml
    // Synth: Set divider bias to disabled
    //HW32_ARRAY_OVERRIDE(0x405C,1),
    // Synth: Set divider bias to disabled (specific for loDivider=5)
    //(uint32_t)0x18000200,
    // override_phy_rx_aaf_bw_0xd.xml
    // Rx: Set anti-aliasing filter bandwidth to 0xD (in ADI0, set IFAMPCTL3[7:4]=0xD)
    //ADI_HALFREG_OVERRIDE(0,61,0xF,0xD),
    // override_phy_gfsk_rx.xml
    // Rx: Set LNA bias current trim offset to 3
    (uint32_t)0x00038883,
    // Rx: Freeze RSSI on sync found event
    HW_REG_OVERRIDE(0x6084,0x35F1),
    // override_phy_gfsk_pa_ramp_agc_reflevel_0x14.xml
    // Tx: Configure PA ramping setting (0x41). Rx: Set AGC reference level to 0x14.
    HW_REG_OVERRIDE(0x6088,0x4114),
    // Tx: Configure PA ramping setting
    HW_REG_OVERRIDE(0x608C,0x8213),
    // override_phy_long_range_dsss2.xml
    // PHY: Configure DSSS SF=2
    HW_REG_OVERRIDE(0x505C,0x0100),
    // override_phy_rx_rssi_offset_27db.xml
    // Rx: Set RSSI offset to adjust reported RSSI by +27 dB
    (uint32_t)0x000388A3,
    // TX power override
    // Tx: Set PA trim to max (in ADI0, set PACTL0=0xF8)
    ADI_REG_OVERRIDE(0,12,0xF8),
    (uint32_t)0x00000943, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0x00000963, // Correct CRC initialization for 16-bit CRC
    (uint32_t)0xFFFFFFFF,
};
#endif
#endif
// Tx Power Values (Pout, TC, GC, IB)
// 1st column is for CC1310 and 2nd column is for CC1350
// This table can apply to both EM and Launch Pad
#if !CONFIG_RANGE_EXT_MODE
const txPwrVal_t txPowerTable_cc1310[] =
  { { TX_POWER_14_DBM,       TX_POUT( 0xA7, 0, 0x3F ) },
    { TX_POWER_12_DBM,       TX_POUT( 0xB8, 0, 0x18 ) },
    { TX_POWER_11_DBM,       TX_POUT( 0x50, 3, 0x1A ) },
    { TX_POWER_10_DBM,       TX_POUT( 0x38, 3, 0x13 ) },
    { TX_POWER_9_DBM,        TX_POUT( 0x2C, 3, 0x0D ) },
    { TX_POWER_8_DBM,        TX_POUT( 0x24, 3, 0x0B ) },
    { TX_POWER_7_DBM,        TX_POUT( 0x20, 3, 0x09 ) },
    { TX_POWER_6_DBM,        TX_POUT( 0x1C, 3, 0x07 ) },
    { TX_POWER_5_DBM,        TX_POUT( 0x18, 3, 0x06 ) },
    { TX_POWER_4_DBM,        TX_POUT( 0x18, 3, 0x05 ) },
    { TX_POWER_3_DBM,        TX_POUT( 0x14, 3, 0x04 ) },
    { TX_POWER_2_DBM,        TX_POUT( 0x10, 1, 0x02 ) },
    { TX_POWER_1_DBM,        TX_POUT( 0x10, 3, 0x03 ) },
    { TX_POWER_0_DBM,        TX_POUT( 0x00, 1, 0x01 ) },
    { TX_POWER_MINUS_10_DBM, TX_POUT( 0x08, 3, 0x00 ) } };

const txPwrVal_t txPowerTable_cc1350[] =
  { { TX_POWER_14_DBM,       TX_POUT( 0xAB, 0, 0x3F ) },
    { TX_POWER_12_DBM,       TX_POUT( 0xBC, 0, 0x2B ) },
    { TX_POWER_11_DBM,       TX_POUT( 0x90, 3, 0x25 ) },
    { TX_POWER_10_DBM,       TX_POUT( 0x58, 3, 0x18 ) },
    { TX_POWER_9_DBM,        TX_POUT( 0x40, 3, 0x12 ) },
    { TX_POWER_8_DBM,        TX_POUT( 0x32, 3, 0x0E ) },
    { TX_POWER_7_DBM,        TX_POUT( 0x2A, 3, 0x0B ) },
    { TX_POWER_6_DBM,        TX_POUT( 0x24, 3, 0x09 ) },
    { TX_POWER_5_DBM,        TX_POUT( 0x20, 3, 0x08 ) },
    { TX_POWER_4_DBM,        TX_POUT( 0x18, 1, 0x04 ) },
    { TX_POWER_3_DBM,        TX_POUT( 0x1C, 3, 0x06 ) },
    { TX_POWER_2_DBM,        TX_POUT( 0x18, 3, 0x05 ) },
    { TX_POWER_1_DBM,        TX_POUT( 0x16, 3, 0x04 ) },
    { TX_POWER_0_DBM,        TX_POUT( 0x12, 3, 0x03 ) },
    { TX_POWER_MINUS_10_DBM, TX_POUT( 0x04, 3, 0x00 ) } };

// Tx Power Table
// first for CC1310 and second for CC1350
// if only one type of chip, fill the same power table for the 2nd array.
// for example, if only CC1310,
//const txPwrTbl_t txPwrTbl[] =
//    { { txPowerTable_cc1310, NUM_TX_POWER_VALUES( txPowerTable_cc1310 ) },
//      { txPowerTable_cc1310, NUM_TX_POWER_VALUES( txPowerTable_cc1310 ) } };
const txPwrTbl_t txPwrTbl[] =
    { { txPowerTable_cc1310, NUM_TX_POWER_VALUES( txPowerTable_cc1310 ) },    /* power table for CC1310 */
      { txPowerTable_cc1350, NUM_TX_POWER_VALUES( txPowerTable_cc1350 ) } };  /* power table for CC1350 */
#else
const txPwrVal_t txPowerTable_cc1310_cc1190[] =
  { { TX_POWER_27_DBM,       TX_POUT( 0x00, 3, 0x05 ) },
    { TX_POWER_26_DBM,       TX_POUT( 0x00, 3, 0x04 ) },
    { TX_POWER_25_DBM,       TX_POUT( 0x00, 3, 0x02 ) },
    { TX_POWER_23_DBM,       TX_POUT( 0x00, 3, 0x01 ) },
    { TX_POWER_18_DBM,       TX_POUT( 0x00, 3, 0x00 ) } };

// Tx Power Table
// first for CC1310 and second for CC1350
// if only one type of chip, fill the same power table for the 2nd array.
// for example, if only CC1310,
//const txPwrTbl_t txPwrTbl[] =
//    { { txPowerTable_cc1310, NUM_TX_POWER_VALUES( txPowerTable_cc1310 ) },
//      { txPowerTable_cc1310, NUM_TX_POWER_VALUES( txPowerTable_cc1310 ) } };
const txPwrTbl_t txPwrTbl[] =
    { { txPowerTable_cc1310_cc1190, NUM_TX_POWER_VALUES( txPowerTable_cc1310_cc1190 ) },    /* power table for CC1310 */
      { txPowerTable_cc1310_cc1190, NUM_TX_POWER_VALUES( txPowerTable_cc1310_cc1190 ) } };  /* power table for CC1350 */
#endif
const uint8_t loDivider = MAC_LO_DIVIDER_5;

// RF Override Table
// PG2.1 and PG2.0 uses the same override table
const uint32_t *pOverridesTable[] =
    { (uint32_t *) pOverridesPG21 };    /* overrides for PG2.1 */

#endif
