/******************************************************************************

 @file  urfc.h

 @brief This file contains configurable variables for the Micro BLE Stack
        radio such as type of RF Front End used with the TI device,
        TX power table, etc. Please see below for more detail.

        In general, the configuration values vary by device type, device version
        and board characteristics. The definitions and values listed in
        urfcfg.h and urfcfg.c can be changed or more sets of definitions and
        values can be added, depending on which version/type of device and what
        characteristics the board has.

        Note: User configurable variables except the elements of
              the power table are only used during the initialization
              of the MAC. Changing the values of these variables
              except power table elements after this will have no
              effect.

 Group: WCS BTS
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

#ifndef URFC_H
#define URFC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include <hal_types.h>
#if !defined(CC26XX_R2) && !defined (CC13X2R1_LAUNCHXL) && !defined(CC26X2R1_LAUNCHXL)
  #include <rf_patches/rf_patch_cpe_ble.h>
  #include <rf_patches/rf_patch_rfe_ble.h>
#endif /* !CC26XX_R2 */
#include <ti/drivers/rf/RF.h>

/*******************************************************************************
 * MACROS
 */

/* TX power table calculation */
//     15..8   | 7..6 | 5..0
//   tempCoeff |  GC  |  IB
//
#define BLE_TX_POUT( IB, GC, TC )                                                  \
  (uint16)((((TC) & 0xFF) << 8) | (((GC) & 0x03) << 6) | ((IB) & 0x3F))

/*******************************************************************************
 * CONSTANTS
 */

// RF Front End Settings
// Note: The use of these values completely depends on how the PCB is laid out.
//       Please see Device Package and Evaluation Module (EM) Board below.
/* RF Front End Settings */
#define RF_FRONTEND_MODE_MASK            0x07
#define RF_FE_DIFFERENTIAL               0
#define RF_FE_SINGLE_ENDED_RFP           1
#define RF_FE_SINGLE_ENDED_RFN           2
#define RF_FE_ANT_DIVERSITY_RFP_FIRST    3
#define RF_FE_ANT_DIVERSITY_RFN_FIRST    4
#define RF_FE_SINGLE_ENDED_RFP_EXT_PINS  5
#define RF_FE_SINGLE_ENDED_RFN_EXT_PINS  6

#define RF_BIAS_MODE_MASK                0x08
#define RF_FE_INT_BIAS                   (0<<3)
#define RF_FE_EXT_BIAS                   (1<<3)

#if defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL) || defined(CC2650DK_7ID) || defined(CC13XX_LAUNCHXL)
// RF Front End Mode and Bias Configuration
#define RF_FE_MODE_AND_BIAS (RF_FE_DIFFERENTIAL | RF_FE_EXT_BIAS)
// RF FE IOD
#define RF_FE_IOD_NUM     3
#define RF_FE_IOD         {NULL, NULL, NULL}
#define RF_FE_IOD_VAL     {NULL, NULL, NULL}
// RF FE saturation and sensitivity
#define RF_FE_SATURATION  0
#define RF_FE_SENSITIVITY 0

/* Tx Power numbers */
#define TX_POWER_5_DBM                   5
#define TX_POWER_4_DBM                   4
#define TX_POWER_3_DBM                   3
#define TX_POWER_2_DBM                   2
#define TX_POWER_1_DBM                   1
#define TX_POWER_0_DBM                   0
#define TX_POWER_MINUS_3_DBM             -3
#define TX_POWER_MINUS_6_DBM             -6
#define TX_POWER_MINUS_9_DBM             -9
#define TX_POWER_MINUS_12_DBM            -12
#define TX_POWER_MINUS_15_DBM            -15
#define TX_POWER_MINUS_18_DBM            -18
#define TX_POWER_MINUS_21_DBM            -21
#endif /* CC2650_LAUNCHXL || CC2640R2_LAUNCHXL || CC2650DK_7ID */

//
// Device Package and Evaluation Module (EM) Board
//
// The device may come in more than one types of packages.
// For each package, the user may change how the RF Front End (FE) is
// configured. The possible FE settings are provided as a set of defines.
// (The user can also set the FE bias, the settings of which are also provided
// as defines.) The user can change the value of RF_FE_MODE_AND_BIAS to
// configure the RF FE as desired. However, while setting the FE configuration
// determines how the device is configured at the package, it is the PCB the
// device is mounted on (the EM) that determines how those signals are routed.
// So while the FE is configurable, how signals are used is fixed by the EM.
// As can be seen, the value of RF_FE_MODE_AND_BIAS is organized by the EM
// board as defined by EMs produced by Texas Instruments Inc. How the device
// is mounted, routed, and configured for a user product would of course be
// user defined, and the value of RF_FE_MODE_AND_BIAS would have to be set
// accordingly; the user could even dispense with the conditional board
// compiles entirely. So too with the usage of the Tx Power tables.

/*******************************************************************************
 * TYPEDEFS
 */

PACKED_TYPEDEF_CONST_STRUCT
{
  int8   dBm;
  uint16 txPowerVal;
} ubTxPowerVal_t;

PACKED_TYPEDEF_CONST_STRUCT
{
  ubTxPowerVal_t* pTxPowerVals;
  uint8           numTxPowerVal;
} ubTxPowerTable_t;

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/* RF Core patch */
extern const RF_Mode          ubRfMode;
/* Overrides for CMD_RADIO_SETUP */
extern       uint32           ubRfRegOverride[];
/* TX Power table */
extern const ubTxPowerTable_t ubTxPowerTable;
/* RF frontend mode bias */
extern const uint8            ubFeModeBias;

/*********************************************************************
 * FUNCTIONS
 */

#ifdef __cplusplus
}
#endif

#endif /* URFC_H */
