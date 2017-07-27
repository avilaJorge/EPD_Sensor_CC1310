/******************************************************************************

 @file  mac_user_config.c

 @brief User configurable variables for the TIMAC radio.

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

/******************************************************************************
 * INCLUDES
 */


#if defined(CC13XX_LAUNCHXL)
#include "CC1350_LAUNCHXL/Board.h"
#elif defined(CC26XX_LAUNCHXL)
#include "CC2650_LAUNCHXL/Board.h"
#elif defined(CC13X2R1_LAUNCHXL)
#include "CC1352R1_LAUNCHXL/Board.h"
#else
/* This is only other platform to be supported */
#include "CC1310DK_7XD/Board.h"
#endif
#include "mac_user_config.h"
#include <ti/drivers/crypto/CryptoCC26XX.h>

/******************************************************************************
 * MACROS
 */

/******************************************************************************
 * CONSTANTS
 */

// Tx Power
#define NUM_TX_POWER_VALUES( x )  (sizeof( x ) / sizeof( txPwrVal_t ))

/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * GLOBAL VARIABLES
 */
#if defined (CC13XX_LAUNCHXL)
//
// Tx Power Table Used Depends on Device Package
//
#if defined( COPROCESSOR )
#include "mac_user_config_cc13x0_subg_cop.h"
#else // other projects
#if defined( US_BAND_1 ) || defined( ETSI_BAND )
#include "mac_user_config_cc13x0_subg_us_euro.h"
#elif defined( CHINA_BAND )
#include "mac_user_config_cc13x0_subg_china.h"
#else /* phy id is something else not defined */
#error "***MAC USER CONFIG BUILD ERROR*** Unknown phy id!"
#endif
#endif
#else
#include "mac_user_config_cc13x2r1_subg.h"
#endif

// RF Driver API Table
const uint32_t rfDriverTable[] =
  { (uint32_t) RF_open,
    (uint32_t) RF_close,
    (uint32_t) RF_postCmd,
	(uint32_t) NULL,
	(uint32_t) NULL,
    (uint32_t) RF_cancelCmd,
    (uint32_t) RF_flushCmd,
    (uint32_t) RF_yield,
    (uint32_t) RF_Params_init,
    (uint32_t) RF_runImmediateCmd,
    (uint32_t) RF_runDirectCmd,
    (uint32_t) RF_ratCompare,
	(uint32_t) NULL,
	(uint32_t) NULL,
    (uint32_t) RF_ratDisableChannel,
    (uint32_t) RF_getCurrentTime,
    (uint32_t) RF_getRssi,
    (uint32_t) RF_getInfo,
    (uint32_t) RF_getCmdOp,
    (uint32_t) RF_scheduleCmd };

// Crypto Driver API Table
const uint32_t cryptoDriverTable[] =
  { (uint32_t) CryptoCC26XX_close,
    (uint32_t) CryptoCC26XX_init,
    (uint32_t) CryptoCC26XX_open,
    (uint32_t) CryptoCC26XX_Params_init,
    (uint32_t) CryptoCC26XX_Transac_init,
    (uint32_t) CryptoCC26XX_allocateKey,
    (uint32_t) CryptoCC26XX_releaseKey,
    (uint32_t) CryptoCC26XX_transact,
    (uint32_t) CryptoCC26XX_transactPolling,
    (uint32_t) CryptoCC26XX_transactCallback };

/******************************************************************************
 */

