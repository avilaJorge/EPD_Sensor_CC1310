/******************************************************************************

 @file  board.h

 @brief Simple gateway to the real Board.h file
        Based on include path, corresponding board.c will also be included

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

#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif
  
#if defined(CC13XX_LAUNCHXL)
/** ============================================================================
 *  @file  board.h
 *
 *  @brief      CC1350 Launchpad Specific header file.
 *              The project options should point to this file if this is the
 *              CC1350 you are developing code for.
 *
 *  The CC1350 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
/* CC1350 board file will be used for both CC1310 and CC1350 */
#include "boards/CC1350_LAUNCHXL/Board.h"
#elif defined(CC26XX_LAUNCHXL)
/** ============================================================================
 *  @file  board.h
 *
 *  @brief      CC2650 Launchpad Specific header file.
 *              The project options should point to this file if this is the
 *              CC2650 you are developing code for.
 *
 *  The CC2650 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#include "boards/CC2650_LAUNCHXL/Board.h"
#elif defined(CC26X2R1_LAUNCHXL)
/** ============================================================================
 *  @file  board.h
 *
 *  @brief      CC26X2R1 Launchpad Specific header file.
 *              The project options should point to this file if this is the
 *              CC26X2R1 you are developing code for.
 *
 *  The CC26X2R1 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#include "boards/CC26X2R1_LAUNCHXL/Board.h"
#elif defined(CC13X2R1_LAUNCHXL)
/** ============================================================================
 *  @file  board.h
 *
 *  @brief      CC13X2R1 Launchpad Specific header file.
 *              The project options should point to this file if this is the
 *              CC13X2R1 you are developing code for.
 *
 *  The CC13X2R1 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#include "boards/CC1352R1_LAUNCHXL/Board.h"
#else
/** ============================================================================
 *  @file  board.h
 *
 *  @brief      CC1350EM Board Specific header file.
 *              The project options should point to this file if this is the
 *              CC1350EM you are developing code for.
 *
 *  The CC1350 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#include "boards/CC1310DK_7XD/Board.h"
#endif
#endif /* BOARD_H */
