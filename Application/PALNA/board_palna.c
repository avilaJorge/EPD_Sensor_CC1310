/******************************************************************************

 @file board_palna.c

 @brief This file contains the interface to the CC1310-CC1190 LP PA/LNA

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
 Includes
 *****************************************************************************/

#include <xdc/std.h>

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

#include "board.h"
#include "board_palna.h"

/******************************************************************************
 Constants
 *****************************************************************************/

/******************************************************************************
 Typedefs
 *****************************************************************************/

/******************************************************************************
 Local Variables
 *****************************************************************************/

/*
 SensorTag LED has exactly the same attributes as that of
 BoardGpioInitTable[]. There is no need to create a new one.
 */
static PIN_Config palnaPinTable[] =
    {
        Board_DIO28_HGM | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL
            | PIN_DRVSTR_MAX, /* High Gain Mode by Default */
        Board_DIO29_LNA | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
            | PIN_DRVSTR_MAX, /* LNA Off by Default */
        Board_DIO30_PA  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL
            | PIN_DRVSTR_MAX, /* PA Off by Default */	
        PIN_TERMINATE /* Terminate list     */
    };

/* PA/LNA pin state */
static PIN_State palnaPinState;

/* PA/LNA Pin Handle */
static PIN_Handle palnaPinHandle;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize PA/LNA

 Public function defined in board_palna.h
 */
void Board_Palna_initialize(uint32_t hgm)
{
	if (hgm)
	{
		if (!palnaPinHandle)
		{
			/* Open PA/LNA PIN driver */
			palnaPinHandle = PIN_open(&palnaPinState, palnaPinTable);
			
			/* Set IO muxing for RFC GPOs */
			PINCC26XX_setMux(palnaPinHandle, Board_DIO29_LNA, IOC_PORT_RFC_GPO0);
			PINCC26XX_setMux(palnaPinHandle, Board_DIO30_PA, IOC_PORT_RFC_GPO1);
		}
		
		PIN_setOutputValue(palnaPinHandle, Board_DIO28_HGM, (hgm & 1));
			
	}
}



