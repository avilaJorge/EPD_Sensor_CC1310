/******************************************************************************

 @file ssf.c

 @brief Sensor Specific Functions

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
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/PIN.h>
#include <string.h>
#include <inc/hw_ints.h>
#include <aon_event.h>
#include <ioc.h>
#include <driverlib/aon_batmon.h>

#include "board.h"
#include "timer.h"
#include "util.h"
#include "board_key.h"
#include "board_lcd.h"
#include "board_led.h"

#include "macconfig.h"

#ifdef ONE_PAGE_NV
#include "nvocop.h"
#else
#include "nvoctp.h"
#endif

#include "icall.h"
#include "sensor.h"
#include "smsgs.h"
#include "ssf.h"
#include "config.h"

#if CONFIG_BLE_SUPPORT
#include "bcomdef.h"
#include "uble.h"
#include "ugap.h"
#include "gap.h"
#endif

#ifdef FEATURE_OAD
#include <Board.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include "uart_printf.h"
#include "oad_switch.h"
#endif

#ifdef FEATURE_NATIVE_OAD
#include <sensor_oad/oad_client.h>
#endif //FEATURE_NATIVE_OAD

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

/* Initial timeout value for the reading clock */
#define READING_INIT_TIMEOUT_VALUE 100

/* SSF Events */
#define KEY_EVENT 0x0001

#define UEB_EVT_MICROBLESTACK 0x002

/* NV Item ID - the device's network information */
#define SSF_NV_NETWORK_INFO_ID 0x0001
/* NV Item ID - the number of black list entries */
#define SSF_NV_BLACKLIST_ENTRIES_ID 0x0002
/* NV Item ID - the black list, use sub ID for each record in the list */
#define SSF_NV_BLACKLIST_ID 0x0003
/* NV Item ID - this devices frame counter */
#define SSF_NV_FRAMECOUNTER_ID 0x0004
/* NV Item ID - Assert reset reason */
#define SSF_NV_RESET_REASON_ID 0x0005
/* NV Item ID - Number of resets */
#define SSF_NV_RESET_COUNT_ID 0x0006
/* NV Item ID - OAD information */
#define SSF_NV_OAD_ID 0x0007

/* Maximum number of black list entries */
#define SSF_MAX_BLACKLIST_ENTRIES 10

/*
 Maximum sub ID for a blacklist item, this is failsafe.  This is
 not the maximum number of items in the list
 */
#define SSF_MAX_BLACKLIST_IDS 500

/* NV Item ID - the device's network information */
#define SSF_NV_NETWORK_INFO_ID 0x0001
/* NV Item ID - Config information */
#define SSF_NV_CONFIG_INFO_ID  0x0002

/* timeout value for trickle timer initialization */
#define TRICKLE_TIMEOUT_VALUE       30000

/* timeout value for poll timer initialization */
#define POLL_TIMEOUT_VALUE          30000

#define FH_ASSOC_TIMER              2000

/* timeout value for poll timer initialization */
#define SCAN_BACKOFF_TIMEOUT_VALUE  60000

/*! NV driver item ID for reset reason */
#define NVID_RESET {NVINTF_SYSID_APP, SSF_NV_RESET_REASON_ID, 0}

/*! Additional Random Delay for Association */
#define ADD_ASSOCIATION_RANDOM_WINDOW 10000
/*
 The increment value needed to save a frame counter. Example, setting this
 constant to 100, means that the frame counter will be saved when the new
 frame counter is 100 more than the last saved frame counter.  Also, when
 the get frame counter function reads the value from NV it will add this value
 to the read value.
 */
#define FRAME_COUNTER_SAVE_WINDOW     25

/* Structure to store the device information in NV */
typedef struct
{
    ApiMac_deviceDescriptor_t device;
    Llc_netInfo_t parent;
} nvDeviceInfo_t;

/******************************************************************************
 External variables
 *****************************************************************************/
#ifdef NV_RESTORE
/*! MAC Configuration Parameters */
extern mac_Config_t Main_user1Cfg;
#endif

/******************************************************************************
 Public variables
 *****************************************************************************/

/*!
 Assert reason for the last reset -  0 - no reason, 2 - HAL/ICALL,
 3 - MAC, 4 - TIRTOS
 */
uint8_t Ssf_resetReseason = 0;

/*! Number of times the device has reset */
uint16_t Ssf_resetCount = 0;

/******************************************************************************
 Local variables
 *****************************************************************************/

/* The application's semaphore */
static ICall_Semaphore sensorSem;

/* Clock/timer resources */
static Clock_Struct readingClkStruct;
static Clock_Handle readingClkHandle;

/* Clock/timer resources for JDLLC */
/* trickle timer */
STATIC Clock_Struct tricklePASClkStruct;
STATIC Clock_Handle tricklePASClkHandle;
STATIC Clock_Struct tricklePCSClkStruct;
STATIC Clock_Handle tricklePCSClkHandle;
/* poll timer */
STATIC Clock_Struct pollClkStruct;
STATIC Clock_Handle pollClkHandle;
/* scan backoff timer */
STATIC Clock_Struct scanBackoffClkStruct;
STATIC Clock_Handle scanBackoffClkHandle;
/* FH assoc delay */
STATIC Clock_Struct fhAssocClkStruct;
STATIC Clock_Handle fhAssocClkHandle;

/* Key press parameters */
static uint8_t keys;

/* pending events */
static uint16_t events = 0;

/* NV Function Pointers */
static NVINTF_nvFuncts_t *pNV = NULL;

/* The last saved frame counter */
static uint32_t lastSavedFrameCounter = 0;

/*! NV driver item ID for reset reason */
static const NVINTF_itemID_t nvResetId = NVID_RESET;

static bool started = false;

static bool led1State = false;

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static void processReadingTimeoutCallback(UArg a0);
static void processKeyChangeCallback(uint8_t keysPressed);
static void processPCSTrickleTimeoutCallback(UArg a0);
static void processPASTrickleTimeoutCallback(UArg a0);
static void processPollTimeoutCallback(UArg a0);
static void processScanBackoffTimeoutCallback(UArg a0);
static void processFHAssocTimeoutCallback(UArg a0);
static int findUnuedBlackListIndex(void);
static uint16_t getNumBlackListEntries(void);
static void saveNumBlackListEntries(uint16_t numEntries);

/******************************************************************************
 Public Functions
 *****************************************************************************/
extern void uBLEInit(void);
extern void MicroEddystoneBeacon_initURL(void);

/*!
 The application calls this function during initialization

 Public function defined in ssf.h
 */
void Ssf_init(void *sem)
{
#ifdef FEATURE_NATIVE_OAD
    OADClient_Params_t OADClientParams;
#endif //FEATURE_NATIVE_OAD

#ifdef NV_RESTORE
    /* Save off the NV Function Pointers */
    pNV = &Main_user1Cfg.nvFps;
#endif

    /* Save off the semaphore */
    sensorSem = sem;

    /* Initialize PA/LNA if enabled */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_rangeExtender,
                           (uint8_t)CONFIG_RANGE_EXT_MODE);

    /* Initialize keys */
    if(Board_Key_initialize(processKeyChangeCallback) == KEY_RIGHT)
    {
        /* Right key is pressed on power up, clear all NV */
        Ssf_clearAllNVItems();
    }

#ifndef POWER_MEAS
    /* Initialize the LCD */
    Board_LCD_open();

    /* Initialize the LEDs */
    Board_Led_initialize();
#endif

    if((pNV != NULL) && (pNV->readItem != NULL))
    {
        /* Attempt to retrieve reason for the reset */
        (void)pNV->readItem(nvResetId, 0, 1, &Ssf_resetReseason);
    }

    if((pNV != NULL) && (pNV->deleteItem != NULL))
    {
        /* Only use this reason once */
        (void)pNV->deleteItem(nvResetId);
    }

    if((pNV != NULL) && (pNV->readItem != NULL))
    {
        NVINTF_itemID_t id;
        uint16_t resetCount = 0;

        /* Setup NV ID */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_RESET_COUNT_ID;
        id.subID = 0;

        /* Read the reset count */
        pNV->readItem(id, 0, sizeof(resetCount), &resetCount);

        Ssf_resetCount = resetCount;
        if(pNV->writeItem)
        {
          /* Update the reset count for the next reset */
          resetCount++;
          pNV->writeItem(id, sizeof(resetCount), &resetCount);
        }
    }

#ifdef FEATURE_NATIVE_OAD
    OADClientParams.pEvent = &Sensor_events;
    OADClientParams.eventSem = sensorSem;

    OADClient_open(&OADClientParams);
#endif //FEATURE_NATIVE_OAD

#if CONFIG_BLE_SUPPORT
    uBLEInit();
#endif /* BLE_SUPPORT */
    LCD_WRITE_STRING("TI Sensor", 1);
#if !defined(AUTO_START)
    LCD_WRITE_STRING("Waiting...", 2);
#endif /* AUTO_START */
//#endif /* !POWER_MEAS */
}

/*!
 The application must call this function periodically to
 process any events that this module needs to process.

 Public function defined in ssf.h
 */
void Ssf_processEvents(void)
{
    /* Did a key press occur? */
    if(events & KEY_EVENT)
    {

        if((keys & KEY_LEFT) && (keys & KEY_RIGHT))
        {
#if defined(FEATURE_OAD)
            if(hUart != NULL)
            {
                UART_close(hUart);
            }
            Oad_markSwitch();
#endif /* FEATURE_OAD */
        }

        /* Left key press is a PAN disassociation request, if the device has started. */
        else if((keys & KEY_LEFT) && (started == true))
        {
            /* Send disassociation request */
            Jdllc_sendDisassociationRequest();
        }

        /* Right key press is BLE URL Toggle. */
        else if(keys & KEY_RIGHT)
        {
#if CONFIG_URL_TOGGLE
            MicroEddystoneBeacon_initURL();
#endif

            if(started == false)
            {
#ifndef POWER_MEAS
                LCD_WRITE_STRING("Starting...", 2);
#endif

                /* Tell the sensor to start */
                Util_setEvent(&Sensor_events, SENSOR_START_EVT);
            }
        }

        /* Clear the key press indication */
        keys = 0;

        /* Clear the event */
        Util_clearEvent(&events, KEY_EVENT);
    }

#ifdef FEATURE_NATIVE_OAD
    /* Did a OAD event occur? */
    if(Sensor_events & SENSOR_OAD_TIMEOUT_EVT)
    {
        OADClient_processEvent(&Sensor_events);
    }
#endif //FEATURE_NATIVE_OAD

#if CONFIG_BLE_SUPPORT
    if (events & UEB_EVT_MICROBLESTACK){
    	ub_processMsg();

    	/* Clear the event */
    	Util_clearEvent(&events, UEB_EVT_MICROBLESTACK);
    }
#endif /* BLE_SUPPORT */
}

/*!
 The application calls this function to indicate that the
 Sensor's state has changed.

 Public function defined in ssf.h
 */
void Ssf_stateChangeUpdate(Jdllc_states_t state)
{
#ifndef POWER_MEAS
   LCD_WRITE_STRING_VALUE("State Changed: ", state, 10, 3);
#endif
}

/*!
 The application calls this function to indicate that it has
 started or restored the device in a network.

 Public function defined in ssf.h
 */
void Ssf_networkUpdate(bool rejoined,
                       ApiMac_deviceDescriptor_t *pDevInfo,
                       Llc_netInfo_t  *pParentInfo)
{
    /* check for valid structure ponters, ignore if not */
    if((pDevInfo != NULL) && (pParentInfo != NULL))
    {
        if((pNV != NULL) && (pNV->writeItem != NULL))
        {
            NVINTF_itemID_t id;
            nvDeviceInfo_t nvItem;

            /* Setup NV ID */
            id.systemID = NVINTF_SYSID_APP;
            id.itemID = SSF_NV_NETWORK_INFO_ID;
            id.subID = 0;

            memcpy(&nvItem.device, pDevInfo, sizeof(ApiMac_deviceDescriptor_t));
            memcpy(&nvItem.parent, pParentInfo, sizeof(Llc_netInfo_t));

            /* Write the NV item */
            pNV->writeItem(id, sizeof(nvDeviceInfo_t), &nvItem);
        }


        if(pParentInfo->fh == false)
        {
#ifndef POWER_MEAS
            if(rejoined == false)
            {
                LCD_WRITE_STRING_VALUE("Started: 0x",
                                       pDevInfo->shortAddress, 16, 4);
            }
            else
            {
                LCD_WRITE_STRING_VALUE("Restarted: 0x",
                                       pDevInfo->shortAddress, 16, 4);
            }

            LCD_WRITE_STRING_VALUE("Channel: ", pParentInfo->channel, 10, 5);
#endif /* !POWER_MEAS */
            started = true;
        }
        else
        {
#ifndef POWER_MEAS
            if(rejoined == false)
            {
                LCD_WRITE_STRING("Device Started", 4);
            }
            else
            {
                LCD_WRITE_STRING("Device Restarted", 4);
            }

            LCD_WRITE_STRING("Frequency Hopping", 5);
#endif /* !POWER_MEAS */

            started = true;
        }

#ifndef POWER_MEAS
        Board_Led_control(board_led_type_LED1, board_led_state_ON);
#endif
        led1State = true;
    }
}

/*!
 The application calls this function to get the device
 *              information in a network.

 Public function defined in ssf.h
 */
bool Ssf_getNetworkInfo(ApiMac_deviceDescriptor_t *pDevInfo,
                        Llc_netInfo_t  *pParentInfo)
{
    if((pNV != NULL) && (pNV->readItem != NULL)
                    && (pDevInfo != NULL) && (pParentInfo != NULL))
    {
        NVINTF_itemID_t id;
        nvDeviceInfo_t nvItem;

        /* Setup NV ID */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_NETWORK_INFO_ID;
        id.subID = 0;

        /* Read Network Information from NV */
        if(pNV->readItem(id, 0, sizeof(nvDeviceInfo_t),
                         &nvItem) == NVINTF_SUCCESS)
        {
            memcpy(pDevInfo, &nvItem.device,
                   sizeof(ApiMac_deviceDescriptor_t));
            memcpy(pParentInfo, &nvItem.parent, sizeof(Llc_netInfo_t));

            return (true);
        }
    }
    return (false);
}

/*!
 The application calls this function to update the OAD info in NV.

 Public function defined in ssf.h
 */
void Ssf_oadInfoUpdate(uint16_t *pOadBlock, uint8_t *pOadImgHdr, uint8_t *pOadImgId, ApiMac_sAddr_t *pOadServerAddr)
{
    NVINTF_itemID_t id;

    /* Setup NV ID */
    id.systemID = NVINTF_SYSID_APP;
    id.itemID = SSF_NV_OAD_ID;
    id.subID = 0;
    /* Write the NV item */
    pNV->writeItem(id, sizeof(uint16_t), pOadBlock);

    if(pOadImgHdr != NULL)
    {
        id.subID = 1;
        /* Write the NV item */
        pNV->writeItem(id, sizeof(uint8_t) * 16, pOadImgHdr);
    }

    if(pOadImgId != NULL)
    {
        id.subID = 2;
        /* Write the NV item */
        pNV->writeItem(id, sizeof(uint8_t), pOadImgId);
    }

    if(pOadServerAddr != NULL)
    {
        id.subID = 3;
        /* Write the NV item */
        pNV->writeItem(id, sizeof(ApiMac_sAddr_t), pOadServerAddr);
    }
}

/*!
 The application calls this function to get the device
 *              information in a network.

 Public function defined in ssf.h
 */
bool Ssf_getOadInfo(uint16_t *pOadBlock, uint8_t *pOadImgHdr, uint8_t *pOadImgId, ApiMac_sAddr_t *pOadServerAddr)
{
    NVINTF_itemID_t id;
    bool status = false;

    if((pNV != NULL) && (pNV->readItem != NULL)
                    && (pOadBlock != NULL) && (pOadImgHdr != NULL))
    {
        /* Setup NV ID */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_OAD_ID;
        id.subID = 0;

        /* Read OAD Block from NV */
        if(pNV->readItem(id, 0, sizeof(uint16_t),
                         pOadBlock) == NVINTF_SUCCESS)
        {
            status = true;
        }
    }

    if(status == true)
    {
        id.subID = 1;
        /* Read OAD image hdr from NV */
        if(pNV->readItem(id, 0, sizeof(uint8_t) * 16,
                         pOadImgHdr) != NVINTF_SUCCESS)
        {
            status = false;
        }
    }

    if(status == true)
    {
        id.subID = 2;
        /* Read OAD image ID from NV */
        if(pNV->readItem(id, 0, sizeof(uint8_t),
                         pOadImgId) != NVINTF_SUCCESS)
        {
            status = false;
        }
    }

    if(status == true)
    {
        id.subID = 3;
        /* Read OAD image ID from NV */
        if(pNV->readItem(id, 0, sizeof(ApiMac_sAddr_t),
                         pOadServerAddr) != NVINTF_SUCCESS)
        {
            status = false;
        }
    }

    return status;
}

/*!
 The application calls this function to indicate a Configuration
 Request message.

 Public function defined in ssf.h
 */
void Ssf_configurationUpdate(Smsgs_configRspMsg_t *pRsp)
{
    if((pNV != NULL) && (pNV->writeItem != NULL) && (pRsp != NULL))
    {
        NVINTF_itemID_t id;
        Ssf_configSettings_t configInfo;

        /* Setup NV ID */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_CONFIG_INFO_ID;
        id.subID = 0;

        configInfo.frameControl = pRsp->frameControl;
        configInfo.reportingInterval = pRsp->reportingInterval;
        configInfo.pollingInterval = pRsp->pollingInterval;

        /* Write the NV item */
        pNV->writeItem(id, sizeof(Ssf_configSettings_t), &configInfo);
    }

#if !defined(CC13XX_LAUNCHXL) && !defined(CC26XX_LAUNCHXL) && !defined(CC26X2R1_LAUNCHXL) && !defined(CC1352R1_LAUNCHXL)
#ifndef POWER_MEAS
    Board_Led_toggle(board_led_type_LED4);
#endif
#endif
}

/*!
 The application calls this function to get the saved device configuration.

 Public function defined in ssf.h
 */
bool Ssf_getConfigInfo(Ssf_configSettings_t *pInfo)
{
    if((pNV != NULL) && (pNV->readItem != NULL) && (pInfo != NULL))
    {
        NVINTF_itemID_t id;

        /* Setup NV ID */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_CONFIG_INFO_ID;
        id.subID = 0;

        /* Read Network Information from NV */
        if(pNV->readItem(id, 0, sizeof(Ssf_configSettings_t),
                         pInfo) == NVINTF_SUCCESS)
        {
            return (true);
        }
    }
    return (false);
}

/*!
 The application calls this function to indicate that a tracking message
 was recieved.

 Public function defined in ssf.h
 */
void Ssf_trackingUpdate(ApiMac_sAddr_t *pSrcAddr)
{
#if !defined(CC13XX_LAUNCHXL) && !defined(CC26XX_LAUNCHXL) && !defined(CC26X2R1_LAUNCHXL) && !defined(CC1352R1_LAUNCHXL)
#ifndef POWER_MEAS
    Board_Led_toggle(board_led_type_LED3);
#endif
#endif
}

/*!
 The application calls this function to indicate sensor data.

 Public function defined in ssf.h
 */
void Ssf_sensorReadingUpdate(Smsgs_sensorMsg_t *pMsg)
{
#ifndef POWER_MEAS
    Board_Led_toggle(board_led_type_LED2);
#endif
}

/*!
 Initialize the reading clock.

 Public function defined in ssf.h
 */
void Ssf_initializeReadingClock(void)
{
    /* Initialize the timers needed for this application */
    readingClkHandle = Timer_construct(&readingClkStruct,
                                        processReadingTimeoutCallback,
                                        READING_INIT_TIMEOUT_VALUE,
                                        0,
                                        false,
                                        0);
}

/*!
 Set the reading clock.

 Public function defined in ssf.h
 */
void Ssf_setReadingClock(uint32_t readingTime)
{
    /* Stop the Reading timer */
    if(Timer_isActive(&readingClkStruct) == true)
    {
        Timer_stop(&readingClkStruct);
    }
#ifdef POWER_MEAS
    if(POWER_TEST_PROFILE != DATA_ACK)
    {
        /* Do not sent data in other power test profiles */
        return;
    }
#endif
    /* Setup timer */
    if ( readingTime )
    {
        Timer_setTimeout(readingClkHandle, readingTime);
        Timer_start(&readingClkStruct);
    }
}

/*!
 Ssf implementation for memory allocation

 Public function defined in ssf.h
 */
void *Ssf_malloc(uint16_t size)
{
    return ICall_malloc(size);
}

/*!
 Ssf implementation for memory de-allocation

 Public function defined in ssf.h
 */
void Ssf_free(void *ptr)
{
    if(ptr != NULL)
    {
        ICall_free(ptr);
    }
}

/*!
 Initialize the trickle clock.

 Public function defined in ssf.h
 */
void Ssf_initializeTrickleClock(void)
{
    /* Initialize trickle timer */
    tricklePASClkHandle = Timer_construct(&tricklePASClkStruct,
                                         processPASTrickleTimeoutCallback,
                                         TRICKLE_TIMEOUT_VALUE,
                                         0,
                                         false,
                                         0);

    tricklePCSClkHandle = Timer_construct(&tricklePCSClkStruct,
                                         processPCSTrickleTimeoutCallback,
                                         TRICKLE_TIMEOUT_VALUE,
                                         0,
                                         false,
                                         0);
}

/*!
 Set the trickle clock.

 Public function defined in ssf.h
 */
void Ssf_setTrickleClock(uint16_t trickleTime, uint8_t frameType)
{
    uint16_t randomNum = 0;
    if(frameType == ApiMac_wisunAsyncFrame_advertisementSolicit)
    {
        /* Stop the PA trickle timer */
        if(Timer_isActive(&tricklePASClkStruct) == true)
        {
            Timer_stop(&tricklePASClkStruct);
        }

        if(trickleTime > 0)
        {
            /* Trickle Time has to be a value chosen random between [t/2, t] */
            randomNum = ((ApiMac_randomByte() << 8) + ApiMac_randomByte());
            trickleTime = (trickleTime >> 1) +
                          (randomNum % (trickleTime >> 1));
            /* Setup timer */
            Timer_setTimeout(tricklePASClkHandle, trickleTime);
            Timer_start(&tricklePASClkStruct);
        }
    }
    else if(frameType == ApiMac_wisunAsyncFrame_configSolicit)
    {
        /* Stop the PC trickle timer */
        if(Timer_isActive(&tricklePCSClkStruct) == true)
        {
            Timer_stop(&tricklePCSClkStruct);
        }

        if(trickleTime > 0)
        {
            /* Setup timer */
            /* Trickle Time has to be a value chosen random between [t/2, t] */
            /* Generate a 16 bit random number */
            randomNum = ((ApiMac_randomByte() << 8) + ApiMac_randomByte());
            trickleTime = (trickleTime >> 1) +
                          (randomNum % (trickleTime >> 1));
            Timer_setTimeout(tricklePCSClkHandle, trickleTime);
            Timer_start(&tricklePCSClkStruct);
        }
    }
}

/*!
 Initialize the poll clock.

 Public function defined in ssf.h
 */
void Ssf_initializePollClock(void)
{
    /* Initialize the timers needed for this application */
    pollClkHandle = Timer_construct(&pollClkStruct,
                                     processPollTimeoutCallback,
                                     POLL_TIMEOUT_VALUE,
                                     0,
                                     false,
                                     0);
}

/*!
 Set the poll clock.

 Public function defined in ssf.h
 */
void Ssf_setPollClock(uint32_t pollTime)
{
    /* Stop the Reading timer */
    if(Timer_isActive(&pollClkStruct) == true)
    {
        Timer_stop(&pollClkStruct);
    }
#ifdef POWER_MEAS
    if ((POWER_TEST_PROFILE == DATA_ACK) || (POWER_TEST_PROFILE == SLEEP))
    {
        return;
    }
#endif
    /* Setup timer */
    if(pollTime > 0)
    {
        Timer_setTimeout(pollClkHandle, pollTime);
        Timer_start(&pollClkStruct);
    }
}

/*!
 Get the poll clock.

 Public function defined in ssf.h
 */
uint32_t Ssf_getPollClock(void)
{
    return Timer_getTimeout(pollClkHandle);
}

/*!
 Initialize the scan backoff clock.

 Public function defined in ssf.h
 */
void Ssf_initializeScanBackoffClock(void)
{
    /* Initialize the timers needed for this application */
    scanBackoffClkHandle = Timer_construct(&scanBackoffClkStruct,
                                           processScanBackoffTimeoutCallback,
                                           SCAN_BACKOFF_TIMEOUT_VALUE,
                                           0,
                                           false,
                                           0);
}

/*!
 Set the scan backoff clock.

 Public function defined in ssf.h
 */
void Ssf_setScanBackoffClock(uint32_t scanBackoffTime)
{
    /* Stop the Reading timer */
    if(Timer_isActive(&scanBackoffClkStruct) == true)
    {
        Timer_stop(&scanBackoffClkStruct);
    }

    /* Setup timer */
    if(scanBackoffTime > 0)
    {
        Timer_setTimeout(scanBackoffClkHandle, scanBackoffTime);
        Timer_start(&scanBackoffClkStruct);
    }
}

/*!
 Stop the scan backoff clock.

 Public function defined in ssf.h
 */
void Ssf_stopScanBackoffClock(void)
{
    /* Stop the Reading timer */
    if(Timer_isActive(&scanBackoffClkStruct) == true)
    {
        Timer_stop(&scanBackoffClkStruct);
    }
}

/*!
 Initialize the FH Association delay clock.

 Public function defined in ssf.h
 */
void Ssf_initializeFHAssocClock(void)
{
    /* Initialize the timers needed for this application */
    fhAssocClkHandle = Timer_construct(&fhAssocClkStruct,
                                       processFHAssocTimeoutCallback,
                                       FH_ASSOC_TIMER,
                                        0,
                                        false,
                                        0);
}

/*!
 Set the FH Association delay clock.

 Public function defined in ssf.h
 */
void Ssf_setFHAssocClock(uint32_t fhAssocTime)
{
    /* Stop the Reading timer */
    if(Timer_isActive(&fhAssocClkStruct) == true)
    {
        Timer_stop(&fhAssocClkStruct);
    }

    /* Setup timer */
    if ( fhAssocTime )
    {
        if(!CERTIFICATION_TEST_MODE)
        {
            /* Adding an additional random delay */
            fhAssocTime = fhAssocTime + (((ApiMac_randomByte() << 8) +
                          ApiMac_randomByte()) % ADD_ASSOCIATION_RANDOM_WINDOW);
        }
        Timer_setTimeout(fhAssocClkHandle, fhAssocTime);
        Timer_start(&fhAssocClkStruct);
    }
}

/*!
 The application calls this function to check if a device exists in blacklist,
 returns its index if it does.

 Public function defined in ssf.h
 */
int Ssf_findBlackListIndex(ApiMac_sAddr_t *pAddr)
{
    if((pNV != NULL) && (pAddr != NULL)
       && (pAddr->addrMode != ApiMac_addrType_none))
    {
        uint16_t numEntries;

        numEntries = getNumBlackListEntries();

        if(numEntries > 0)
        {
            NVINTF_itemID_t id;
            uint8_t stat;
            int subId = 0;
            int readItems = 0;

            /* Setup NV ID for the black list records */
            id.systemID = NVINTF_SYSID_APP;
            id.itemID = SSF_NV_BLACKLIST_ID;

            while((readItems < numEntries) && (subId < SSF_MAX_BLACKLIST_IDS))
            {
                ApiMac_sAddr_t item;

                id.subID = subId;

                /* Read Network Information from NV */
                stat = pNV->readItem(id, 0, sizeof(ApiMac_sAddr_t), &item);
                if(stat == NVINTF_SUCCESS)
                {
                    if(pAddr->addrMode == item.addrMode)
                    {
                        /* Is the address the same */
                        if(((pAddr->addrMode == ApiMac_addrType_short)
                            && (pAddr->addr.shortAddr == item.addr.shortAddr))
                           || ((pAddr->addrMode == ApiMac_addrType_extended)
                               && (memcmp(&pAddr->addr.extAddr,
                                          &item.addr.extAddr,
                                          APIMAC_SADDR_EXT_LEN)
                                   == 0)))
                        {
                            return (subId);
                        }
                    }
                    readItems++;
                }
                subId++;
            }
        }
    }

    return (-1);
}

/*!
 Update the Frame Counter

 Public function defined in ssf.h
 */
void Ssf_updateFrameCounter(ApiMac_sAddr_t *pDevAddr, uint32_t frameCntr)
{
    if(pDevAddr == NULL)
    {
        if((pNV != NULL) && (pNV->writeItem != NULL) && (frameCntr >=
              (lastSavedFrameCounter + FRAME_COUNTER_SAVE_WINDOW)))
        {
            NVINTF_itemID_t id;

            /* Setup NV ID */
            id.systemID = NVINTF_SYSID_APP;
            id.itemID = SSF_NV_FRAMECOUNTER_ID;
            id.subID = 0;

            /* Write the NV item */
            if(pNV->writeItem(id, sizeof(uint32_t), &frameCntr)
                            == NVINTF_SUCCESS)
            {
                lastSavedFrameCounter = frameCntr;
            }
        }
    }
}

/*!
 Get the Frame Counter

 Public function defined in ssf.h
 */
bool Ssf_getFrameCounter(ApiMac_sAddr_t *pDevAddr, uint32_t *pFrameCntr)
{
    /* Check for valid pointer */
    if(pFrameCntr != NULL)
    {
        /*
         A pDevAddr that is null means to get the frame counter for this device
         */
        if(pDevAddr == NULL)
        {
            if((pNV != NULL) && (pNV->readItem != NULL))
            {
                NVINTF_itemID_t id;

                /* Setup NV ID */
                id.systemID = NVINTF_SYSID_APP;
                id.itemID = SSF_NV_FRAMECOUNTER_ID;
                id.subID = 0;

                /* Read Network Information from NV */
                if(pNV->readItem(id, 0, sizeof(uint32_t), pFrameCntr)
                                == NVINTF_SUCCESS)
                {
                    /* Set to the next window */
                    *pFrameCntr += FRAME_COUNTER_SAVE_WINDOW;
                    return(true);
                }
                else
                {
                    /*
                     Wasn't found, so write 0, so the next time it will be
                     greater than 0
                     */
                    uint32_t fc = 0;

                    /* Setup NV ID */
                    id.systemID = NVINTF_SYSID_APP;
                    id.itemID = SSF_NV_FRAMECOUNTER_ID;
                    id.subID = 0;

                    /* Write the NV item */
                    pNV->writeItem(id, sizeof(uint32_t), &fc);
                }
            }
        }

        *pFrameCntr = 0;
    }
    return (false);
}

/*!
 Display Error

 Public function defined in ssf.h
 */
void Ssf_displayError(uint8_t *pTxt, uint8_t code)
{
#ifndef POWER_MEAS
    LCD_WRITE_STRING_VALUE((char*)pTxt, code, 16, 5);
#endif
}

/*!
 Assert Indication

 Public function defined in ssf.h
 */
void Ssf_assertInd(uint8_t reason)
{
    if((pNV != NULL) && (pNV->writeItem != NULL))
    {
        /* Attempt to save reason to read after reset */
        (void)pNV->writeItem(nvResetId, 1, &reason);
    }
}

/*!

 Clear network information in NV

 Public function defined in ssf.h
 */
void Ssf_clearNetworkInfo()
{
    if((pNV != NULL) && (pNV->deleteItem != NULL))
    {
        NVINTF_itemID_t id;

        /* Setup NV ID */
#ifdef ONE_PAGE_NV
        id.systemID = 0xFF;
        id.itemID = 0xFFFF;
        id.subID = 0xFFFF;
#else
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_NETWORK_INFO_ID;
        id.subID = 0;
#endif
        pNV->deleteItem(id);

        /* sensor ready to associate again */
        started = false;
    }
}

/*!
 Clear all the NV Items

 Public function defined in ssf.h
 */
void Ssf_clearAllNVItems(void)

{
#ifdef ONE_PAGE_NV
    if((pNV != NULL) && (pNV->deleteItem != NULL))
    {
        NVINTF_itemID_t id;

        /* Clear all information */
        id.systemID = 0xFF;
        id.itemID = 0xFFFF;
        id.subID = 0xFFFF;
        pNV->deleteItem(id);
    }
#else
    /* Clear Network Information */
    Ssf_clearNetworkInfo();

    if((pNV != NULL) && (pNV->deleteItem != NULL))
    {
        NVINTF_itemID_t id;
        uint16_t entries;

        /* Clear the black list entries number */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_BLACKLIST_ENTRIES_ID;
        id.subID = 0;
        pNV->deleteItem(id);

        /*
         Clear the black list entries.  Brute force through
         every possible subID, if it doesn't exist that's fine,
         it will fail in deleteItem.
         */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_BLACKLIST_ID;
        for(entries = 0; entries < SSF_MAX_BLACKLIST_IDS; entries++)
        {
            id.subID = entries;
            pNV->deleteItem(id);
        }

        /* Clear the device tx frame counter */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_FRAMECOUNTER_ID;
        id.subID = 0;
        pNV->deleteItem(id);

        /* Clear the reset reason */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_RESET_REASON_ID;
        id.subID = 0;
        pNV->deleteItem(id);

        /* Clear the reset count */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_RESET_COUNT_ID;
        id.subID = 0;
        pNV->deleteItem(id);
    }
#endif
}

/*!
 Add an entry into the black list

 Public function defined in ssf.h
 */
bool Ssf_addBlackListItem(ApiMac_sAddr_t *pAddr)
{
    bool retVal = false;

    if((pNV != NULL) && (pNV->writeItem != NULL)
    		&& (pAddr != NULL) && (pAddr->addrMode != ApiMac_addrType_none))
    {
        if(Ssf_findBlackListIndex(pAddr))
        {
            retVal = true;
        }
        else
        {
            uint8_t stat;
            NVINTF_itemID_t id;
            uint16_t numEntries = getNumBlackListEntries();

            /* Check the maximum size */
            if(numEntries < SSF_MAX_BLACKLIST_ENTRIES)
            {
                /* Setup NV ID for the black list record */
                id.systemID = NVINTF_SYSID_APP;
                id.itemID = SSF_NV_BLACKLIST_ID;
                id.subID = findUnuedBlackListIndex();

                /* write the black list record */
                stat = pNV->writeItem(id, sizeof(ApiMac_sAddr_t), pAddr);
                if(stat == NVINTF_SUCCESS)
                {
                    /* Update the number of entries */
                    numEntries++;
                    saveNumBlackListEntries(numEntries);
                    retVal = true;
                }
            }
        }
    }

    return(retVal);
}

/*!
 Delete an address from the black list

 Public function defined in ssf.h
 */
void Ssf_removeBlackListItem(ApiMac_sAddr_t *pAddr)
{
    if((pNV != NULL) && (pNV->deleteItem != NULL))
    {
        int index;

        /* Does the item exist? */
        index = Ssf_findBlackListIndex(pAddr);
        if(index > 0)
        {
            uint8_t stat;
            NVINTF_itemID_t id;

            /* Setup NV ID for the black list record */
            id.systemID = NVINTF_SYSID_APP;
            id.itemID = SSF_NV_BLACKLIST_ID;
            id.subID = (uint16_t)index;

            stat = pNV->deleteItem(id);
            if(stat == NVINTF_SUCCESS)
            {
                /* Update the number of entries */
                uint16_t numEntries = getNumBlackListEntries();

                if(numEntries > 0)
                {
                    numEntries--;
                    saveNumBlackListEntries(numEntries);
                }
            }
        }
    }
}

/*!
 Read the on-board temperature sensors

 Public function defined in ssf.h
 */
int16_t Ssf_readTempSensor(void)
{
#ifdef POWER_MEAS
    return (0);
#else
    return ((int16_t)AONBatMonTemperatureGetDegC());
#endif /* POWER_MEAS */
}

/*!
 The application calls this function to toggle an LED.

 Public function defined in ssf.h
 */
bool Ssf_toggleLED(void)
{
    if(led1State == true)
    {
        led1State = false;
#ifndef POWER_MEAS
        Board_Led_control(board_led_type_LED1, board_led_state_OFF);
#endif
    }
    else
    {
        led1State = true;
#ifndef POWER_MEAS
        Board_Led_control(board_led_type_LED1, board_led_state_ON);
#endif
    }

    return(led1State);
}

/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief   Reading timeout handler function.
 *
 * @param   a0 - ignored
 */
static void processReadingTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Sensor_events, SENSOR_READING_TIMEOUT_EVT);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Key event handler function
 *
 * @param       keysPressed - keys that are pressed
 */
static void processKeyChangeCallback(uint8_t keysPressed)
{
    keys = keysPressed;

    events |= KEY_EVENT;

    /* Wake up the application thread when it waits for keys event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Trickle timeout handler function for PA .
 *
 * @param       a0 - ignored
 */
static void processPASTrickleTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Jdllc_events, JDLLC_PAS_EVT);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Trickle timeout handler function for PC.
 *
 * @param       a0 - ignored
 */
static void processPCSTrickleTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Jdllc_events, JDLLC_PCS_EVT);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Poll timeout handler function  .
 *
 * @param       a0 - ignored
 */
static void processPollTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Jdllc_events, JDLLC_POLL_EVT);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Scan backoff timeout handler function  .
 *
 * @param       a0 - ignored
 */
static void processScanBackoffTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Jdllc_events, JDLLC_SCAN_BACKOFF);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       FH Assoc Delay timeout handler function  .
 *
 * @param       a0 - ignored
 */
static void processFHAssocTimeoutCallback(UArg a0)
{
    (void)a0; /* Parameter is not used */

    Util_setEvent(&Jdllc_events, JDLLC_ASSOCIATE_REQ_EVT);

    /* Wake up the application thread when it waits for clock event */
    Semaphore_post(sensorSem);
}

/*!
 * @brief       Find an unused blacklist index
 *
 * @return      sub index not used
 */
static int findUnuedBlackListIndex(void)
{
    int subId = 0;

    if(pNV != NULL)
    {
        uint16_t numEntries;

        numEntries = getNumBlackListEntries();

        if(numEntries > 0)
        {
            NVINTF_itemID_t id;

            int readItems = 0;
            /* Setup NV ID for the black list records */
            id.systemID = NVINTF_SYSID_APP;
            id.itemID = SSF_NV_BLACKLIST_ID;

            while((readItems < numEntries) && (subId < SSF_MAX_BLACKLIST_IDS))
            {
                ApiMac_sAddr_t item;
                uint8_t stat;

                id.subID = subId;

                /* Read Network Information from NV */
                stat = pNV->readItem(id, 0, sizeof(ApiMac_sAddr_t), &item);
                if(stat == NVINTF_NOTFOUND)
                {
                    /* Use this sub id */
                    break;
                }
                else if(stat == NVINTF_SUCCESS)
                {
                    readItems++;
                }
                subId++;
            }
        }
    }

    return (subId);
}

/*!
 * @brief       Read the number of black list items stored
 *
 * @return      number of entries in the black list
 */
static uint16_t getNumBlackListEntries(void)
{
    uint16_t numEntries = 0;

    if(pNV != NULL)
    {
        NVINTF_itemID_t id;
        uint8_t stat;

        /* Setup NV ID for the number of entries in the black list */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_BLACKLIST_ENTRIES_ID;
        id.subID = 0;

        /* Read the number of black list items from NV */
        stat = pNV->readItem(id, 0, sizeof(uint16_t), &numEntries);
        if(stat != NVINTF_SUCCESS)
        {
            numEntries = 0;
        }
    }
    return (numEntries);
}

/*!
 * @brief       Read the number of black list items stored
 *
 * @return      number of entries in the black list
 */
static void saveNumBlackListEntries(uint16_t numEntries)
{
    if(pNV != NULL)
    {
        NVINTF_itemID_t id;

        /* Setup NV ID for the number of entries in the black list */
        id.systemID = NVINTF_SYSID_APP;
        id.itemID = SSF_NV_BLACKLIST_ENTRIES_ID;
        id.subID = 0;

        /* Read the number of black list items from NV */
        pNV->writeItem(id, sizeof(uint16_t), &numEntries);
    }
}
#if CONFIG_BLE_SUPPORT
/*********************************************************************
* @fn      uBLEStack_eventProxy
*
* @brief   Post an event to the application so that a Micro BLE Stack internal
*          event is processed by Micro BLE Stack later in the application
*          task's context.
*
* @param   None
*
* @return  None
*/
void uBLEStack_eventProxy()
{
 // MicroEddystoneBeacon_enqueueMsg(UEB_EVT_MICROBLESTACK, 0);
	Util_setEvent(&events, UEB_EVT_MICROBLESTACK);
    Semaphore_post(sensorSem);
}
#endif
