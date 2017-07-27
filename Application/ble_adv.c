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
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/PIN.h>
#include <string.h>
#include <driverlib/aon_batmon.h>
#include "board.h"
#include "timer.h"
#include "board_lcd.h"
#include "config.h"

#if CONFIG_BLE_SUPPORT
#include "bcomdef.h"
#include "uble.h"
#include "ugap.h"
#include "gap.h"

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

// Eddystone Base 128-bit UUID: EE0CXXXX-8786-40BA-AB96-99B91AC981D8
#define EDDYSTONE_BASE_UUID_128( uuid )  0xD8, 0x81, 0xC9, 0x1A, 0xB9, 0x99, \
                                         0x96, 0xAB, 0xBA, 0x40, 0x86, 0x87, \
                           LO_UINT16( uuid ), HI_UINT16( uuid ), 0x0C, 0xEE

// Pre-generated Random Static Address
#define UEB_PREGEN_RAND_STATIC_ADDR    {0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc}

// Eddystone definitions
#define EDDYSTONE_SERVICE_UUID                  0xFEAA

#define EDDYSTONE_FRAME_TYPE_UID                0x00
#define EDDYSTONE_FRAME_TYPE_URL                0x10
#define EDDYSTONE_FRAME_TYPE_TLM                0x20

#define EDDYSTONE_FRAME_OVERHEAD_LEN            8
#define EDDYSTONE_SVC_DATA_OVERHEAD_LEN         3
#define EDDYSTONE_MAX_URL_LEN                   18

// # of URL Scheme Prefix types
#define EDDYSTONE_URL_PREFIX_MAX        4
// # of encodable URL words
#define EDDYSTONE_URL_ENCODING_MAX      14

#define EDDYSTONE_URI_DATA_DEFAULT      "http://www.ti.com/tool/TIDEP0084"//"http://www.ti.com/ble"

// App to App event
typedef struct {
  uint16 event;
  uint8 data;
} uebEvt_t;

// Eddystone UID frame
typedef struct {
  uint8 frameType;      // UID
  int8 rangingData;
  uint8 namespaceID[10];
  uint8 instanceID[6];
  uint8 reserved[2];
} eddystoneUID_t;

// Eddystone URL frame
typedef struct {
  uint8 frameType;      // URL | Flags
  int8 txPower;
  uint8 encodedURL[EDDYSTONE_MAX_URL_LEN];  // the 1st byte is prefix
} eddystoneURL_t;

// Eddystone TLM frame
typedef struct {
  uint8 frameType;      // TLM
  uint8 version;        // 0x00 for now
  uint8 vBatt[2];       // Battery Voltage, 1mV/bit, Big Endian
  uint8 temp[2];        // Temperature. Signed 8.8 fixed point
  uint8 advCnt[4];      // Adv count since power-up/reboot
  uint8 secCnt[4];      // Time since power-up/reboot
                          // in 0.1 second resolution
} eddystoneTLM_t;

typedef union {
  eddystoneUID_t uid;
  eddystoneURL_t url;
  eddystoneTLM_t tlm;
} eddystoneFrame_t;

typedef struct {
  uint8 length1;        // 2
  uint8 dataType1;      // for Flags data type (0x01)
  uint8 data1;          // for Flags data (0x04)
  uint8 length2;        // 3
  uint8 dataType2;      // for 16-bit Svc UUID list data type (0x03)
  uint8 data2;          // for Eddystone UUID LSB (0xAA)
  uint8 data3;          // for Eddystone UUID MSB (0xFE)
  uint8 length;         // Eddystone service data length
  uint8 dataType3;      // for Svc Data data type (0x16)
  uint8 data4;          // for Eddystone UUID LSB (0xAA)
  uint8 data5;          // for Eddystone UUID MSB (0xFE)
  eddystoneFrame_t frame;
} eddystoneAdvData_t;

// Broadcaster state
bool uebBcastActive;

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static eddystoneAdvData_t eddystoneAdv = {
// Flags; this sets the device to use general discoverable mode
    0x02,// length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED | GAP_ADTYPE_FLAGS_GENERAL,

    // Complete list of 16-bit Service UUIDs
    0x03,// length of this data including the data type byte
    GAP_ADTYPE_16BIT_COMPLETE, LO_UINT16(EDDYSTONE_SERVICE_UUID), HI_UINT16(
        EDDYSTONE_SERVICE_UUID),

    // Service Data
    0x03,// to be set properly later
    GAP_ADTYPE_SERVICE_DATA, LO_UINT16(EDDYSTONE_SERVICE_UUID), HI_UINT16(
        EDDYSTONE_SERVICE_UUID) };

eddystoneUID_t eddystoneUID;
eddystoneURL_t eddystoneURL;
eddystoneTLM_t eddystoneTLM;

uint8 eddystoneURLDataLen;

// Array of URL Scheme Prefices
static char* eddystoneURLPrefix[EDDYSTONE_URL_PREFIX_MAX] = { "http://www.",
    "https://www.", "http://", "https://" };

// Array of URLs to be encoded
static char* eddystoneURLEncoding[EDDYSTONE_URL_ENCODING_MAX] = { ".com/",
    ".org/", ".edu/", ".net/", ".info/", ".biz/", ".gov/", ".com/", ".org/",
    ".edu/", ".net/", ".info/", ".biz/", ".gov/" };

static uint32 advCount = 0;

// Eddystone frame type currently used
static uint8 currentFrameType = EDDYSTONE_FRAME_TYPE_URL;

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static void MicroEddystoneBeacon_updateTLM(void);
static void MicroEddystoneBeacon_initUID(void);
static void MicroEddystoneBeacon_updateAdvDataWithFrame(uint8 frameType);

static void MicroEddystoneBeacon_bcast_stateChangeCB(ugBcastState_t newState);
static void MicroEddystoneBeacon_bcast_advPrepareCB(void);
static void MicroEddystoneBeacon_bcast_advDoneCB(bStatus_t status);

extern void uBLEStack_eventProxy(void);

/******************************************************************************
 Global Variables
 *****************************************************************************/
extern PIN_Handle ledPinHandle;

/*********************************************************************
* @fn      MicroEddystoneBeacon_initUID
*
* @brief   initialize UID frame
*
* @param   none
*
* @return  none
*/
static void MicroEddystoneBeacon_initUID(void)
{
 // Set Eddystone UID frame with meaningless numbers for example.
 // This need to be replaced with some algorithm-based formula
 // for production.
 eddystoneUID.namespaceID[0] = 0x00;
 eddystoneUID.namespaceID[1] = 0x01;
 eddystoneUID.namespaceID[2] = 0x02;
 eddystoneUID.namespaceID[3] = 0x03;
 eddystoneUID.namespaceID[4] = 0x04;
 eddystoneUID.namespaceID[5] = 0x05;
 eddystoneUID.namespaceID[6] = 0x06;
 eddystoneUID.namespaceID[7] = 0x07;
 eddystoneUID.namespaceID[8] = 0x08;
 eddystoneUID.namespaceID[9] = 0x09;

 eddystoneUID.instanceID[0] = 0x04;
 eddystoneUID.instanceID[1] = 0x51;
 eddystoneUID.instanceID[2] = 0x40;
 eddystoneUID.instanceID[3] = 0x00;
 eddystoneUID.instanceID[4] = 0xB0;
 eddystoneUID.instanceID[5] = 0x00;
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_encodeURL
*
* @brief   Encodes URL in accordance with Eddystone URL frame spec
*
* @param   urlOrg - Plain-string URL to be encoded
*          urlEnc - Encoded URL. Should be URLCFGSVC_CHAR_URI_DATA_LEN-long.
*
* @return  0 if the prefix is invalid
*          The length of the encoded URL including prefix otherwise
*/
uint8 MicroEddystoneBeacon_encodeURL(char* urlOrg, uint8* urlEnc)
{
 uint8 i, j;
 uint8 urlLen;
 uint8 tokenLen;

 urlLen = (uint8) strlen(urlOrg);

 // search for a matching prefix
 for (i = 0; i < EDDYSTONE_URL_PREFIX_MAX; i++)
 {
   tokenLen = strlen(eddystoneURLPrefix[i]);
   if (strncmp(eddystoneURLPrefix[i], urlOrg, tokenLen) == 0)
   {
     break;
   }
 }

 if (i == EDDYSTONE_URL_PREFIX_MAX)
 {
   return 0;       // wrong prefix
 }

 // use the matching prefix number
 urlEnc[0] = i;
 urlOrg += tokenLen;
 urlLen -= tokenLen;

 // search for a token to be encoded
 for (i = 0; i < urlLen; i++)
 {
   for (j = 0; j < EDDYSTONE_URL_ENCODING_MAX; j++)
   {
     tokenLen = strlen(eddystoneURLEncoding[j]);
     if (strncmp(eddystoneURLEncoding[j], urlOrg + i, tokenLen) == 0)
     {
       // matching part found
       break;
     }
   }

   if (j < EDDYSTONE_URL_ENCODING_MAX)
   {
     memcpy(&urlEnc[1], urlOrg, i);
     // use the encoded byte
     urlEnc[i + 1] = j;
     break;
   }
 }

 if (i < urlLen)
 {
   memcpy(&urlEnc[i + 2], urlOrg + i + tokenLen, urlLen - i - tokenLen);
   return urlLen - tokenLen + 2;
 }

 memcpy(&urlEnc[1], urlOrg, urlLen);
 return urlLen + 1;
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_initUID
*
* @brief   initialize URL frame
*
* @param   none
*
* @return  none
*/
void MicroEddystoneBeacon_initURL(void)
{
    static uint32 urlCount = 0;

    const char* urls[5] = {"http://www.ti.com","http://tinyurl.com/z7ofjy7","http://tinyurl.com/jt6j7ya","http://tinyurl.com/h53v6fe","http://www.ti.com/TI154Stack"};

    // Set Eddystone URL frame with the URL of TI BLE site.
    eddystoneURLDataLen = MicroEddystoneBeacon_encodeURL(
    (char *)urls[urlCount++], eddystoneURL.encodedURL);
    if (urlCount>4){
       urlCount=0;
 }
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_updateTLM
*
* @brief   Update TLM elements
*
* @param   none
*
* @return  none
*/
static void MicroEddystoneBeacon_updateTLM(void)
{
 uint32 time100MiliSec;
 uint32 batt;

 // Battery voltage (bit 10:8 - integer, but 7:0 fraction)
 batt = AONBatMonBatteryVoltageGet();
 batt = (batt * 125) >> 5; // convert V to mV
 eddystoneTLM.vBatt[0] = HI_UINT16(batt);
 eddystoneTLM.vBatt[1] = LO_UINT16(batt);

 // Temperature - 19.5 (Celcius) for example
 eddystoneTLM.temp[0] = 19;
 eddystoneTLM.temp[1] = 256 / 2;

 // advertise packet cnt;
 eddystoneTLM.advCnt[0] = BREAK_UINT32(advCount, 3);
 eddystoneTLM.advCnt[1] = BREAK_UINT32(advCount, 2);
 eddystoneTLM.advCnt[2] = BREAK_UINT32(advCount, 1);
 eddystoneTLM.advCnt[3] = BREAK_UINT32(advCount, 0);

 // running time
 // the number of 100-ms periods that have passed since the beginning.
 // no consideration of roll over for now.
 time100MiliSec = Clock_getTicks() / (100000 / Clock_tickPeriod);
 eddystoneTLM.secCnt[0] = BREAK_UINT32(time100MiliSec, 3);
 eddystoneTLM.secCnt[1] = BREAK_UINT32(time100MiliSec, 2);
 eddystoneTLM.secCnt[2] = BREAK_UINT32(time100MiliSec, 1);
 eddystoneTLM.secCnt[3] = BREAK_UINT32(time100MiliSec, 0);

 LCD_WRITE_STRING("TLM data updates:",3);
 LCD_WRITE_STRING_VALUE("Cnt=",advCount,10,4);
 LCD_WRITE_STRING_VALUE("Time=",time100MiliSec,10,5);
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_selectFrame
*
* @brief   Selecting the type of frame to be put in the service data
*
* @param   frameType - Eddystone frame type
*
* @return  none
*/
static void MicroEddystoneBeacon_updateAdvDataWithFrame(uint8 frameType)
{
 if (frameType == EDDYSTONE_FRAME_TYPE_UID
     || frameType == EDDYSTONE_FRAME_TYPE_URL
     || frameType == EDDYSTONE_FRAME_TYPE_TLM)
 {
   eddystoneFrame_t* pFrame;
   uint8 frameSize;

   eddystoneAdv.length = EDDYSTONE_SVC_DATA_OVERHEAD_LEN;
   // Fill with 0s first
   memset((uint8*) &eddystoneAdv.frame, 0x00, sizeof(eddystoneFrame_t));

   switch (frameType) {
   case EDDYSTONE_FRAME_TYPE_UID:
     eddystoneUID.frameType = EDDYSTONE_FRAME_TYPE_UID;
     frameSize = sizeof(eddystoneUID_t);
     pFrame = (eddystoneFrame_t *) &eddystoneUID;

     LCD_WRITE_STRING("Eddystone UID Mode",6);
     break;

   case EDDYSTONE_FRAME_TYPE_URL:
     eddystoneURL.frameType = EDDYSTONE_FRAME_TYPE_URL;
     frameSize = sizeof(eddystoneURL_t) - EDDYSTONE_MAX_URL_LEN
         + eddystoneURLDataLen;
     pFrame = (eddystoneFrame_t *) &eddystoneURL;

     LCD_WRITE_STRING("Eddystone URL Mode",6);
     break;

   case EDDYSTONE_FRAME_TYPE_TLM:
     eddystoneTLM.frameType = EDDYSTONE_FRAME_TYPE_TLM;
     frameSize = sizeof(eddystoneTLM_t);
     pFrame = (eddystoneFrame_t *) &eddystoneTLM;
     break;
   }

   memcpy((uint8 *) &eddystoneAdv.frame, (uint8 *) pFrame, frameSize);
   eddystoneAdv.length += frameSize;

   ub_setParameter(UB_PARAM_ADVDATA,
                   EDDYSTONE_FRAME_OVERHEAD_LEN + eddystoneAdv.length,
                   &eddystoneAdv);
 }
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_startBroadcast
*
* @brief   Start broadcasting.
*          If configuration mode was on going, stop it.
*
* @param   none
*
* @return  none
*/
void MicroEddystoneBeacon_startBroadcast(void)
{
 uint8 tempPower;

 ub_getParameter(UB_PARAM_TXPOWER, &tempPower);
 eddystoneUID.rangingData = tempPower;
 eddystoneURL.txPower = tempPower;

 // Select UID or URL frame as adv data initially
 MicroEddystoneBeacon_updateAdvDataWithFrame(currentFrameType);

  //ug_bcastStart(10); /* Broadcaster will stop after 100th Adv event */
 ug_bcastStart(0); /* Broadcaster runs indefinitely */
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_bcast_stateChange_CB
*
* @brief   Callback from Micro Broadcaster indicating a state change.
*
* @param   newState - new state
*
* @return  None.
*/
static void MicroEddystoneBeacon_bcast_stateChangeCB(ugBcastState_t newState)
{
 switch (newState)
 {
 case UG_BCAST_STATE_INITIALIZED:
   uebBcastActive = FALSE;
   LCD_WRITE_STRING("BC_State:Initialized",7);
   {
     // Parameter containers
     uint16 param16; /* 16-bit parameter */

     // Setup broadcaster duty cycle
    ug_bcastSetDuty(100, 50); /* OnTime: 10 sec, OffTime: 5 sec */

     // Setup adv interval
     param16 = DEFAULT_ADVERTISING_INTERVAL;
     ub_setParameter(UB_PARAM_ADVINTERVAL, sizeof(uint16), &param16);

     uint8  param8;  /* 8-bit parameter */
     // Setup adv channel map
    param8 = UB_ADV_CHAN_37 | UB_ADV_CHAN_38 | UB_ADV_CHAN_39; /* Use only channels 38 & 39 */
    ub_setParameter(UB_PARAM_ADVCHANMAP, sizeof(uint8), &param8);
   }
   // Start advertising
   MicroEddystoneBeacon_startBroadcast();
   break;

 case UG_BCAST_STATE_IDLE:
   uebBcastActive = FALSE;
   LCD_WRITE_STRING("BC_State:Idle",7);
   break;

 case UG_BCAST_STATE_ADVERTISING:
   uebBcastActive = TRUE;
   LCD_WRITE_STRING("BC_State:Advertising",7);
   break;

 case UG_BCAST_STATE_WAITING:
     LCD_WRITE_STRING("BC_State:Waiting",7);
   break;

 default:
   break;
 }
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_bcast_advPrepareCB
*
* @brief   Callback from Micro Broadcaster notifying that the next
*          advertising event is about to start so it's time to update
*          the adv payload.
*
* @param   None.
*
* @return  None.
*/
static void MicroEddystoneBeacon_bcast_advPrepareCB(void)
{
 uint8 timeToAdv = 0;

 MicroEddystoneBeacon_updateTLM();
 MicroEddystoneBeacon_updateAdvDataWithFrame(EDDYSTONE_FRAME_TYPE_TLM);

 // Disable ADV_PREPARE notification from the next Adv
 ub_setParameter(UB_PARAM_TIMETOADV, sizeof(timeToAdv), &timeToAdv);
}

/*********************************************************************
* @fn      MicroEddystoneBeacon_bcast_advDoneCB
*
* @brief   Callback from Micro Broadcaster notifying that an
*          advertising event has been done.
*
* @param   status - How the last event was done. SUCCESS or FAILURE.
*
* @return  None.
*/
static void MicroEddystoneBeacon_bcast_advDoneCB(bStatus_t status)
{
 advCount++;

 // change URL:

 if ((advCount % 10) == 8)
 {
   uint8 timeToAdv = 5;

   // Set TimeToAdv parameter to get MicroEddystoneBeacon_bcast_advPrepareCB()
   // callback issued 5 ms before the second next advertising event.
   ub_setParameter(UB_PARAM_TIMETOADV, sizeof(timeToAdv), &timeToAdv);
 }
 else if ((advCount % 10) == 0)
 {
   // Send UID or URL
   MicroEddystoneBeacon_updateAdvDataWithFrame(currentFrameType);
 }

 LCD_WRITE_STRING_VALUE("Adv's done=", advCount,10,5);
}

/*!
 * @brief       2.4 GHz Antenna switch setting.
 */
void bleAntSettingHandler(void)
{
    //Swtich RF switch to 2.4G antenna
    PIN_setOutputValue(ledPinHandle, Board_DIO1_RFSW, 0);
}

void uBLEInit(void)
{
#if defined(FEATURE_STATIC_ADDR)
  uint8 staticAddr[] = UEB_PREGEN_RAND_STATIC_ADDR;
#endif /* FEATURE_STATIC_ADDR */
  ugBcastCBs_t bcastCBs = {
    MicroEddystoneBeacon_bcast_stateChangeCB,
    MicroEddystoneBeacon_bcast_advPrepareCB,
    MicroEddystoneBeacon_bcast_advDoneCB };

  // Initialize UID frame
  MicroEddystoneBeacon_initUID();

  // Initialize URL frame
  MicroEddystoneBeacon_initURL();

  /* Initialize Micro BLE Stack */

  /*set ant switch handler for 2.4Ghz before initializing BLE stack */
  ub_registerAntSwitchCb(bleAntSettingHandler);

  #if defined(FEATURE_STATIC_ADDR)
  ub_stackInit(UB_ADDRTYPE_STATIC, staticAddr, uBLEStack_eventProxy);
  #else  /* FEATURE_STATIC_ADDR */
  ub_stackInit(UB_ADDRTYPE_PUBLIC, NULL, uBLEStack_eventProxy, RF_TIME_CRITICAL);
  #endif /* FEATURE_STATIC_ADDR */

  /* Initilaize Micro GAP Broadcaster Role */
  ug_bcastInit(&bcastCBs);
}
#endif
