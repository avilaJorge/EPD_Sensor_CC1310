/******************************************************************************

 @file sensor.c

 @brief TIMAC 2.0 Sensor Example Application

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
#include <string.h>
#include <stdint.h>
#include "util.h"
#include "api_mac.h"
#include "jdllc.h"
#include "ssf.h"
#include "smsgs.h"
#include "sensor.h"
#include "config.h"
#include "board_led.h"
#include "icall.h"

/* Image Header files */
#include <ti/drivers/utils/RingBuf.h>
#include "Data_Module.h"
#include <ti/drivers/UART.h>

#ifdef FEATURE_NATIVE_OAD
#include <common/native_oad/oad_protocol.h>
#include <sensor_oad/oad_client.h>
#endif //FEATURE_NATIVE_OAD

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

#if !defined(CONFIG_AUTO_START)
#if defined(AUTO_START)
#define CONFIG_AUTO_START 1
#else
#define CONFIG_AUTO_START 0
#endif
#endif

/* Size of image data buffer */
#define IMAGE_DATA_BUFFER_SIZE 4000

/* Image data RF packet size */
#define IMAGE_DATA_RF_PACKET_SIZE 101

/* Image data UART packet size */
#define IMAGE_DATA_UART_PACKET_SIZE 32

/* Number of image data packets to receive from RF*/
#define IMAGE_DATA_NUM_RX_RF_PACKETS 150

/* Number of image data packets to send via UART*/
#define IMAGE_DATA_NUM_TX_UART_PACKETS 469

/* default MSDU Handle rollover */
#define MSDU_HANDLE_MAX 0x1F

/* App marker in MSDU handle */
#define APP_MARKER_MSDU_HANDLE 0x80

/* App Message Tracking Mask */
#define APP_MASK_MSDU_HANDLE 0x60

/* App Sensor Data marker for the MSDU handle */
#define APP_SENSOR_MSDU_HANDLE 0x40

/* App tracking response marker for the MSDU handle */
#define APP_TRACKRSP_MSDU_HANDLE 0x20

/* App config response marker for the MSDU handle */
#define APP_CONFIGRSP_MSDU_HANDLE 0x60

/* Reporting Interval Min and Max (in milliseconds) */
#define MIN_REPORTING_INTERVAL 1000
#define MAX_REPORTING_INTERVAL 360000

/* Polling Interval Min and Max (in milliseconds) */
#define MIN_POLLING_INTERVAL 100//1000
#define MAX_POLLING_INTERVAL 10000

/* Inter packet interval in certification test mode */
#if CERTIFICATION_TEST_MODE
#if ((CONFIG_PHY_ID >= APIMAC_MRFSK_STD_PHY_ID_BEGIN) && (CONFIG_PHY_ID <= APIMAC_MRFSK_GENERIC_PHY_ID_BEGIN))
/*! Regular Mode */
#define SENSOR_TEST_RAMP_DATA_SIZE   75
#define CERT_MODE_INTER_PKT_INTERVAL 50
#elif ((CONFIG_PHY_ID >= APIMAC_MRFSK_GENERIC_PHY_ID_BEGIN + 1) && (CONFIG_PHY_ID <= APIMAC_MRFSK_GENERIC_PHY_ID_END))
/*! LRM Mode */
#define SENSOR_TEST_RAMP_DATA_SIZE   20
#define CERT_MODE_INTER_PKT_INTERVAL 300
#else
#error "PHY ID is wrong."
#endif
#endif
/******************************************************************************
 External Variables
 *****************************************************************************/

extern UART_Handle uartHandle;

/******************************************************************************
 Global variables
 *****************************************************************************/

/* Task pending events */
uint16_t Sensor_events = 0;

/*! Sensor statistics */
Smsgs_msgStatsField_t Sensor_msgStats =
    { 0 };

#ifdef POWER_MEAS
/*! Power Meas Stats fields */
Smsgs_powerMeastatsField_t Sensor_pwrMeasStats =
    { 0 };
#endif
/******************************************************************************
 Local variables
 *****************************************************************************/

static void *sem;

/*! Rejoined flag */
static bool rejoining = false;

/*! Collector's address */
static ApiMac_sAddr_t collectorAddr = {0};

/* Join Time Ticks (used for average join time calculations) */
static uint_fast32_t joinTimeTicks = 0;

/* Interim Delay Ticks (used for average delay calculations) */
static uint_fast32_t interimDelayTicks = 0;

/* RingBuf Object and handle for object */
static RingBuf_Object ringBufObj;
static RingBuf_Handle ringBufHandle = &ringBufObj;

/* Buffer for receiving image data */
static uint8_t dataBuf[IMAGE_DATA_BUFFER_SIZE];

/* Index of sent image data packet */
static uint16_t txPacketIndex = 0;

/* Index of received image data packet */
static uint16_t rxPacketIndex = 0;

/* Data array for sending UART messages */
static uint8_t uartImageDataPacket[IMAGE_DATA_UART_PACKET_SIZE];

/*! Device's Outgoing MSDU Handle values */
STATIC uint8_t deviceTxMsduHandle = 0;

STATIC Smsgs_configReqMsg_t configSettings;

/*!
 Temp Sensor field - valid only if Smsgs_dataFields_tempSensor
 is set in frameControl.
 */
STATIC Smsgs_tempSensorField_t tempSensor =
    { 0 };

/*!
 Light Sensor field - valid only if Smsgs_dataFields_lightSensor
 is set in frameControl.
 */
STATIC Smsgs_lightSensorField_t lightSensor =
    { 0 };

/*!
 Humidity Sensor field - valid only if Smsgs_dataFields_humiditySensor
 is set in frameControl.
 */
STATIC Smsgs_humiditySensorField_t humiditySensor =
    { 0 };

STATIC Llc_netInfo_t parentInfo = {0};

/* For CMD_EraseFlash */
Command_info_t eraseCmdInfo = {
        RECV_HEADER0,   // Header0
        RECV_HEADER1,   // Header1
        0,              // Packet_Index
        CMD_EraseFlash, // cmd
        6,              // Length
        Res_Result,     // Response
        Result_Fail       // Result
};

Flash_Parameter_Info_t eraseFlashInfo = {
        0x001A0000, // Address //TODO: Get this information from Flash libraries in PDI Apps
        0x0008      // Length
};

/* For CMD_WriteImageFileInfo */
Command_info_t imageFileCmdInfo = {
        RECV_HEADER0,           // Header0
        RECV_HEADER1,           // Header1
        0,                      // Packet_Index
        CMD_WriteImageFileInfo, // cmd
        0x0020,                 // Length
        Res_Result,             // Response
        Result_Fail             // Result
};

ImageFile_info_t imageFileInfo = {
        0x001A7FE0,             //Address; //TODO: Use Enums for these
        IMAGE_DATA_NUM_TX_UART_PACKETS, //ImagePacketLen; //TODO: Update using sent data
        M_IsExist,              //Mark; (0xFE = M_IsExist)
        11,                     //PanelSize;
        0x03,                   //ImageType;
        "Danger"                //Name[23];
};

/* For CMD_WriteImageData */
Command_info_t imageCmdInfo = {
        RECV_HEADER0,           // Header0
        RECV_HEADER1,           // Header1
        0,                      // Packet_Index
        CMD_WriteImageData,     // cmd
        0x0026,                 // Length
        Res_Not,                // Response
        Result_Fail             // Result
};

Flash_Info_t imageDataInfo = {
        0x001A0000,             // Address
        0x0020                  // Length
};

/* For CMD_EPDShow */
Command_info_t showCmdInfo = {
        RECV_HEADER0,           // Header0
        RECV_HEADER1,           // Header1
        0,                      // Packet_Index
        CMD_EPDShow,            // cmd
        0x00C0,                 // Length
        Res_Result,             // Response
        Result_Fail             // Result
};

ShowEpd_Info_t showInfo = {
        0x0A,                   // Driver_Type
        0x0B,                   // EPD_Size
        0x9C,                   // Temperature
        OTP_Mode,               // IsOTP (OperationMode_t)
        0x001A0000,             // PrevImageAddress
        0x001A0000              // NewImageAddress
};

/******************************************************************************
 Local function prototypes
 *****************************************************************************/

static void initializeClocks(void);
static void dataCnfCB(ApiMac_mcpsDataCnf_t *pDataCnf);
static void dataIndCB(ApiMac_mcpsDataInd_t *pDataInd);
static uint8_t getMsduHandle(Smsgs_cmdIds_t msgType);
static void processSensorMsgEvt(void);
#if SENSOR_TEST_RAMP_DATA_SIZE
static void processSensorRampMsgEvt(void);
#endif
static bool sendSensorMessage(ApiMac_sAddr_t *pDstAddr,
                              Smsgs_sensorMsg_t *pMsg);
static void processConfigRequest(ApiMac_mcpsDataInd_t *pDataInd);
static void processImageDataRequest(ApiMac_mcpsDataInd_t *pDataInd); //XXX:
static uint8_t receiveImageData(ApiMac_mcpsDataInd_t *pDataInd);
static bool sendConfigRsp(ApiMac_sAddr_t *pDstAddr, Smsgs_configRspMsg_t *pMsg);
static uint16_t startEPDUpdate(void);
static uint16_t sendImageDataPacket(void);
static uint16_t displayImage(void);
static uint16_t validateFrameControl(uint16_t frameControl);

static void jdllcJoinedCb(ApiMac_deviceDescriptor_t *pDevInfo,
                          Llc_netInfo_t  *pStartedInfo);
static void jdllcDisassocIndCb(ApiMac_sAddrExt_t *extAddress,
                               ApiMac_disassocateReason_t reason);
static void jdllcDisassocCnfCb(ApiMac_sAddrExt_t *extAddress,
                               ApiMac_status_t status);
static void jdllcStateChangeCb(Jdllc_states_t state);
static void readSensors(void);

/******************************************************************************
 Callback tables
 *****************************************************************************/

/*! API MAC Callback table */
STATIC ApiMac_callbacks_t Sensor_macCallbacks =
    {
      /*! Associate Indicated callback */
      NULL,
      /*! Associate Confirmation callback */
      NULL,
      /*! Disassociate Indication callback */
      NULL,
      /*! Disassociate Confirmation callback */
      NULL,
      /*! Beacon Notify Indication callback */
      NULL,
      /*! Orphan Indication callback */
      NULL,
      /*! Scan Confirmation callback */
      NULL,
      /*! Start Confirmation callback */
      NULL,
      /*! Sync Loss Indication callback */
      NULL,
      /*! Poll Confirm callback */
      NULL,
      /*! Comm Status Indication callback */
      NULL,
      /*! Poll Indication Callback */
      NULL,
      /*! Data Confirmation callback */
      dataCnfCB,
      /*! Data Indication callback */
      dataIndCB,
      /*! Purge Confirm callback */
      NULL,
      /*! WiSUN Async Indication callback */
      NULL,
      /*! WiSUN Async Confirmation callback */
      NULL,
      /*! Unprocessed message callback */
      NULL
    };

STATIC Jdllc_callbacks_t jdllcCallbacks =
    {
      /*! Network Joined Indication callback */
      jdllcJoinedCb,
      /* Disassociation Indication callback */
      jdllcDisassocIndCb,
      /* Disassociation Confirm callback */
      jdllcDisassocCnfCb,
      /*! State Changed indication callback */
      jdllcStateChangeCb
    };

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 Initialize this application.

 Public function defined in sensor.h
 */
void Sensor_init(void)
{
    uint32_t frameCounter = 0;

    /* Initialize the sensor's structures */
    memset(&configSettings, 0, sizeof(Smsgs_configReqMsg_t));
#if defined(TEMP_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_tempSensor;
#endif
#if defined(LIGHT_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_lightSensor;
#endif
#if defined(HUMIDITY_SENSOR)
    configSettings.frameControl |= Smsgs_dataFields_humiditySensor;
#endif
    configSettings.frameControl |= Smsgs_dataFields_msgStats;
    configSettings.frameControl |= Smsgs_dataFields_configSettings;
#ifndef POWER_MEAS
    if(!CERTIFICATION_TEST_MODE)
    {

        configSettings.reportingInterval = CONFIG_REPORTING_INTERVAL;
    }
    else
    {
        /* start back to back data transmission at the earliest */
        configSettings.reportingInterval = 100;
    }
    configSettings.pollingInterval = CONFIG_POLLING_INTERVAL;
#else
        configSettings.reportingInterval = POWER_TEST_EVT_INT;
        configSettings.pollingInterval = POWER_TEST_EVT_INT;
#endif

    /* Initialize the MAC */
    sem = ApiMac_init(CONFIG_FH_ENABLE);

    /* Initialize the Joining Device Logical Link Controller */
    Jdllc_init(&Sensor_macCallbacks, &jdllcCallbacks);

    /* Register the MAC Callbacks */
    ApiMac_registerCallbacks(&Sensor_macCallbacks);

    /* Initialize the platform specific functions */
    Ssf_init(sem);

    Ssf_getFrameCounter(NULL, &frameCounter);

#ifdef FEATURE_MAC_SECURITY
    /* Initialize the MAC Security */
    Jdllc_securityInit(frameCounter);
#endif /* FEATURE_MAC_SECURITY */

    /* Set the transmit power */
    ApiMac_mlmeSetReqUint8(ApiMac_attribute_phyTransmitPowerSigned,
                           (uint8_t)CONFIG_TRANSMIT_POWER);
#ifdef FCS_TYPE16
    /* Set the fcs type */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_fcsType,
                           (bool)1);
#endif
    /* Initialize the app clocks */
    initializeClocks();

    /* Initialize RingBuf */
    RingBuf_construct(ringBufHandle, dataBuf, IMAGE_DATA_BUFFER_SIZE);

    if(CONFIG_AUTO_START)
    {
        /* Start the device */
        Util_setEvent(&Sensor_events, SENSOR_START_EVT);
    }
}

/*!
 Application task processing.

 Public function defined in sensor.h
 */
void Sensor_process(void)
{
    /* Start the collector device in the network */
    if(Sensor_events & SENSOR_START_EVT)
    {
        ApiMac_deviceDescriptor_t devInfo;
        Llc_netInfo_t parentInfo;

        if(Ssf_getNetworkInfo(&devInfo, &parentInfo ) == true)
        {
            Ssf_configSettings_t configInfo;
#ifdef FEATURE_MAC_SECURITY
            ApiMac_status_t stat;
#endif /* FEATURE_MAC_SECURITY */

            /* Do we have config settings? */
            if(Ssf_getConfigInfo(&configInfo) == true)
            {
                /* Save the config information */
                configSettings.frameControl = configInfo.frameControl;
                configSettings.reportingInterval = configInfo.reportingInterval;
                configSettings.pollingInterval = configInfo.pollingInterval;

                /* Update the polling interval in the LLC */
                Jdllc_setPollRate(configSettings.pollingInterval);
            }

            /* Initially, setup the parent as the collector */
            if(parentInfo.fh == true && !LRM_MODE)
            {
                collectorAddr.addrMode = ApiMac_addrType_extended;
                memcpy(&collectorAddr.addr.extAddr,
                       parentInfo.devInfo.extAddress, APIMAC_SADDR_EXT_LEN);
            }
            else
            {
                collectorAddr.addrMode = ApiMac_addrType_short;
                collectorAddr.addr.shortAddr = parentInfo.devInfo.shortAddress;
            }

#ifdef FEATURE_MAC_SECURITY
            /* Put the parent in the security device list */
            stat = Jdllc_addSecDevice(parentInfo.devInfo.panID,
                                      parentInfo.devInfo.shortAddress,
                                      &parentInfo.devInfo.extAddress, 0);
            if(stat != ApiMac_status_success)
            {
                Ssf_displayError("Auth Error: 0x", (uint8_t)stat);
            }
#endif /* FEATURE_MAC_SECURITY */

            Jdllc_rejoin(&devInfo, &parentInfo);
            rejoining = true;
        }
        else
        {
            /* Get Start Timestamp */
            joinTimeTicks = ICall_getTicks();
            Jdllc_join();
        }

        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_START_EVT);
    }

    /* Is it time to send the next sensor data message? */
    if(Sensor_events & SENSOR_READING_TIMEOUT_EVT)
    {
        /* In certification test mode, back to back data shall be sent */
        if(!CERTIFICATION_TEST_MODE)
        {
            /* Setup for the next message */
            Ssf_setReadingClock(configSettings.reportingInterval);
        }


#if SENSOR_TEST_RAMP_DATA_SIZE
        processSensorRampMsgEvt();
#else
        /* Read sensors */
        readSensors();

        /* Process Sensor Reading Message Event */
        processSensorMsgEvt();
#endif

        /* Clear the event */
        Util_clearEvent(&Sensor_events, SENSOR_READING_TIMEOUT_EVT);
    }

    //TODO: Add event code here for START_EPD event and SEND event
    /* Are we starting an EPD update? */
    if(Sensor_events & SENSOR_START_EPD_UPDATE_EVT) {

        /* Start the EPD update process */
        startEPDUpdate();

        /* Clear the start EPD event and set the send EPD image data event */
        Util_clearEvent(&Sensor_events, SENSOR_START_EPD_UPDATE_EVT);
    }

    /* Are we ready to begin sending image data via UART? */
    if(Sensor_events & SENSOR_SEND_EPD_IMAGE_DATA_EVT) {

        /* Are we done sending packets? */
        if(txPacketIndex < IMAGE_DATA_NUM_TX_UART_PACKETS) {

            /* Is there anything left to send in the RingBuf? */
            if(RingBuf_getCount(ringBufHandle) > 0) {
                sendImageDataPacket();
                txPacketIndex++;
            } else {
                /* Clear the event that calls this function in order to continue RX */
                Util_clearEvent(&Sensor_events, SENSOR_SEND_EPD_IMAGE_DATA_EVT);
            }
        } else {
            /* If so clear send event, reset packetIndex and set display event */
            Util_clearEvent(&Sensor_events, SENSOR_SEND_EPD_IMAGE_DATA_EVT);
            Util_setEvent(&Sensor_events, SENSOR_DISPLAY_IMAGE_EVT);
        }
    }

    /* Are we ready to display the EPD image? */
    if(Sensor_events & SENSOR_DISPLAY_IMAGE_EVT) {

        /* Display the image and clear the event */
        displayImage();
        Util_clearEvent(&Sensor_events, SENSOR_DISPLAY_IMAGE_EVT);
    }

    /* Process LLC Events */
    Jdllc_process();

    /* Allow the Specific functions to process */
    Ssf_processEvents();

    /*
     Don't process ApiMac messages until all of the sensor events
     are processed.
     */
    if(Sensor_events == 0)
    {
        /* Wait for response message or events */
        ApiMac_processIncoming();
    }
}

/*!
 * @brief   Send MAC data request
 *
 * @param   type - message type
 * @param   pDstAddr - destination address
 * @param   rxOnIdle - true if not a sleepy device
 * @param   len - length of payload
 * @param   pData - pointer to the buffer
 *
 * @return  true if sent, false if not
 */
bool Sensor_sendMsg(Smsgs_cmdIds_t type, ApiMac_sAddr_t *pDstAddr,
                    bool rxOnIdle, uint16_t len, uint8_t *pData)
{
    bool ret = false;
    ApiMac_mcpsDataReq_t dataReq;

    /* Fill the data request field */
    memset(&dataReq, 0, sizeof(ApiMac_mcpsDataReq_t));

    memcpy(&dataReq.dstAddr, pDstAddr, sizeof(ApiMac_sAddr_t));

    if(pDstAddr->addrMode == ApiMac_addrType_extended)
    {
        dataReq.srcAddrMode = ApiMac_addrType_extended;
    }
    else
    {
        dataReq.srcAddrMode = ApiMac_addrType_short;
    }

    if(rejoining == true)
    {
        ApiMac_mlmeGetReqUint16(ApiMac_attribute_panId,
                                &(parentInfo.devInfo.panID));
    }

    dataReq.dstPanId = parentInfo.devInfo.panID;

    dataReq.msduHandle = getMsduHandle(type);

    dataReq.txOptions.ack = true;

    if(CERTIFICATION_TEST_MODE)
    {
        dataReq.txOptions.ack = false;
    }

    if(rxOnIdle == false)
    {
        dataReq.txOptions.indirect = true;
    }

    dataReq.msdu.len = len;
    dataReq.msdu.p = pData;

#ifdef FEATURE_MAC_SECURITY
    Jdllc_securityFill(&dataReq.sec);
#endif /* FEATURE_MAC_SECURITY */

    if(type == Smsgs_cmdIds_sensorData || type == Smsgs_cmdIds_rampdata)
    {
        interimDelayTicks = ICall_getTicks();
        Sensor_msgStats.msgsAttempted++;
    }
    else if(type == Smsgs_cmdIds_trackingRsp)
    {
        Sensor_msgStats.trackingResponseAttempts++;
    }
    else if(type == Smsgs_cmdIds_configRsp)
    {
        Sensor_msgStats.configResponseAttempts++;
    }

    /* Send the message */
    if(ApiMac_mcpsDataReq(&dataReq) == ApiMac_status_success)
    {
        ret = true;
    }
    else
    {
        /* handle transaction overflow by retrying */
        if(type == Smsgs_cmdIds_sensorData || type == Smsgs_cmdIds_rampdata)
        {
            Ssf_setReadingClock(configSettings.reportingInterval);
            Sensor_msgStats.msgsAttempted++;
        }
    }

    return (ret);
}


/******************************************************************************
 Local Functions
 *****************************************************************************/

/*!
 * @brief       Initialize the clocks.
 */
static void initializeClocks(void)
{
    /* Initialize the reading clock */
    Ssf_initializeReadingClock();
}

/*!
 * @brief      MAC Data Confirm callback.
 *
 * @param      pDataCnf - pointer to the data confirm information
 */
static void dataCnfCB(ApiMac_mcpsDataCnf_t *pDataCnf)
{
    /* Record statistics */
    if(pDataCnf->status == ApiMac_status_channelAccessFailure)
    {
        Sensor_msgStats.channelAccessFailures++;
    }
    else if(pDataCnf->status == ApiMac_status_noAck)
    {
        Sensor_msgStats.macAckFailures++;
    }
    else if(pDataCnf->status != ApiMac_status_success)
    {
        Sensor_msgStats.otherDataRequestFailures++;
        Ssf_displayError("dataCnf: ", pDataCnf->status);
    }
    else if(pDataCnf->status == ApiMac_status_success)
    {
        Ssf_updateFrameCounter(NULL, pDataCnf->frameCntr);
    }

    /* Make sure the message came from the app */
    if(pDataCnf->msduHandle & APP_MARKER_MSDU_HANDLE)
    {
        /* What message type was the original request? */
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_SENSOR_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                /* Calculate Average Delay */
                interimDelayTicks = ICall_getTicks() - interimDelayTicks;
                uint_fast32_t avgDelay =
                        Sensor_msgStats.msgsSent * Sensor_msgStats.interimDelay;
                avgDelay += (interimDelayTicks / TICKPERIOD_MS_US);
                avgDelay /= (Sensor_msgStats.msgsSent + 1);
                Sensor_msgStats.interimDelay = avgDelay;

                Sensor_msgStats.msgsSent++;
            }

#if CERTIFICATION_TEST_MODE
            {
                /* Setup for the next message */
                Ssf_setReadingClock(CERT_MODE_INTER_PKT_INTERVAL);
            }
#endif
        }
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_TRACKRSP_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                Sensor_msgStats.trackingResponseSent++;
            }
        }
        if((pDataCnf->msduHandle & APP_MASK_MSDU_HANDLE)
           == APP_CONFIGRSP_MSDU_HANDLE)
        {
            if(pDataCnf->status == ApiMac_status_success)
            {
                Sensor_msgStats.configResponseSent++;
            }
        }
    }
}

/*!
 * @brief      MAC Data Indication callback.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void dataIndCB(ApiMac_mcpsDataInd_t *pDataInd)
{
    uint8_t cmdBytes[SMSGS_TOGGLE_LED_RESPONSE_MSG_LEN];

    if((pDataInd != NULL) && (pDataInd->msdu.p != NULL)
       && (pDataInd->msdu.len > 0))
    {
        Smsgs_cmdIds_t cmdId = (Smsgs_cmdIds_t)*(pDataInd->msdu.p);

#ifdef FEATURE_MAC_SECURITY
        if(Jdllc_securityCheck(&(pDataInd->sec)) == false)
        {
            /* reject the message */
            return;
        }
#endif /* FEATURE_MAC_SECURITY */

        switch(cmdId)
        {
            case Smsgs_cmdIds_configReq:
                processConfigRequest(pDataInd);
                Sensor_msgStats.configRequests++;
                break;

            case Smsgs_cmdIds_trackingReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_TRACKING_REQUEST_MSG_LENGTH)
                {
                    /* Update stats */
                    Sensor_msgStats.trackingRequests++;

                    /* Indicate tracking message received */
                    Ssf_trackingUpdate(&pDataInd->srcAddr);

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_trackingRsp;
                    Sensor_sendMsg(Smsgs_cmdIds_trackingRsp,
                            &pDataInd->srcAddr, true,
                            1, cmdBytes);
                }
                break;

            case Smsgs_cmdIds_toggleLedReq:
                /* Make sure the message is the correct size */
                if(pDataInd->msdu.len == SMSGS_TOGGLE_LED_REQUEST_MSG_LEN)
                {

                    /* send the response message directly */
                    cmdBytes[0] = (uint8_t) Smsgs_cmdIds_toggleLedRsp;
                    cmdBytes[1] = Ssf_toggleLED();
                    Sensor_sendMsg(Smsgs_cmdIds_toggleLedRsp,
                            &pDataInd->srcAddr, true,
                            SMSGS_TOGGLE_LED_RESPONSE_MSG_LEN,
                            cmdBytes);
                }
                break;
#ifdef POWER_MEAS
            case Smsgs_cmdIds_rampdata:
                Sensor_pwrMeasStats.rampDataRcvd++;
                break;
#endif

#ifdef FEATURE_NATIVE_OAD
            case Smsgs_cmdIds_oad:
                //Index past the Smsgs_cmdId
                OADProtocol_ParseIncoming((void*) &(pDataInd->srcAddr), pDataInd->msdu.p + 1);
                break;
#endif //FEATURE_NATIVE_OAD

            case Smsgs_cmdIds_imageDataReq:
                processImageDataRequest(pDataInd);
                break; //XXX:

            case Smsgs_cmdIds_imageData:
                receiveImageData(pDataInd);
                break;

            default:
                /* Should not receive other messages */
                break;
        }
    }
}

/*!
 * @brief      Get the next MSDU Handle
 *             <BR>
 *             The MSDU handle has 3 parts:<BR>
 *             - The MSBit(7), when set means the the application sent the
 *               message
 *             - Bit 6, when set means that the app message is a config request
 *             - Bits 0-5, used as a message counter that rolls over.
 *
 * @param      msgType - message command id needed
 *
 * @return     msdu Handle
 */
static uint8_t getMsduHandle(Smsgs_cmdIds_t msgType)
{
    uint8_t msduHandle = deviceTxMsduHandle;

    /* Increment for the next msdu handle, or roll over */
    if(deviceTxMsduHandle >= MSDU_HANDLE_MAX)
    {
        deviceTxMsduHandle = 0;
    }
    else
    {
        deviceTxMsduHandle++;
    }

    /* Add the App specific bit */
    msduHandle |= APP_MARKER_MSDU_HANDLE;

    /* Add the message type bit */
    if(msgType == Smsgs_cmdIds_sensorData || msgType == Smsgs_cmdIds_rampdata)
    {
        msduHandle |= APP_SENSOR_MSDU_HANDLE;
    }
    else if(msgType == Smsgs_cmdIds_trackingRsp)
    {
        msduHandle |= APP_TRACKRSP_MSDU_HANDLE;
    }
    else if(msgType == Smsgs_cmdIds_configRsp)
    {
        msduHandle |= APP_CONFIGRSP_MSDU_HANDLE;
    }

    return (msduHandle);
}

/*!
 @brief  Build and send fixed size ramp data
 */
#if SENSOR_TEST_RAMP_DATA_SIZE
static void processSensorRampMsgEvt(void)
{
    uint8_t *pMsgBuf;
    uint16_t index;

    pMsgBuf = (uint8_t *)Ssf_malloc(SENSOR_TEST_RAMP_DATA_SIZE);
    if(pMsgBuf)
    {
        uint8_t *pBuf = pMsgBuf;
        *pBuf++ = (uint8_t)Smsgs_cmdIds_rampdata;
        for(index = 1; index < SENSOR_TEST_RAMP_DATA_SIZE; index++)
        {
            *pBuf++ = (uint8_t) (index & 0xFF);
        }

#ifndef POWER_MEAS
        Board_Led_toggle(board_led_type_LED2);
#endif

        Sensor_sendMsg(Smsgs_cmdIds_rampdata, &collectorAddr, true,
                SENSOR_TEST_RAMP_DATA_SIZE, pMsgBuf);

        Ssf_free(pMsgBuf);
    }

}
#endif

/*!
 @brief   Build and send sensor data message
 */
static void processSensorMsgEvt(void)
{
    Smsgs_sensorMsg_t sensor;
    uint32_t stat;

    memset(&sensor, 0, sizeof(Smsgs_sensorMsg_t));

    ApiMac_mlmeGetReqUint32(ApiMac_attribute_diagRxSecureFail, &stat);
    Sensor_msgStats.rxDecryptFailures = (uint16_t)stat;

    ApiMac_mlmeGetReqUint32(ApiMac_attribute_diagTxSecureFail, &stat);
    Sensor_msgStats.txEncryptFailures = (uint16_t)stat;

    ApiMac_mlmeGetReqArray(ApiMac_attribute_extendedAddress,
    		               sensor.extAddress);

    /* fill in the message */
    sensor.frameControl = configSettings.frameControl;
    if(sensor.frameControl & Smsgs_dataFields_tempSensor)
    {
        memcpy(&sensor.tempSensor, &tempSensor,
               sizeof(Smsgs_tempSensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_lightSensor)
    {
        memcpy(&sensor.lightSensor, &lightSensor,
               sizeof(Smsgs_lightSensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_humiditySensor)
    {
        memcpy(&sensor.humiditySensor, &humiditySensor,
               sizeof(Smsgs_humiditySensorField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_msgStats)
    {
        memcpy(&sensor.msgStats, &Sensor_msgStats,
               sizeof(Smsgs_msgStatsField_t));
    }
    if(sensor.frameControl & Smsgs_dataFields_configSettings)
    {
        sensor.configSettings.pollingInterval = configSettings.pollingInterval;
        sensor.configSettings.reportingInterval = configSettings
                        .reportingInterval;
    }

    /* inform the user interface */
    Ssf_sensorReadingUpdate(&sensor);

    /* send the data to the collector */
    sendSensorMessage(&collectorAddr, &sensor);
}

/*!
 * @brief   Build and send sensor data message
 *
 * @param   pDstAddr - Where to send the message
 * @param   pMsg - pointer to the sensor data
 *
 * @return  true if message was sent, false if not
 */
static bool sendSensorMessage(ApiMac_sAddr_t *pDstAddr, Smsgs_sensorMsg_t *pMsg)
{
    bool ret = false;
    uint8_t *pMsgBuf;
    uint16_t len = SMSGS_BASIC_SENSOR_LEN;

    /* Figure out the length */
    if(pMsg->frameControl & Smsgs_dataFields_tempSensor)
    {
        len += SMSGS_SENSOR_TEMP_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_lightSensor)
    {
        len += SMSGS_SENSOR_LIGHT_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_humiditySensor)
    {
        len += SMSGS_SENSOR_HUMIDITY_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_msgStats)
    {
        len += SMSGS_SENSOR_MSG_STATS_LEN;
    }
    if(pMsg->frameControl & Smsgs_dataFields_configSettings)
    {
        len += SMSGS_SENSOR_CONFIG_SETTINGS_LEN;
    }

    pMsgBuf = (uint8_t *)Ssf_malloc(len);
    if(pMsgBuf)
    {
        uint8_t *pBuf = pMsgBuf;

        *pBuf++ = (uint8_t)Smsgs_cmdIds_sensorData;

        memcpy(pBuf, pMsg->extAddress, SMGS_SENSOR_EXTADDR_LEN);
        pBuf += SMGS_SENSOR_EXTADDR_LEN;

        pBuf  = Util_bufferUint16(pBuf,pMsg->frameControl);

        if(pMsg->frameControl & Smsgs_dataFields_tempSensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->tempSensor.ambienceTemp);
            pBuf = Util_bufferUint16(pBuf, pMsg->tempSensor.objectTemp);
        }
        if(pMsg->frameControl & Smsgs_dataFields_lightSensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->lightSensor.rawData);
        }
        if(pMsg->frameControl & Smsgs_dataFields_humiditySensor)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->humiditySensor.temp);
            pBuf = Util_bufferUint16(pBuf, pMsg->humiditySensor.humidity);
        }
        if(pMsg->frameControl & Smsgs_dataFields_msgStats)
        {
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinAttempts);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinFails);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.msgsAttempted);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.msgsSent);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.trackingRequests);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.trackingResponseAttempts);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.trackingResponseSent);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.configRequests);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.configResponseAttempts);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.configResponseSent);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.channelAccessFailures);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.macAckFailures);
            pBuf = Util_bufferUint16(pBuf,
                                     pMsg->msgStats.otherDataRequestFailures);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.syncLossIndications);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.rxDecryptFailures);
            pBuf = Util_bufferUint16(pBuf,  pMsg->msgStats.txEncryptFailures);
            pBuf = Util_bufferUint16(pBuf, Ssf_resetCount);
            pBuf = Util_bufferUint16(pBuf,  Ssf_resetReseason);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.joinTime);
            pBuf = Util_bufferUint16(pBuf, pMsg->msgStats.interimDelay);
        }
        if(pMsg->frameControl & Smsgs_dataFields_configSettings)
        {
            pBuf = Util_bufferUint32(pBuf,
                                     pMsg->configSettings.reportingInterval);
            pBuf = Util_bufferUint32(pBuf,
                                     pMsg->configSettings.pollingInterval);

        }

        ret = Sensor_sendMsg(Smsgs_cmdIds_sensorData, pDstAddr, true, len, pMsgBuf);

        Ssf_free(pMsgBuf);
    }

    return (ret);
}

/*!
 * @brief      Process the Config Request message.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void processConfigRequest(ApiMac_mcpsDataInd_t *pDataInd)
{
    Smsgs_statusValues_t stat = Smsgs_statusValues_invalid;
    Smsgs_configRspMsg_t configRsp;

    memset(&configRsp, 0, sizeof(Smsgs_configRspMsg_t));

    /* Make sure the message is the correct size */
    if(pDataInd->msdu.len == SMSGS_CONFIG_REQUEST_MSG_LENGTH)
    {
        uint8_t *pBuf = pDataInd->msdu.p;
        uint16_t frameControl;
        uint32_t reportingInterval;
        uint32_t pollingInterval;

        /* Parse the message */
        configSettings.cmdId = (Smsgs_cmdIds_t)*pBuf++;
        frameControl = Util_parseUint16(pBuf);
        pBuf += 2;
        reportingInterval = Util_parseUint32(pBuf);
        pBuf += 4;
        pollingInterval = Util_parseUint32(pBuf);

        stat = Smsgs_statusValues_success;
        collectorAddr.addrMode = pDataInd->srcAddr.addrMode;
        if(collectorAddr.addrMode == ApiMac_addrType_short)
        {
            collectorAddr.addr.shortAddr = pDataInd->srcAddr.addr.shortAddr;
        }
        else
        {
            memcpy(collectorAddr.addr.extAddr, pDataInd->srcAddr.addr.extAddr,
                   (APIMAC_SADDR_EXT_LEN));
        }

        configSettings.frameControl = validateFrameControl(frameControl);
        if(configSettings.frameControl != frameControl)
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        configRsp.frameControl = configSettings.frameControl;

        if((reportingInterval < MIN_REPORTING_INTERVAL)
           || (reportingInterval > MAX_REPORTING_INTERVAL))
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        else
        {
#ifndef POWER_MEAS
            configSettings.reportingInterval = reportingInterval;
#endif
            {
                uint32_t randomNum;
                randomNum = ((ApiMac_randomByte() << 16) +
                             (ApiMac_randomByte() << 8) + ApiMac_randomByte());
                randomNum = (randomNum % reportingInterval) +
                        SENSOR_MIN_POLL_TIME;
                Ssf_setReadingClock(randomNum);
            }

        }
        configRsp.reportingInterval = configSettings.reportingInterval;

        if((pollingInterval < MIN_POLLING_INTERVAL)
           || (pollingInterval > MAX_POLLING_INTERVAL))
        {
            stat = Smsgs_statusValues_partialSuccess;
        }
        else
        {
            configSettings.pollingInterval = pollingInterval;
            Jdllc_setPollRate(configSettings.pollingInterval);
        }
        configRsp.pollingInterval = configSettings.pollingInterval;
    }

    /* Send the response message */
    configRsp.cmdId = Smsgs_cmdIds_configRsp;
    configRsp.status = stat;

    /* Update the user */
    Ssf_configurationUpdate(&configRsp);

    /* Response the the source device */
    sendConfigRsp(&pDataInd->srcAddr, &configRsp);
}

/*!
 * @brief      Process the Image Data Request message.
 *
 * @param      pDataInd - pointer to the data indication information
 */
static void processImageDataRequest(ApiMac_mcpsDataInd_t *pDataInd) {
    //XXX:

    uint8_t msgBuf[SMSGS_IMAGE_DATA_RESPONSE_MSG_LEN];

    /* Make sure the message is the correct size */
    if(pDataInd->msdu.len == SMSGS_IMAGE_DATA_REQUEST_MSG_LEN) {

        /* Fill the image data response buffer */
        msgBuf[0] = (uint8_t) Smsgs_cmdIds_imageDataRsp;
        /* TODO: should make an enum for the below field and figure
         * out in what cases the sensor would not be ready
         */
        msgBuf[1] = true; /* Set to true if sensor is ready to RX image data */


        /* Send the image data response */
        Sensor_sendMsg(Smsgs_cmdIds_imageDataRsp,
                       &pDataInd->srcAddr,
                       true,
                       SMSGS_IMAGE_DATA_RESPONSE_MSG_LEN,
                       msgBuf);

        /* We are ready to start the EPD update, set event and reset indices*/
        Util_setEvent(&Sensor_events, SENSOR_START_EPD_UPDATE_EVT);
        rxPacketIndex = 0;
        txPacketIndex = 0;
    }
}

/*! XXX:
 * @brief      Start the EPD update process.  This will send the first two messages
 *             that the EPD device expects which are to erase flash that previously
 *             stored the image and to write the image file information to this flash.
 *
 * @return     Returns number of bytes sent via UART
 */
static uint16_t startEPDUpdate(void) {

    uint16_t bytesSent = 0;

    /* First we send the erase flash command */
    bytesSent = UART_write(uartHandle, (uint8_t *) &eraseCmdInfo,
                           sizeof(Command_info_t));
    // After a short pause we can send the payload
    // TODO: Replace this with something better
    int i;
    for(i = 0; i < 3; i++) { }
    // Send the message payload
    bytesSent += UART_write(uartHandle, (uint8_t *) &eraseFlashInfo,
                           sizeof(Flash_Parameter_Info_t));
    // Another short pause
    for(i = 0; i < 3; i++) { }
    bytesSent += UART_write(uartHandle, (uint8_t *) &imageFileCmdInfo,
                           sizeof(Command_info_t));
    // One last short pause
    for(i = 0; i < 3; i++) { }
    bytesSent += UART_write(uartHandle, (uint8_t *) &imageFileInfo,
                           sizeof(ImageFile_info_t));
    return bytesSent;
}

/*!
 * @brief      Receive image data coming from collecter.
 *
 * @param      pDataInd - pointer to the data indication information
 *
 * @return     Returns number of bytes sent via UART
 */
static uint8_t receiveImageData(ApiMac_mcpsDataInd_t *pDataInd) {

    uint8_t *pBuf = pDataInd->msdu.p;
    uint8_t bytesReceived = 0;

    /* Make sure the message is the correct size */
    if(pDataInd->msdu.len == IMAGE_DATA_RF_PACKET_SIZE) {
        // Put image data in the buffer
        int i;
        for(i = 1; i < IMAGE_DATA_RF_PACKET_SIZE; i++) {
            RingBuf_put(ringBufHandle, pBuf[i]);
            bytesReceived++;
        }
    }

    Ssf_toggleLED();

    /* Increment rxPacketIndex */
    rxPacketIndex++;

    /* Set the UART event if the RingBuf is full or we are done receiving packets*/
    if(RingBuf_isFull(ringBufHandle) || (rxPacketIndex >= IMAGE_DATA_NUM_RX_RF_PACKETS )) {
        Util_setEvent(&Sensor_events, SENSOR_SEND_EPD_IMAGE_DATA_EVT);
    }
    return bytesReceived;
}

/*!
 * @brief      Send image data packet
 *
 * @return     Returns number of bytes sent via UART
 */
static uint16_t sendImageDataPacket(void) {

    int i;
    uint16_t bytesSent = 0;
    uint8_t imageByte;

    /* Load data buffer with image data */
    if(RingBuf_getCount(ringBufHandle) > 0) {
        for(i = 0; i < IMAGE_DATA_UART_PACKET_SIZE; i++) {
            RingBuf_get(ringBufHandle, &imageByte);
            uartImageDataPacket[i] = imageByte;
        }
    }

    /* Send UART messages */
    bytesSent = UART_write(uartHandle, (uint8_t *) &imageCmdInfo,
                           sizeof(Command_info_t));
    //Short pause required by PDI Apps
    for(i = 0; i < 3; i++) {}
    bytesSent += UART_write(uartHandle, (uint8_t *) uartImageDataPacket,
                            IMAGE_DATA_UART_PACKET_SIZE);

    imageCmdInfo.Packet_index++;
    return bytesSent;
}

/*!
 * @brief      Send display image command to EPD
 *
 * @return     Returns number of bytes sent via UART
 */
static uint16_t displayImage(void) {

    uint16_t bytesSent = 0;
    int i;

    /* Send UART messages */
    bytesSent = UART_write(uartHandle, (uint8_t *) &showCmdInfo,
                           sizeof(Command_info_t));
    for(i = 0; i < 3; i++) {}
    bytesSent += UART_write(uartHandle, (uint8_t *) &showInfo,
                           sizeof(ShowEpd_Info_t));

    return bytesSent;
}

/*!
 * @brief   Build and send Config Response message
 *
 * @param   pDstAddr - Where to send the message
 * @param   pMsg - pointer to the Config Response
 *
 * @return  true if message was sent, false if not
 */
static bool sendConfigRsp(ApiMac_sAddr_t *pDstAddr, Smsgs_configRspMsg_t *pMsg)
{
    uint8_t msgBuf[SMSGS_CONFIG_RESPONSE_MSG_LENGTH];
    uint8_t *pBuf = msgBuf;

    *pBuf++ = (uint8_t) Smsgs_cmdIds_configRsp;
    pBuf = Util_bufferUint16(pBuf, pMsg->status);
    pBuf = Util_bufferUint16(pBuf, pMsg->frameControl);
    pBuf = Util_bufferUint32(pBuf, pMsg->reportingInterval);
    pBuf = Util_bufferUint32(pBuf, pMsg->pollingInterval);

    return (Sensor_sendMsg(Smsgs_cmdIds_configRsp, pDstAddr, true,
                    SMSGS_CONFIG_RESPONSE_MSG_LENGTH, msgBuf));
}

/*!
 * @brief   Filter the frameControl with readings supported by this device.
 *
 * @param   frameControl - suggested frameControl
 *
 * @return  new frame control settings supported
 */
static uint16_t validateFrameControl(uint16_t frameControl)
{
    uint16_t newFrameControl = 0;

#if defined(TEMP_SENSOR)
    if(frameControl & Smsgs_dataFields_tempSensor)
    {
        newFrameControl |= Smsgs_dataFields_tempSensor;
    }
#endif
#if defined(LIGHT_SENSOR)
    if(frameControl & Smsgs_dataFields_lightSensor)
    {
        newFrameControl |= Smsgs_dataFields_lightSensor;
    }
#endif
#if defined(HUMIDITY_SENSOR)
    if(frameControl & Smsgs_dataFields_humiditySensor)
    {
        newFrameControl |= Smsgs_dataFields_humiditySensor;
    }
#endif
    if(frameControl & Smsgs_dataFields_msgStats)
    {
        newFrameControl |= Smsgs_dataFields_msgStats;
    }
    if(frameControl & Smsgs_dataFields_configSettings)
    {
        newFrameControl |= Smsgs_dataFields_configSettings;
    }

    return (newFrameControl);
}


/*!
 * @brief   The device joined callback.
 *
 * @param   pDevInfo - This device's information
 * @param   pParentInfo - This is the parent's information
 */
static void jdllcJoinedCb(ApiMac_deviceDescriptor_t *pDevInfo,
                          Llc_netInfo_t *pParentInfo)
{
    uint32_t randomNum = 0;

    /* Copy the parent information */
    memcpy(&parentInfo, pParentInfo, sizeof(Llc_netInfo_t));

    /* Set the collector's address as the parent's address */
    if (pParentInfo->fh && !LRM_MODE)
    {
        collectorAddr.addrMode = ApiMac_addrType_extended;
        memcpy(collectorAddr.addr.extAddr, pParentInfo->devInfo.extAddress,
               (APIMAC_SADDR_EXT_LEN));
    }
    else
    {
        collectorAddr.addrMode = ApiMac_addrType_short;
        collectorAddr.addr.shortAddr = pParentInfo->devInfo.shortAddress;
    }

    /* Start the reporting timer */
    if(CONFIG_FH_ENABLE)
    {
        randomNum = ((ApiMac_randomByte() << 16) +
                     (ApiMac_randomByte() << 8) + ApiMac_randomByte());
        randomNum = (randomNum % configSettings.reportingInterval) +
                    SENSOR_MIN_POLL_TIME;
        Ssf_setReadingClock(randomNum);
    }
    else
    {
        Ssf_setReadingClock(configSettings.reportingInterval);
    }

    /* Inform the user of the joined information */
    Ssf_networkUpdate(rejoining, pDevInfo, pParentInfo);

    if((rejoining == false) && (pParentInfo->fh == false))
    {
#ifdef FEATURE_MAC_SECURITY
        ApiMac_status_t stat;
        /* Add the parent to the security device list */
        stat = Jdllc_addSecDevice(pParentInfo->devInfo.panID,
                                  pParentInfo->devInfo.shortAddress,
                                  &pParentInfo->devInfo.extAddress, 0);
        if(stat != ApiMac_status_success)
        {
            Ssf_displayError("Auth Error: 0x", (uint8_t)stat);
        }
#endif /* FEATURE_MAC_SECURITY */
    }

#if (CONFIG_SUPERFRAME_ORDER != 15) && defined(MAC_NO_AUTO_REQ)
    /*
     * Set MAC Auto Request to false to enable multiple poll requests
     * per beacon interval
     */
    ApiMac_mlmeSetReqBool(ApiMac_attribute_autoRequest, false);
#endif

    /* Calculate Join Time */
    joinTimeTicks = ICall_getTicks() - joinTimeTicks;
    Sensor_msgStats.joinTime = joinTimeTicks / TICKPERIOD_MS_US;
}

/*!
 * @brief   Disassociation indication callback.
 *
 * @param   pExtAddress - extended address
 * @param   reason - reason for disassociation
 */
static void jdllcDisassocIndCb(ApiMac_sAddrExt_t *pExtAddress,
                               ApiMac_disassocateReason_t reason)
{
    /* Stop the reporting timer */
    Ssf_setReadingClock(0);
    Ssf_clearNetworkInfo();

#ifdef FEATURE_NATIVE_OAD
    /* OAD abort with no auto resume */
    OADClient_abort(false);
#endif //FEATURE_NATIVE_OAD
}

/*!
 * @brief   Disassociation confirm callback to an application intiated
 *          disassociation request.
 *
 * @param   pExtAddress - extended address
 * @param   status - status of disassociation
 */
static void jdllcDisassocCnfCb(ApiMac_sAddrExt_t *pExtAddress,
                               ApiMac_status_t status)
{
    /* Stop the reporting timer */
    Ssf_setReadingClock(0);
    Ssf_clearNetworkInfo();

#ifdef FEATURE_NATIVE_OAD
    /* OAD abort with no auto resume */
    OADClient_abort(false);
#endif //FEATURE_NATIVE_OAD
}

/*!
 * @brief   JDLLC state change callback.
 *
 * @param   state - new state
 */
static void jdllcStateChangeCb(Jdllc_states_t state)
{
#ifdef FEATURE_NATIVE_OAD
    if( (state == Jdllc_states_joined) || (state == Jdllc_states_rejoined))
    {
#if (CONFIG_SUPERFRAME_ORDER == 15)
        /* resume an OAD that may have aborted */
        OADClient_resume(30000);
#else
        /* resume an OAD that may have aborted */
        OADClient_resume(60000);
#endif
    }
    else if(state == Jdllc_states_orphan)
    {
        /* OAD abort with no auto resume */
        OADClient_abort(false);
    }
#endif /* FEATURE_NATIVE_OAD */

    Ssf_stateChangeUpdate(state);
}


/*!
 * @brief   Manually read the sensors
 */
static void readSensors(void)
{
#if defined(TEMP_SENSOR)
    /* Read the temp sensor values */
    tempSensor.ambienceTemp = Ssf_readTempSensor();
    tempSensor.objectTemp =  tempSensor.ambienceTemp;
#endif
}
