/******************************************************************************

 @file fh_pib.c

 @brief TIMAC 2.0 FH PIB API

 Group: WCS LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2016-2017, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: simplelink_cc13x0_sdk_1_40_00_10"
 Release Date: 2017-06-27 19:05:53
 *****************************************************************************/

/******************************************************************************
 Includes
 *****************************************************************************/

#include <string.h>
#include "fh_api.h"
#include "fh_pib.h"
#include "fh_util.h"
#ifdef FEATURE_FH_OPT
#include "mac_mgmt.h"
#endif

#ifdef FEATURE_FREQ_HOP_MODE

/******************************************************************************
 Constants and definitions
 *****************************************************************************/

#define FH_PIB_ID_START         FHPIB_TRACK_PARENT_EUI
#define FH_PIB_ID_END           FHPIB_NUM_MAX_SLEEP_NODES
#define FH_PIB_ID_OFFSET        (0)
#define FH_PIB_ID_INVALID       (0xFFFF)

#ifndef offsetof
#define offsetof(T, member)     (__INTADDR__((&((T *)0)->member)))
#endif /* offsetof */

#define FH_UINT8_SIZE           (1)
#define FH_UINT16_SIZE          (2)
#define FH_UINT32_SIZE          (4)

/* FH related PIB access and min/max table type */
typedef struct __attribute__((__packed__))
{
    uint8_t     offset;
    uint8_t     len;
    uint32_t    min;
    uint32_t    max;
} FH_PibTbl_t;

/******************************************************************************
 Local variables
 *****************************************************************************/

/*! FH related PIB default values */
static const FHPIB_DB_t FHPIB_defaults =
{
    /*! macTrackParentEUI */
    FHPIB_DEFAULT_EUI,
    /*! macBcInterval : 4.25s */
    4250,
    /*! macUcExcludedChannels */
    {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00,
    },
    /*! macBcExcludedChannels */
    {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00,
    },
    /*! macUcDwellInterval */
    250,
    /*! macBcDwellInterval */
    250,
    /*! macClockDrift */
    20,
    /*! macTimingAccuracy */
    0x00,
    /*! macUcChannelFunction */
    0x00,
    /*! macBcChannelFunction */
    0x00,
    /*! macUseParentBSIE */
    0x00,
    /*! macBroadcastSchedId */
    0x0000,
    /*! macUcFixedChannel */
    0x0000,
    /*! macBcFixedChannel */
    0x0000,
    /*! macPanSize */
    0x0001,
    /*! macRoutingCost */
    0x00,
    /*! macRoutingMethod : 1(L3 RPL), 0(L2 MHDS)*/
    0x01,
    /*! macEapolReady : 1(ready to accept EAPOL message)*/
    0x01,
    /*! macFanTpsVersion : 1(WiSUN FAN version 1.0) */
    0x00,
    /*! macNetName */
    {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    },
    /*! macPanVersion */
    0x0000,
    /*! macGtk0Hash */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*! macGtk1Hash */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*! macGtk2Hash */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*! macGtk3Hash */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*! macNeighborValidTime : 120 mins*/
    120,
    /*! macBaseBackOff : 0 */
    0x08,
    /*! maximum non-sleep nodes */
    2,
    /*! maximum sleep node */
    (FHPIB_MAX_NUM_DEVICE -2)

};

/*!
 *  PIB access and min/max table.  min/max of 0/0 means not checked;
 *  if min/max are equal, element is read-only
 */
static const FH_PibTbl_t FH_PibTbl[] =
{
    /*!< FHPIB_TRACK_PARENT_EUI */
    {offsetof(FHPIB_DB_t, macTrackParentEUI), sizeof(sAddrExt_t), 0, 0},
    /*!< FHPIB_BC_INTERVAL */
    {offsetof(FHPIB_DB_t, macBcInterval), sizeof(uint32_t), 250, 16777215},
    /*!< FHPIB_UC_EXCLUDED_CHANNELS */
    {offsetof(FHPIB_DB_t, macUcExcludedChannels),
              FHPIB_MAX_BIT_MAP_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_BC_EXCLUDED_CHANNELS */
    {offsetof(FHPIB_DB_t, macBcExcludedChannels),
              FHPIB_MAX_BIT_MAP_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_UC_DWELL_INTERVAL */
    {offsetof(FHPIB_DB_t, macUcDwellInterval), sizeof(uint8_t), 15, 250},
    /*!< FHPIB_BC_DWELL_INTERVAL */
    {offsetof(FHPIB_DB_t, macBcDwellInterval), sizeof(uint8_t), 15, 250},
    /*!< FHPIB_CLOCK_DRIFT */
    {offsetof(FHPIB_DB_t, macClockDrift), sizeof(uint8_t), 0, 255},
    /*!< FHPIB_TIMING_ACCURACY */
    {offsetof(FHPIB_DB_t, macTimingAccuracy), sizeof(uint8_t), 0, 255},
    /*!< FHPIB_UC_CHANNEL_FUNCTION */
    {offsetof(FHPIB_DB_t, macUcChannelFunction), sizeof(uint8_t), 0, 3},
    /*!< FHPIB_BC_CHANNEL_FUNCTION */
    {offsetof(FHPIB_DB_t, macBcChannelFunction), sizeof(uint8_t), 0, 3},
    /*!< FHPIB_USE_PARENT_BS_IE */
    {offsetof(FHPIB_DB_t, macUseParentBSIE), sizeof(uint8_t), 0, 1},
    /*!< FHPIB_BROCAST_SCHED_ID */
    {offsetof(FHPIB_DB_t, macBroadcastSchedId), sizeof(uint16_t), 0, 65535},
    /*!< FHPIB_UC_FIXED_CHANNEL */
    {offsetof(FHPIB_DB_t, macUcFixedChannel), sizeof(uint16_t), 0, 255},
    /*!< FHPIB_BC_FIXED_CHANNEL */
    {offsetof(FHPIB_DB_t, macBcFixedChannel), sizeof(uint16_t), 0, 255},
    /*!< FHPIB_PAN_SIZE */
    {offsetof(FHPIB_DB_t, macPanSize), sizeof(uint16_t), 0, 65535},
    /*!< FHPIB_ROUTING_COST */
    {offsetof(FHPIB_DB_t, macRoutingCost), sizeof(uint8_t), 0, 255},
    /*!< FHPIB_ROUTING_METHOD */
    {offsetof(FHPIB_DB_t, macRoutingMethod), sizeof(uint8_t), 0, 1},
    /*!< FHPIB_EAPOL_READY */
    {offsetof(FHPIB_DB_t, macEapolReady), sizeof(uint8_t), 0, 1},
    /*!< FHPIB_FAN_TPS_VERSION */
    {offsetof(FHPIB_DB_t, macFanTpsVersion), sizeof(uint8_t), 0, 7},
    /*!< FHPIB_NET_NAME */
    {offsetof(FHPIB_DB_t, macNetName),
              FHPIB_NET_NAME_SIZE_MAX*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_PAN_VERSION */
    {offsetof(FHPIB_DB_t, macPanVersion), sizeof(uint16_t), 0, 65535},
    /*!< FHPIB_GTK_0_HASH */
    {offsetof(FHPIB_DB_t, macGtk0Hash),
              FHPIB_GTK_HASH_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_GTK_1_HASH */
    {offsetof(FHPIB_DB_t, macGtk1Hash),
              FHPIB_GTK_HASH_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_GTK_2_HASH */
    {offsetof(FHPIB_DB_t, macGtk2Hash),
              FHPIB_GTK_HASH_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_GTK_3_HASH */
    {offsetof(FHPIB_DB_t, macGtk3Hash),
              FHPIB_GTK_HASH_SIZE*sizeof(uint8_t), 0, 0},
    /*!< FHPIB_HOPPING_NEIGHBOR_VALID_TIME */
    {offsetof(FHPIB_DB_t, macNeighborValidTime), sizeof(uint16_t), 5, 600},
    /*!< CSMA Base Backoff */
    {offsetof(FHPIB_DB_t, macBaseBackOff), sizeof(uint8_t), 0, 16},
    /*!< maximum non-sleep nodes */
    {offsetof(FHPIB_DB_t, macMaxNonSleepNodes), sizeof(uint8_t), 0, 50},
    /*!< maximum sleep node */
    {offsetof(FHPIB_DB_t, macMaxSleepNodes), sizeof(uint8_t), 0, 50},

};

/******************************************************************************
 Glocal variables
 *****************************************************************************/

/* FH related PIB */
FHPIB_DB_t FHPIB_db;

/******************************************************************************
 Local Function Prototypes
 *****************************************************************************/

static uint16_t FHPIB_getIndex(uint16_t fhPibId)
{
    if((fhPibId >= FH_PIB_ID_START) && (fhPibId <= FH_PIB_ID_END))
    {
        return(fhPibId - FH_PIB_ID_START + FH_PIB_ID_OFFSET);
    }
    else
    {
        return(FH_PIB_ID_INVALID);
    }
}

/******************************************************************************
 Public Functions
 *****************************************************************************/

/*!
 FHPIB_reset

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API void FHPIB_reset(void)
{
    /* copy FH-related PIB defaults */
    FHPIB_db = FHPIB_defaults;
}

/*!
 FHPIB_getLen

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API uint8_t FHPIB_getLen(uint16_t fhPibId)
{
    uint16_t i;

    if((i = FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(0);
    }

    return (FH_PibTbl[i].len);
}

/*!
 FHPIB_set

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_set(uint16_t fhPibId, void *pData)
{
    uint16_t i;
    uint8_t pibLen;
    uint8_t numChannels, maxChannels, excludedChannels;
    halIntState_t intState;

    if((i = FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(FHAPI_STATUS_ERR_NOT_SUPPORTED_PIB);
    }

#ifdef FEATURE_FH_OPT
    if ((fhPibId == FHPIB_NUM_MAX_SLEEP_NODES)
     || (fhPibId == FHPIB_NUM_MAX_NON_SLEEP_NODES)
     || (fhPibId == FHPIB_BC_INTERVAL)
     || (fhPibId == FHPIB_BC_DWELL_INTERVAL))
    {
        if (macCheckPhyLRMMode())
        {
            return(FHAPI_STATUS_ERR_NOT_SUPPORTED_PIB);
        }
    }
#endif

    /* do range check; no range check if min and max are zero */
    if ((FH_PibTbl[i].min != 0) || (FH_PibTbl[i].max != 0))
    {
        /* if min == max, this is a read-only attribute */
        if (FH_PibTbl[i].min == FH_PibTbl[i].max)
        {
            return(FHAPI_STATUS_ERR_READ_ONLY_PIB);
        }

        /* range check for general case */
        pibLen = FH_PibTbl[i].len;
        switch(pibLen)
        {
            case FH_UINT8_SIZE:
                if ((*((uint8_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint8_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            case FH_UINT16_SIZE:
                if ((*((uint16_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint16_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            case FH_UINT32_SIZE:
                if ((*((uint32_t *) pData) < FH_PibTbl[i].min)
                 || (*((uint32_t *) pData) > FH_PibTbl[i].max))
                {
                    return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
                }
                break;
            default:
                break;
        }
    }

    HAL_ENTER_CRITICAL_SECTION(intState);
    switch(fhPibId)
    {
        case FHPIB_NET_NAME:
            memset((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, 0,
                    FH_PibTbl[i].len);
            memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                    strlen((char *)pData) < FH_PibTbl[i].len ?
                    strlen((char *)pData) : FH_PibTbl[i].len);
            break;
        case FHPIB_BC_FIXED_CHANNEL:
        case FHPIB_UC_FIXED_CHANNEL:
            maxChannels = FHUTIL_getMaxChannels();
            if (*((uint16_t *) pData) < maxChannels)
            {
                memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                        FH_PibTbl[i].len);
            }
            else
            {
                HAL_EXIT_CRITICAL_SECTION(intState);
                return FHAPI_STATUS_ERR_INVALID_PARAM_PIB;
            }
            break;
        case FHPIB_BC_EXCLUDED_CHANNELS:
        case FHPIB_UC_EXCLUDED_CHANNELS:
            maxChannels = FHUTIL_getMaxChannels();
            excludedChannels = FHUTIL_getBitCounts((uint8_t *)pData, maxChannels);
            numChannels = maxChannels - excludedChannels;
            if(numChannels > 0)
            {
                memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                        FH_PibTbl[i].len);
            }
            else
            {
                HAL_EXIT_CRITICAL_SECTION(intState);
                return FHAPI_STATUS_ERR_INVALID_PARAM_PIB;
            }
            if(fhPibId == FHPIB_UC_EXCLUDED_CHANNELS)
            {
                FH_hnd.ucNumChannels = numChannels;
            }
            else if(fhPibId == FHPIB_BC_EXCLUDED_CHANNELS)
            {
                FH_hnd.bcNumChannels = numChannels;
            }
            break;
        default:
            memcpy((uint8_t *)&FHPIB_db + FH_PibTbl[i].offset, pData,
                    FH_PibTbl[i].len);
            break;
    }
    /* sanity checking for number of non-sllep and leep node */
    if (fhPibId == FHPIB_NUM_MAX_NON_SLEEP_NODES)
    {
        /* reset the sleep node to zero */
        FHPIB_db.macMaxSleepNodes = 0;
    }

    if (fhPibId == FHPIB_NUM_MAX_SLEEP_NODES)
    {
        /* sanity checking to make sure total number is not greater than MAX device
         * table size
         */
        if (  ((FHPIB_db.macMaxNonSleepNodes + FHPIB_db.macMaxSleepNodes) > FHPIB_MAX_NUM_DEVICE ) ||
              ((FHPIB_db.macMaxNonSleepNodes == 0 ) && (FHPIB_db.macMaxSleepNodes == 0) ) )
        {
            /* back to default */
            FHPIB_db.macMaxNonSleepNodes = FHPIB_defaults.macMaxNonSleepNodes;
            FHPIB_db.macMaxSleepNodes = FHPIB_defaults.macMaxSleepNodes;
            HAL_EXIT_CRITICAL_SECTION(intState);
            return(FHAPI_STATUS_ERR_INVALID_PARAM_PIB);
        }
    }
    HAL_EXIT_CRITICAL_SECTION(intState);
    return(FHAPI_STATUS_SUCCESS);
}

/*!
 FHPIB_get

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_get(uint16_t fhPibId, void *pData)
{
    uint16_t i;
    halIntState_t intState;

    if((i = FHPIB_getIndex(fhPibId)) == FH_PIB_ID_INVALID)
    {
        return(FHAPI_STATUS_ERR_NOT_SUPPORTED_PIB);
    }

    HAL_ENTER_CRITICAL_SECTION(intState);
    memcpy(pData, (uint8_t *)&FHPIB_db + FH_PibTbl[i].offset,
            FH_PibTbl[i].len);
    HAL_EXIT_CRITICAL_SECTION(intState);
    return(FHAPI_STATUS_SUCCESS);
}
#else
/*!
 FHPIB_getLen

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API uint8_t FHPIB_getLen(uint16_t fhPibId)
{
    (void)fhPibId;
    return(0);
}

/*!
 FHPIB_get

 Public function defined in fh_pib.h
 */
MAC_INTERNAL_API FHAPI_status FHPIB_get(uint16_t fhPibId, void *pData)
{
    (void)fhPibId;
    (void)pData;
    return(FHAPI_STATUS_ERR);
}
#endif /* FEATURE_FREQ_HOP_MODE */
