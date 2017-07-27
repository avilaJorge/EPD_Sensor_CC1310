/******************************************************************************

 @file  mac_pib.c

 @brief This module contains procedures for the MAC PIB.

 Group: WCS, LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2006-2017, Texas Instruments Incorporated
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

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_mcu.h"
#include "hal_board.h"
#include "mac_api.h"
#include "mac_spec.h"
#include "mac_radio.h"
#include "mac_low_level.h"
#include "mac_radio_defs.h"
#include "mac_main.h"
#include "mac_pib.h"
#include "mac_mgmt.h"
#include "osal.h"
#include <stddef.h>

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* Attribute index constants, based on attribute ID values from spec */
#define MAC_ATTR_SET1_START       0x40
#define MAC_ATTR_SET1_END         0x64
#define MAC_ATTR_SET1_OFFSET      0
#define MAC_ATTR_SET2_START       0xE0
#define MAC_ATTR_SET2_END         0xF5
#define MAC_ATTR_SET2_OFFSET      (MAC_ATTR_SET1_END - MAC_ATTR_SET1_START + MAC_ATTR_SET1_OFFSET + 1)

/* frame response values */
#define MAC_MAX_FRAME_RESPONSE_MIN  143
#define MAC_MAX_FRAME_RESPONSE_MAX  25776

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* PIB access and min/max table type */
typedef struct
{
  uint8     offset;
  uint8     len;
  uint8     min;
  uint8     max;
} macPibTbl_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* PIB default values */
static CODE const macPib_t macPibDefaults =
{
  54,                                         /* ackWaitDuration */
  FALSE,                                      /* associationPermit */
  TRUE,                                       /* autoRequest */
  FALSE,                                      /* battLifeExt */
  6,                                          /* battLifeExtPeriods */

  NULL,                                       /* *pMacBeaconPayload */
  0,                                          /* beaconPayloadLength */
  MAC_BO_NON_BEACON,                          /* beaconOrder */
  0,                                          /* beaconTxTime */
  0,                                          /* bsn */

  {0, SADDR_MODE_EXT},                        /* coordExtendedAddress */
  MAC_SHORT_ADDR_NONE,                        /* coordShortAddress */
  0,                                          /* dsn */
  FALSE,                                      /* gtsPermit */
  4,                                          /* maxCsmaBackoffs */

  MAC_DEFAULT_MIN_BE,                         /* minBe */
  0xFFFF,                                     /* panId */
  FALSE,                                      /* promiscuousMode */
  FALSE,                                      /* rxOnWhenIdle */
  MAC_SHORT_ADDR_NONE,                        /* shortAddress */

  MAC_SO_NONE,                                /* superframeOrder */
  0x01F4,                                     /* transactionPersistenceTime */
  FALSE,                                      /* assocciatedPanCoord */
  MAC_DEFAULT_MAX_BE,                         /* maxBe */
  8000,                                       /* maxFrameTotalWaitTime */
  3,                                          /* maxFrameRetries */
  32,                                         /* responseWaitTime */
  0,                                          /* syncSymbolOffset */
  TRUE,                                       /* timeStampSupported */
  FALSE,                                      /* securityEnabled */

  0,                                          /* ebsn */
  0,                                          /* Enhanced beacon order */
  16383,                                      /* Non Beacon PAN enhanced beacon order */
  15,                                         /* ehanced beacon offset time slot */
  FALSE,                                      /* MPM IE to be included */

  MAC_PREAMBLE_LEN_8,                         /* FSK presamble len, 4 to 31 */
  0,                                          /* MR-FSK PHY SFD, valid values 0, 1 */

  /* Proprietary */
  14,                                         /* phyTransmitPower */
  MAC_CHAN_LOWEST,                            /* logicalChannel */
  {0, SADDR_MODE_EXT},                        /* extendedAddress */
  1,                                          /* altBe */
  MAC_BO_NON_BEACON,                          /* deviceBeaconOrder */
  0,                                          /* power savings for rf4ce */
  0,                                          /* default is frame Ver is always 0,
                                                 except for security enabled packets */
  9,                                          /* default channel page */
  MAC_STD_US_915_PHY_1,                       /* current PHY Descriptor ID */
  0,                                          /* fcsType */

  0,                                          /* diagsRxCrcPass */
  0,                                          /* diagsRxCrcFail */
  0,                                          /* diagsRxBcast */
  0,                                          /* diagsTxBcast */
  0,                                          /* diagsRxUcast */
  0,                                          /* diagsTxUcast */
  0,                                          /* diagsTxUcastRetry */
  0,                                          /* diagsTxUcastFail */
  0,                                          /* diagsRxSecFail */
  0,                                          /* diagsTxSecFail */
  -83,                                        /* LBT RSSI Threshold */
  0                                           /* range extender mode */
};


/* PIB access and min/max table.  min/max of 0/0 means not checked; if min/max are
 * equal, element is read-only
 */
static CODE const macPibTbl_t macPibTbl[] =
{
  {offsetof(macPib_t, ackWaitDuration), sizeof(uint8), 54, 54},                      /* MAC_ACK_WAIT_DURATION */
  {offsetof(macPib_t, associationPermit), sizeof(bool), FALSE, TRUE},                /* MAC_ASSOCIATION_PERMIT */
  {offsetof(macPib_t, autoRequest), sizeof(bool), FALSE, TRUE},                      /* MAC_AUTO_REQUEST */
  {offsetof(macPib_t, battLifeExt), sizeof(bool), FALSE, TRUE},                      /* MAC_BATT_LIFE_EXT */
  {offsetof(macPib_t, battLifeExtPeriods), sizeof(uint8), 6, 6},                     /* MAC_BATT_LIFE_EXT_PERIODS */

  {offsetof(macPib_t, pBeaconPayload), sizeof(uint8 *), 0, 0},                       /* MAC_BEACON_PAYLOAD */
  {offsetof(macPib_t, beaconPayloadLength), sizeof(uint8), 0, MAC_PIB_MAX_BEACON_PAYLOAD_LEN}, /* MAC_BEACON_PAYLOAD_LENGTH */
  {offsetof(macPib_t, beaconOrder), sizeof(uint8), 0, 15},                           /* MAC_BEACON_ORDER */
  {offsetof(macPib_t, beaconTxTime), sizeof(uint32), 1, 1},                          /* MAC_BEACON_TX_TIME */
  {offsetof(macPib_t, bsn), sizeof(uint8), 0x00, 0xFF},                              /* MAC_BSN */

  {offsetof(macPib_t, coordExtendedAddress.addr.extAddr), sizeof(sAddrExt_t), 0, 0}, /* MAC_COORD_EXTENDED_ADDRESS */
  {offsetof(macPib_t, coordShortAddress), sizeof(uint16), 0, 0},                     /* MAC_COORD_SHORT_ADDRESS */
  {offsetof(macPib_t, dsn), sizeof(uint8), 0x00, 0xFF},                              /* MAC_DSN */
  {offsetof(macPib_t, gtsPermit), sizeof(bool), FALSE, TRUE},                        /* MAC_GTS_PERMIT */

  /* Range of maxCsmaBackoffs is between 0 and 5 in IEEE 802.15.4.
   * Such restriction is removed here to allow 802.15.4 non-compliant experimental
   * applications.
   * To be compliant with 802.15.4, application or upper layer must not set this
   * PIB attribute out of range of 802.15.4.
   * The range is still restricted to 254 since value 255 would cause backoff
   * counter to overflow, wrap around and loop infinitely. */
  {offsetof(macPib_t, maxCsmaBackoffs), sizeof(uint8), 0, 254},                      /* MAC_MAX_CSMA_BACKOFFS */

  {offsetof(macPib_t, minBe), sizeof(uint8), 0, 8},                                  /* MAC_MIN_BE */
  {offsetof(macPib_t, panId), sizeof(uint16), 0, 0},                                 /* MAC_PAN_ID */
  {offsetof(macPib_t, promiscuousMode), sizeof(bool), FALSE, TRUE},                  /* MAC_PROMISCUOUS_MODE */
  {offsetof(macPib_t, rxOnWhenIdle), sizeof(bool), FALSE, TRUE},                     /* MAC_RX_ON_WHEN_IDLE */
  {offsetof(macPib_t, shortAddress), sizeof(uint16), 0, 0},                          /* MAC_SHORT_ADDRESS */

  {offsetof(macPib_t, superframeOrder), sizeof(uint8), 0, 15},                       /* MAC_SUPERFRAME_ORDER */
  {offsetof(macPib_t, transactionPersistenceTime), sizeof(uint16), 0, 0},            /* MAC_TRANSACTION_PERSISTENCE_TIME */
  {offsetof(macPib_t, associatedPanCoord), sizeof(bool), FALSE, TRUE},               /* MAC_ASSOCIATED_PAN_COORD */

  /* Range of maxBe is between 3 and 8 in IEEE 802.15.4.
   * Such restriction is removed here to allow 802.15.4 non-compliant experimental
   * applications.
   * To be compliant with 802.15.4, application or upper layer must not set this
   * PIB attribute out of range of 802.15.4. */
  {offsetof(macPib_t, maxBe), sizeof(uint8), 0, 8},                                  /* MAC_MAX_BE */
  {offsetof(macPib_t, maxFrameTotalWaitTime), sizeof(uint16), 0x00, 0xFF},           /* MAC_MAX_FRAME_RESPONSE_TIME */

  {offsetof(macPib_t, maxFrameRetries), sizeof(uint8), 0, 7},                        /* MAC_MAX_FRAME_RETRIES */
  {offsetof(macPib_t, responseWaitTime), sizeof(uint8), 2, 64},                      /* MAC_RESPONSE_WAIT_TIME */
  {offsetof(macPib_t, syncSymbolOffset), sizeof(uint8), 0, 0},                       /* MAC_SYNC_SYMBOL_OFFSET */
  {offsetof(macPib_t, timeStampSupported), sizeof(bool), FALSE, TRUE},               /* MAC_TIMESTAMP_SUPPORTED */
  {offsetof(macPib_t, securityEnabled), sizeof(bool), FALSE, TRUE},                  /* MAC_SECURITY_ENABLED */

  {offsetof(macPib_t, ebsn), sizeof(uint8), 0x00, 0xFF},                             /* MAC_EBSN                */
  {offsetof(macPib_t, eBeaconOrder), sizeof(uint8), 0, 15 },                         /* MAC_EBEACON_ORDER       */
  {offsetof(macPib_t, eBeaconOrderNBPAN), sizeof(uint16), 0x00, 0xFF},               /* MAC_EBEACON_ORDER_NBPAN */
  {offsetof(macPib_t, offsetTimeSlot), sizeof(uint8), 1, 15},                        /* MAC_OFFSET_TIMESLOT     */
  {offsetof(macPib_t, includeMPMIE), sizeof(bool), FALSE, TRUE},                     /* MAC_INCLUDE_MPMIE       */

  {offsetof(macPib_t, fskPreambleLen), sizeof(uint8), MAC_MIN_PREAMBLE_LEN, MAC_MAX_PREAMBLE_LEN},              /* MAC_PHY_FSK_PREAMBLE_LEN */
  {offsetof(macPib_t, mrFSKSFD), sizeof(uint8), 0, 1},                               /* MAC_PHY_MRFSKSFD        */

  /* Proprietary PIBs */
  {offsetof(macPib_t, phyTransmitPower), sizeof(uint8), 0, 0xFF},                   /* MAC_PHY_TRANSMIT_POWER_SIGNED */
  {offsetof(macPib_t, logicalChannel), sizeof(uint8), MAC_CHAN_LOWEST, MAC_CHAN_HIGHEST},     /* MAC_LOGICAL_CHANNEL */
  {offsetof(macPib_t, extendedAddress.addr.extAddr), sizeof(sAddrExt_t), 0, 0},      /* MAC_EXTENDED_ADDRESS */
  {offsetof(macPib_t, altBe), sizeof(uint8), 0, 8},                                  /* MAC_ALT_BE */
  {offsetof(macPib_t, deviceBeaconOrder), sizeof(uint8), 0, 15},                     /* MAC_DEVICE_BEACON_ORDER */
  {offsetof(macPib_t, rf4cepowerSavings), sizeof(uint8), 0, 1},                      /* MAC_RF4CE_POWER_SAVINGS */
  {offsetof(macPib_t, frameVersionSupport), sizeof(uint8), 0, 0xFF},                 /* MAC_FRAME_VERSION_SUPPORT */
  {offsetof(macPib_t, channelPage), sizeof(uint8), 9, 10},                            /* MAC_CHANNEL_PAGE */
  {offsetof(macPib_t, curPhyID), sizeof(uint8), MAC_MRFSK_STD_PHY_ID_BEGIN, MAC_MRFSK_GENERIC_PHY_ID_END}, /* MAC_PHY_CURRENT_DESCRIPTOR_ID */
  {offsetof(macPib_t, fcsType), sizeof(bool), FALSE, TRUE },                         /* MAC_FCS_TYPE */

  {offsetof(macPib_t, diagsRxCrcPass), sizeof(uint32), 0, 0},                        /* MAC_DIAGS_RX_CRC_PASS */
  {offsetof(macPib_t, diagsRxCrcFail), sizeof(uint32), 0, 0},                        /* MAC_DIAGS_RX_CRC_FAIL */
  {offsetof(macPib_t, diagsRxBcast), sizeof(uint32), 0, 0},                          /* MAC_DIAGS_RX_BCAST */
  {offsetof(macPib_t, diagsTxBcast), sizeof(uint32), 0, 0},                          /* MAC_DIAGS_TX_BCAST */
  {offsetof(macPib_t, diagsRxUcast), sizeof(uint32), 0, 0},                          /* MAC_DIAGS_RX_UCAST */
  {offsetof(macPib_t, diagsTxUcast), sizeof(uint32), 0, 0},                          /* MAC_DIAGS_TX_UCAST */
  {offsetof(macPib_t, diagsTxUcastRetry), sizeof(uint32), 0, 0},                     /* MAC_DIAGS_TX_UCAST_RETRY */
  {offsetof(macPib_t, diagsTxUcastFail), sizeof(uint32), 0, 0},                      /* MAC_DIAGS_TX_UCAST_FAIL */
  {offsetof(macPib_t, diagsRxSecFail), sizeof(uint32), 0, 0},                        /* MAC_DIAGS_RX_SEC_FAIL */
  {offsetof(macPib_t, diagsTxSecFail), sizeof(uint32), 0, 0},                        /* MAC_DIAGS_TX_SEC_FAIL */
  {offsetof(macPib_t, rssiThreshold), sizeof(int8), 0, 0xFF},                        /* MAC_RSSI_THRESHOLD */
  {offsetof(macPib_t, rangeExt), sizeof(uint8), 0, 1},                               /* MAC_RANGE_EXTENDER */
};

/* Invalid PIB table index used for error code */
#define MAC_PIB_INVALID     ((uint8) (sizeof(macPibTbl) / sizeof(macPibTbl[0])))

#ifndef FREQ_2_4G
/* MR-FSK channel center frequencies */
#define MAC_STD_US_915_BAND_MODE_1_CENTER_FREQ_KHZ              902200
#define MAC_STD_US_915_BAND_MODE_2_CENTER_FREQ_KHZ              902400
#define MAC_STD_ETSI_863_BAND_MODE_1_CENTER_FREQ_KHZ            863125
#define MAC_GENERIC_CHINA_433_BAND_MODE_1_CENTER_FREQ_KHZ       433300
#define MAC_GENERIC_US_LRM_915_BAND_MODE_1_CENTER_FREQ_KHZ      902200
#define MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_CENTER_FREQ_KHZ   433300
#define MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_CENTER_FREQ_KHZ    863125

/* MR-FSK channel spacing */
#define MAC_STD_US_915_BAND_MODE_1_CHAN_SPACING_KHZ             200
#define MAC_STD_US_915_BAND_MODE_2_CHAN_SPACING_KHZ             400
#define MAC_STD_ETSI_863_BAND_MODE_1_CHAN_SPACING_KHZ           200
#define MAC_GENERIC_CHINA_433_BAND_MODE_1_CHAN_SPACING_KHZ      200
#define MAC_GENERIC_US_LRM_915_BAND_MODE_1_CHAN_SPACING_KHZ     200
#define MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_CHAN_SPACING_KHZ  200
#define MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_CHAN_SPACING_KHZ   200

/* MR-FSK Number of Channels */
#define MAC_STD_US_915_BAND_MODE_1_TOTAL_CHANNELS               129
#define MAC_STD_US_915_BAND_MODE_2_TOTAL_CHANNELS               64
#define MAC_STD_ETSI_863_BAND_MODE_1_TOTAL_CHANNELS             34
#define MAC_GENERIC_CHINA_433_BAND_MODE_1_TOTAL_CHANNELS        7
#define MAC_GENERIC_US_LRM_915_BAND_MODE_1_TOTAL_CHANNELS       129
#define MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_TOTAL_CHANNELS    7
#define MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_TOTAL_CHANNELS     34

/* MAC 15.4g Standard PHY Descriptor Table */
CODE const macMRFSKPHYDesc_t macMRFSKStdPhyTable[MAC_STANDARD_PHY_DESCRIPTOR_ENTRIES] =
{
  { MAC_STD_US_915_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_STD_US_915_BAND_MODE_1_CHAN_SPACING_KHZ,
    MAC_STD_US_915_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_50_K,
  MAC_MODULATION_INDEX_2FSK_50K,
    MAC_CCA_TYPE_CSMA_CA },
  { MAC_STD_US_915_BAND_MODE_2_CENTER_FREQ_KHZ, MAC_STD_US_915_BAND_MODE_2_CHAN_SPACING_KHZ,
    MAC_STD_US_915_BAND_MODE_2_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_150_K,
  MAC_MODULATION_INDEX_2FSK_150K,
    MAC_CCA_TYPE_CSMA_CA },
  { MAC_STD_ETSI_863_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_STD_ETSI_863_BAND_MODE_1_CHAN_SPACING_KHZ,
    MAC_STD_ETSI_863_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_50_K,
  MAC_MODULATION_INDEX_2FSK_50K,
    MAC_CCA_TYPE_LBT }
};

/* MAC 15.4g Generic PHY Descriptor Table */
macMRFSKPHYDesc_t macMRFSKGenPhyTable[MAC_GENERIC_PHY_DESCRIPTOR_ENTRIES] =
{
 { MAC_GENERIC_CHINA_433_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_GENERIC_CHINA_433_BAND_MODE_1_CHAN_SPACING_KHZ,
   MAC_GENERIC_CHINA_433_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_50_K,
 MAC_MODULATION_INDEX_2FSK_50K,
   MAC_CCA_TYPE_CSMA_CA },
 { MAC_GENERIC_US_LRM_915_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_GENERIC_US_LRM_915_BAND_MODE_1_CHAN_SPACING_KHZ,
   MAC_GENERIC_US_LRM_915_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_20_K,
 MAC_MODULATION_INDEX_2FSK_5K,
   MAC_CCA_TYPE_CSMA_CA },
 { MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_CHAN_SPACING_KHZ,
   MAC_GENERIC_CHINA_LRM_433_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_20_K,
 MAC_MODULATION_INDEX_2FSK_5K,
   MAC_CCA_TYPE_CSMA_CA },
 { MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_CENTER_FREQ_KHZ, MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_CHAN_SPACING_KHZ,
   MAC_GENERIC_ETSI_LRM_863_BAND_MODE_1_TOTAL_CHANNELS, MAC_2_FSK_MODULATION, MAC_MRFSK_SYMBOL_RATE_20_K,
 MAC_MODULATION_INDEX_2FSK_5K,
   MAC_CCA_TYPE_LBT },
};
#else
/* MR-FSK channel center frequencies */
#define MAC_IEEE_CENTER_FREQ_KHZ              2405000
/* MR-FSK channel spacing */
#define MAC_IEEE_CHAN_SPACING_KHZ             5000
/* MR-FSK Number of Channels */
#define MAC_IEEE_TOTAL_CHANNELS               16

/* MAC 15.4 IEEE PHY Descriptor Table */
CODE const macIEEEPHYDesc_t macIEEEPhyTable =
  { MAC_IEEE_CENTER_FREQ_KHZ, MAC_IEEE_CHAN_SPACING_KHZ, MAC_IEEE_TOTAL_CHANNELS, MAC_CCA_TYPE_CSMA_CA };
#endif
/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* MAC PIB */
macPib_t macPib;

#if defined (FEATURE_BEACON_MODE) || defined (FEATURE_NON_BEACON_MODE)
#ifdef ICALL_LITE
uint8 macBeaconPayload[MAC_PIB_MAX_BEACON_PAYLOAD_LEN] = {0};
#endif /* !ICALL_LITE */
#endif

extern uint8 MAC_MlmeGetReqSize( uint8 pibAttribute );

#if defined( FEATURE_MAC_PIB_PTR )

/* Pointer to the MAC PIB */
macPib_t* pMacPib = &macPib;

/**************************************************************************************************
 * @fn          MAC_MlmeSetActivePib
 *
 * @brief       This function initializes the PIB.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void MAC_MlmeSetActivePib( void* pPib )
{
  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);
  pMacPib = (macPib_t *)pPib;
  HAL_EXIT_CRITICAL_SECTION(intState);
}
#endif /* FEATURE_MAC_PIB_PTR */

/**************************************************************************************************
 * @fn          macPibReset
 *
 * @brief       This function initializes the PIB.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
MAC_INTERNAL_API void macPibReset(void)
{
  /* copy PIB defaults */
#if defined( FEATURE_MAC_PIB_PTR )
  *pMacPib = macPibDefaults;
#else
  macPib = macPibDefaults;
#endif /* FEATURE_MAC_PIB_PTR */

  /* initialize random sequence numbers */
  pMacPib->dsn = macRadioRandomByte();
  pMacPib->bsn = macRadioRandomByte();
}

/**************************************************************************************************
 * @fn          macPibIndex
 *
 * @brief       This function takes an PIB attribute and returns the index in to
 *              macPibTbl for the attribute.
 *
 * input parameters
 *
 * @param       pibAttribute - PIB attribute to look up.
 *
 * output parameters
 *
 * None.
 *
 * @return      Index in to macPibTbl for the attribute or MAC_PIB_INVALID.
 **************************************************************************************************
 */
MAC_INTERNAL_API uint8 macPibIndex(uint8 pibAttribute)
{
  if ((pibAttribute >= MAC_ATTR_SET1_START) && (pibAttribute <= MAC_ATTR_SET1_END))
  {
    return (pibAttribute - MAC_ATTR_SET1_START + MAC_ATTR_SET1_OFFSET);
  }
  else if ((pibAttribute >= MAC_ATTR_SET2_START) && (pibAttribute <= MAC_ATTR_SET2_END))
  {
    return (pibAttribute - MAC_ATTR_SET2_START + MAC_ATTR_SET2_OFFSET);
  }
  else
  {
    return MAC_PIB_INVALID;
  }
}


/**************************************************************************************************
 * @fn          MAC_MlmeGetReq
 *
 * @brief       This direct execute function retrieves an attribute value
 *              from the MAC PIB.
 *
 * input parameters
 *
 * @param       pibAttribute - The attribute identifier.
 * @param       pValue - pointer to the attribute value.
 *
 * output parameters
 *
 * @param       pValue - pointer to the attribute value.
 *
 * @return      The status of the request, as follows:
 *              MAC_SUCCESS Operation successful.
 *              MAC_UNSUPPORTED_ATTRIBUTE Attribute not found.
 *
 **************************************************************************************************
 */
uint8 MAC_MlmeGetReq(uint8 pibAttribute, void *pValue)
{
  uint8         i;
  halIntState_t intState;

  if ((i = macPibIndex(pibAttribute)) == MAC_PIB_INVALID)
  {
    return MAC_UNSUPPORTED_ATTRIBUTE;
  }

  HAL_ENTER_CRITICAL_SECTION(intState);
  osal_memcpy(pValue, (uint8 *) pMacPib + macPibTbl[i].offset, macPibTbl[i].len);
  HAL_EXIT_CRITICAL_SECTION(intState);
  return MAC_SUCCESS;
}

/**************************************************************************************************
 * @fn          MAC_MlmeGetReqSize
 *
 * @brief       This direct execute function gets the MAC PIB attribute size
 *
 * input parameters
 *
 * @param       pibAttribute - The attribute identifier.
 *
 * output parameters
 *
 * None.
 *
 * @return      size in bytes
 *
 **************************************************************************************************
 */
uint8 MAC_MlmeGetReqSize( uint8 pibAttribute )
{
  uint8 index;

  if ((index = macPibIndex(pibAttribute)) == MAC_PIB_INVALID)
  {
    return 0;
  }

  return ( macPibTbl[index].len );
}

/**************************************************************************************************
 * @fn          MAC_MlmeSetReq
 *
 * @brief       This direct execute function sets an attribute value
 *              in the MAC PIB.
 *
 * input parameters
 *
 * @param       pibAttribute - The attribute identifier.
 * @param       pValue - pointer to the attribute value.
 *
 * output parameters
 *
 * None.
 *
 * @return      The status of the request, as follows:
 *              MAC_SUCCESS Operation successful.
 *              MAC_UNSUPPORTED_ATTRIBUTE Attribute not found.
 *
 **************************************************************************************************
 */
uint8 MAC_MlmeSetReq(uint8 pibAttribute, void *pValue)
{
  uint8         i;
  halIntState_t intState;

  if (pibAttribute == MAC_BEACON_PAYLOAD)
#if defined (FEATURE_BEACON_MODE) || defined (FEATURE_NON_BEACON_MODE)	  
  {
#if (defined ICALL_LITE) && (defined STACK_LIBRARY)
    /* Store the payload locally */
    osal_memcpy(macBeaconPayload, pValue, MAC_PIB_MAX_BEACON_PAYLOAD_LEN);
    pMacPib->pBeaconPayload = (uint8 *)&macBeaconPayload;
#else
    pMacPib->pBeaconPayload = pValue;
#endif /* ICALL_LITE && STACK_LIBRARY */
    return MAC_SUCCESS;
  }
#else
  {
    return MAC_UNSUPPORTED_ATTRIBUTE;  
  }
#endif

  /* look up attribute in PIB table */
  if ((i = macPibIndex(pibAttribute)) == MAC_PIB_INVALID)
  {
    return MAC_UNSUPPORTED_ATTRIBUTE;
  }

  /* do range check; no range check if min and max are zero */
  if ((macPibTbl[i].min != 0) || (macPibTbl[i].max != 0))
  {
    /* if min == max, this is a read-only attribute */
    if (macPibTbl[i].min == macPibTbl[i].max)
    {
      return MAC_READ_ONLY;
    }

    /* check for special cases */
    if (pibAttribute == MAC_MAX_FRAME_TOTAL_WAIT_TIME)
    {
      if ((*((uint16 *) pValue) < MAC_MAX_FRAME_RESPONSE_MIN) ||
          (*((uint16 *) pValue) > MAC_MAX_FRAME_RESPONSE_MAX))
      {
        return MAC_INVALID_PARAMETER;
      }
    }
    else if (pibAttribute == MAC_EBEACON_ORDER_NBPAN)
    {
      if ((*((uint16 *) pValue) < MAC_EBEACON_ORDER_NBPAN_MIN) ||
          (*((uint16 *) pValue) > MAC_EBEACON_ORDER_NBPAN_MAX))
      {
        return MAC_INVALID_PARAMETER;
      }
    }
    else if (pibAttribute == MAC_PHY_TRANSMIT_POWER_SIGNED)
    {
      if (macGetRadioTxPowerReg(*((int8*) pValue)) == MAC_RADIO_TX_POWER_INVALID)
      {
        return MAC_INVALID_PARAMETER;
      }
    }
    else if (pibAttribute == MAC_PHY_CURRENT_DESCRIPTOR_ID)
    {
#ifndef FREQ_2_4G
      /* Only if the PhyID changes, reconfigure radio per PhyID */
      if (pMacPib->curPhyID == *((uint8*) pValue))
#endif
      {
        return MAC_SUCCESS;
      }
    }
#ifdef FREQ_2_4G
    else if (pibAttribute == MAC_CHANNEL_PAGE)
    {
        return MAC_SUCCESS;
    }
#endif
    /* range check for general case */
    else if ((*((uint8 *) pValue) < macPibTbl[i].min) || (*((uint8 *) pValue) > macPibTbl[i].max))
    {
      return MAC_INVALID_PARAMETER;
    }

  }

  /* set value in PIB */
  HAL_ENTER_CRITICAL_SECTION(intState);
  osal_memcpy((uint8 *) pMacPib + macPibTbl[i].offset, pValue, macPibTbl[i].len);
  HAL_EXIT_CRITICAL_SECTION(intState);

  /* handle special cases */
  switch (pibAttribute)
  {
    case MAC_PAN_ID:
      /* set pan id in radio */
      macRadioSetPanID(pMacPib->panId);
      break;

    case MAC_SHORT_ADDRESS:
      /* set short address in radio */
      macRadioSetShortAddr(pMacPib->shortAddress);
      break;

    case MAC_RX_ON_WHEN_IDLE:
      /* turn rx on or off */
      if (pMacPib->rxOnWhenIdle)
      {
        macRxEnable(MAC_RX_WHEN_IDLE);
      }
      else
      {
        macRxDisable(MAC_RX_WHEN_IDLE);
      }
      break;

    case MAC_LOGICAL_CHANNEL:
      if (TRUE != macRadioSetChannel(pMacPib->logicalChannel))
      {
        return MAC_NO_RESOURCES;
      }
      break;

    case MAC_EXTENDED_ADDRESS:
      /* set ext address in radio */
      macRadioSetIEEEAddr(pMacPib->extendedAddress.addr.extAddr);
      break;

    case MAC_PHY_TRANSMIT_POWER_SIGNED:
      macRadioSetTxPower(pMacPib->phyTransmitPower);
      break;

    case MAC_PHY_CURRENT_DESCRIPTOR_ID:
      macSetDefaultsByPhyID();
      MAC_ResumeReq();
      break;

    case MAC_RANGE_EXTENDER:
      macSetDefaultsByRE();
      macRadioSetRE(pMacPib->rangeExt);
      if (pMacPib->rangeExt)
      {
        MAC_ResumeReq();
      }
      break;

    default:
      break;
  }

  return MAC_SUCCESS;
}

/**************************************************************************************************
 * @fn          MAC_GetPHYParamReq
 *
 * @brief       This direct execute function retrieves an attribute value
 *              from the PHY descriptor related entries
 *
 * @param       phyAttribute - The attribute identifier.
 * @param       phyID - phyID corresponding to the attribute
 * @param       pValue - pointer to the attribute value.
 *
 * @return      The status of the request, as follows:
 *              MAC_SUCCESS Operation successful.
 *              MAC_UNSUPPORTED_ATTRIBUTE Attribute not found.
 *
 **************************************************************************************************
 */
uint8 MAC_GetPHYParamReq(uint8 phyAttribute, uint8 phyID, void *pValue)
{
#ifndef FREQ_2_4G
  /* phyAttribute can only be MAC_PHY_DESCRIPTOR */
  macMRFSKPHYDesc_t *pMRFSKDesc = NULL;
#endif
  uint8 status = MAC_SUCCESS;

  MAC_PARAM_STATUS( pValue != NULL, status );

  if ( MAC_SUCCESS != status )
  {
    return status;
  }

#ifndef FREQ_2_4G
  switch (phyAttribute)
  {
    case MAC_PHY_DESCRIPTOR:

      pMRFSKDesc = macMRFSKGetPhyDesc(phyID);

      if ( pMRFSKDesc != NULL )
      {
        osal_memcpy(((macMRFSKPHYDesc_t *)pValue), pMRFSKDesc, sizeof(macMRFSKPHYDesc_t) );
      }
      else
      {
        return MAC_INVALID_PARAMETER;
      }

      break;

    default:
      return MAC_UNSUPPORTED_ATTRIBUTE;
  }
#endif
  return status;
}

/**************************************************************************************************
 * @fn          MAC_SetPHYParamReq
 *
 * @brief       This direct execute function sets an attribute value
 *              from the PHY descriptor related entries
 *              Note: this function will cause a radio restart.
 *
 * input parameters
 *
 * @param       phyAttribute - The attribute identifier.
*  @param       phyID corresponding to the attribute
 * @param       pValue - pointer to the attribute value.
 *
 * @return      The status of the request, as follows:
 *              MAC_SUCCESS Operation successful.
 *              MAC_UNSUPPORTED_ATTRIBUTE Attribute not found.
 *
 **************************************************************************************************
 */
uint8 MAC_SetPHYParamReq(uint8 phyAttribute, uint8 phyID, void *pValue)
{
  uint8 status = MAC_SUCCESS;
  MAC_PARAM_STATUS( pValue != NULL, status );

  if ( MAC_SUCCESS != status )
  {
    return status;
  }
#ifndef FREQ_2_4G
  /* phyAttribute can only be MAC_PHY_DESCRIPTOR */
  switch (phyAttribute)
  {
    case MAC_PHY_DESCRIPTOR:

       if ( ( phyID >= MAC_MRFSK_GENERIC_PHY_ID_BEGIN ) &&
            ( phyID <= MAC_MRFSK_GENERIC_PHY_ID_END) )
       {
         osal_memcpy(&macMRFSKGenPhyTable[phyID - MAC_MRFSK_GENERIC_PHY_ID_BEGIN] , pValue,
                     sizeof(macMRFSKPHYDesc_t));

         /* Reconfigure radio per phyID */
       }
       else
       {
         /* TBD if standard PHY descriptors can be overwritten and/or need to
          * to set valid flag for them */
         return MAC_INVALID_PARAMETER;
       }

       break;

    default:
      return MAC_UNSUPPORTED_ATTRIBUTE;
  }
#endif
  return status;
}
#ifndef FREQ_2_4G
/**************************************************************************************************
 * @fn          macMRFSKGetPhyDesc
 *
 * @brief       This function selects the indexed MR-FSK PHY descriptor
 *
 * input parameters
 *
 * @param       phyID - index of the PHY descriptor
 *
 * @return      pValue - pointer to the PHY descriptor
 *
 **************************************************************************************************
 */
MAC_INTERNAL_API macMRFSKPHYDesc_t* macMRFSKGetPhyDesc(uint8 phyID)
{
  macMRFSKPHYDesc_t *pMRFSKDesc = NULL;

  if (phyID >= MAC_MRFSK_STD_PHY_ID_BEGIN && phyID <= MAC_MRFSK_STD_PHY_ID_END)
  {
    pMRFSKDesc = (macMRFSKPHYDesc_t*) &macMRFSKStdPhyTable[phyID - MAC_MRFSK_STD_PHY_ID_BEGIN];
  }
  else if (phyID >= MAC_MRFSK_GENERIC_PHY_ID_BEGIN &&
           phyID <= MAC_MRFSK_GENERIC_PHY_ID_END)
  {
    pMRFSKDesc = (macMRFSKPHYDesc_t*) &macMRFSKGenPhyTable[phyID - MAC_MRFSK_GENERIC_PHY_ID_BEGIN];
  }
  return pMRFSKDesc;
}
#else
/**************************************************************************************************
 * @fn          macIEEEGetPhyDesc
 *
 * @brief       This function selects the IEEE PHY descriptor
 *
 * input parameters
 *
 * @param       phyID - index of the PHY descriptor
 *
 * @return      pValue - pointer to the PHY descriptor
 *
 **************************************************************************************************
 */
MAC_INTERNAL_API macIEEEPHYDesc_t* macIEEEGetPhyDesc(uint8 phyID)
{
  (void) phyID;

  return (macIEEEPHYDesc_t*)&macIEEEPhyTable;
}
#endif
