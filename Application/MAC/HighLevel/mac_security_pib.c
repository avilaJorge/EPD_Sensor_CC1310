/******************************************************************************

 @file  mac_security_pib.c

 @brief This module contains procedures for the Security-related MAC PIB.

 Group: WCS, LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2010-2017, Texas Instruments Incorporated
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

#ifdef FEATURE_MAC_SECURITY
/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_mcu.h"
#include "hal_board.h"
#include "mac_api.h"
#include "mac_spec.h"
#include "mac_low_level.h"
#include "mac_main.h"
#include "mac_security_pib.h"
#include "mac_pib.h"
#include "osal.h"
#include <stddef.h>

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* Attribute index constants, based on attribute ID values from spec */
#define MAC_ATTR_SECURITY_SET1_START       0x81
#define MAC_ATTR_SECURITY_SET1_END         0x8B
#define MAC_ATTR_SECURITY_SET1_OFFSET      0
#define MAC_ATTR_SECURITY_SET2_START       0xD0
#define MAC_ATTR_SECURITY_SET2_END         0xD5


/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* Security related PIB access and min/max table type */
typedef struct
{
  uint8     offset;
  uint8     len;
  uint8     min;
  uint8     max;
} macSecurityPibTbl_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

#ifdef STATIC_MAC_SECURITY_DEFAULTS
/* Security related PIB default values */
static CODE const macSecurityPib_t macSecurityPibDefaults =
{
  0,                                                 /* keyTableEntries */
  0,                                                 /* deviceTableEntries */
  0,                                                 /* securityLevelTableEntries */
  0x00,                                              /* autoRequestSecurityLevel */
  0,                                                 /* autoRequestKeyIdMode */
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* autoRequestKeySource */
  0xFF,                                              /* autoRequestKeyIndex */
  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  /* defaultKeySource */
  {0, SADDR_MODE_EXT},                               /* panCoordExtendedAddress */
  MAC_SHORT_ADDR_NONE,                               /* panCoordShortAddress */
  {                                                  /* macKeyTable */
    {NULL, 0, NULL, 0, NULL, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {NULL, 0, NULL, 0, NULL, 0, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}
  },
  {                                                  /* macKeyIdLookupList */
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x00}},
    {{{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x00}}
  },
  {                                                  /* macKeyDeviceList */
    {{0x00, false, false}, {0x00, false, false}, {0x00, false, false}  /* The rest, if any, will be initialized to zeros */},
    {{0x00, false, false}, {0x00, false, false}, {0x00, false, false}  /* The rest, if any, will be initialized to zeros */}
  },
  {
    {{MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME},{MAC_FRAME_TYPE_COMMAND, MAC_DATA_REQ_FRAME}}, /* macKeyUsageList */
    {{MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME},{MAC_FRAME_TYPE_COMMAND, MAC_DATA_REQ_FRAME}}
  },
  {                                                  /* macDeviceTable */
    {0x0000, 0xFFFF, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 0, 0, FALSE},
    {0x0000, 0xFFFF, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 0, 0, FALSE},
    {0x0000, 0xFFFF, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 0, 0, FALSE}
    /* The rest, if any, will be initialized to zeros */
  },
  {                                                   /* macSecurityLevelTable */
    {MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME, MAC_SEC_LEVEL_ENC_MIC_32, TRUE},
    {MAC_FRAME_TYPE_COMMAND, MAC_DATA_REQ_FRAME, MAC_SEC_LEVEL_ENC_MIC_32, TRUE}
  }
};
#endif

/* PIB access and min/max table.  min/max of 0/0 means not checked; if min/max are
 * equal, element is read-only
 */
static CODE const macSecurityPibTbl_t macSecurityPibTbl[] =
{
  {offsetof(macSecurityPib_t, keyTableEntries), sizeof(uint8), 0, MAX_KEY_TABLE_ENTRIES},         /* MAC_KEY_TABLE_ENTRIES */
  {offsetof(macSecurityPib_t, deviceTableEntries), sizeof(uint8), 0, MAX_DEVICE_TABLE_ENTRIES},   /* MAC_DEVICE_TABLE_ENTRIES */
  {offsetof(macSecurityPib_t, securityLevelTableEntries), sizeof(uint8), 0, MAX_SECURITY_LEVEL_TABLE_ENTRIES},
                                                                                                  /* MAC_SECURITY_LEVEL_TABLE_ENTRIES */
  {0, 0, 0, 0},                                                                                   /* MAC_FRAME_COUNTER */
  {offsetof(macSecurityPib_t, autoRequestSecurityLevel), sizeof(uint8), 0, 7},                    /* MAC_AUTO_REQUEST_SECURITY_LEVEL */
  {offsetof(macSecurityPib_t, autoRequestKeyIdMode), sizeof(uint8), 0, 3},                        /* MAC_AUTO_REQUEST_KEY_ID_MODE */
  {offsetof(macSecurityPib_t, autoRequestKeySource), MAC_KEY_SOURCE_MAX_LEN*sizeof(uint8), 0, 0}, /* MAC_AUTO_REQUEST_KEY_SOURCE*/
  {offsetof(macSecurityPib_t, autoRequestKeyIndex), sizeof(uint8), 0x01, 0xFF},                   /* MAC_AUTO_REQUEST_KEY_INDEX */
  {offsetof(macSecurityPib_t, defaultKeySource), MAC_KEY_SOURCE_MAX_LEN*sizeof(uint8), 0, 0},     /* MAC_DEFAULT_KEY_SOURCE */
  {offsetof(macSecurityPib_t, panCoordExtendedAddress), sizeof(sAddrExt_t), 0, 0},                /* MAC_PAN_COORD_EXTENDED_ADDRESS */
  {offsetof(macSecurityPib_t, panCoordShortAddress), sizeof(uint16), 0, 0},                       /* MAC_PAN_COORD_SHORT_ADDRESS */
};

/* Invalid security PIB table index used for error code */
#define MAC_SECURITY_PIB_INVALID     ((uint8) (sizeof(macSecurityPibTbl) / sizeof(macSecurityPibTbl[0])))

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* Security related MAC PIB */
macSecurityPib_t macSecurityPib;

uint8 MAC_MlmeGetSecurityReqSize( uint8 pibAttribute );

#if defined( FEATURE_MAC_PIB_PTR )

/* Pointer to the mac security PIB */
macSecurityPib_t* pMacSecurityPib = &macSecurityPib;

/**************************************************************************************************
 * @fn          MAC_MlmeSetActiveSecurityPib
 *
 * @brief       This direct execute function sets the active MAC security PIB.
 *
 * input parameters
 *
 * @param       pPib - pointer to the PIB structure.
 *
 * output parameters
 *
 * @return      None.
 *
 **************************************************************************************************
 */
void MAC_MlmeSetActiveSecurityPib( void* pSecPib)
{
  pMacSecurityPib = (macSecurityPib_t *)pSecPib;
}
#endif /* FEATURE_MAC_PIB_PTR */

/**************************************************************************************************
 * @fn          macSecurityPibReset
 *
 * @brief       This function initializes the security related PIB.
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
MAC_INTERNAL_API void macSecurityPibReset(void)
{
  /* copy security related PIB defaults */
#if defined( FEATURE_MAC_PIB_PTR )
  *pMacSecurityPib = macSecurityPibDefaults;
#else

#ifdef STATIC_MAC_SECURITY_DEFAULTS
  macSecurityPib = macSecurityPibDefaults;

  /* Fix up arrays that can change size */

  /* Fix up for MAX_DEVICE_TABLE_ENTRIES */
  {
      uint16 x;
      for ( x = 0; x < MAX_KEY_TABLE_ENTRIES; x++ )
      {
          uint16 y;
          for ( y = 0; y < MAX_DEVICE_TABLE_ENTRIES; y++ )
          {
              macSecurityPib.macKeyDeviceList[x][y].deviceDescriptorHandle = 0;
              macSecurityPib.macKeyDeviceList[x][y].uniqueDevice = false;
              macSecurityPib.macKeyDeviceList[x][y].blackListed = false;
          }
      }

      for ( x = 0; x < MAX_DEVICE_TABLE_ENTRIES; x++ )
      {
          osal_memcpy( &macSecurityPib.macDeviceTable[x],
                       &macSecurityPibDefaults.macDeviceTable[0],
                       sizeof(deviceDescriptor_t));
      }
  }
#else
  /* init the macPib to zero */
  osal_memset(&macSecurityPib, 0x00, sizeof(macSecurityPib));

  {
      uint16_t x;

      /* auto request key source
       * {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  autoRequestKeySource
       */
      osal_memset(&macSecurityPib.autoRequestKeySource[0], 0xFF,
                  sizeof(macSecurityPib.autoRequestKeySource));
      macSecurityPib.autoRequestKeyIndex = 0xFF;
      osal_memset(&macSecurityPib.defaultKeySource[0], 0xFF,
                  sizeof(macSecurityPib.defaultKeySource));

      macSecurityPib.panCoordExtendedAddress.addrMode = SADDR_MODE_EXT;
      macSecurityPib.panCoordShortAddress = MAC_SHORT_ADDR_NONE;

      /* macKeyusageList */
      for (x = 0; x < MAX_KEY_TABLE_ENTRIES; x++)
      {
          macSecurityPib.macKeyUsageList[x][0].frameType = MAC_FRAME_TYPE_DATA;
          macSecurityPib.macKeyUsageList[x][0].cmdFrameId = MAC_DATA_REQ_FRAME;

          macSecurityPib.macKeyUsageList[x][1].frameType = MAC_FRAME_TYPE_COMMAND;
          macSecurityPib.macKeyUsageList[x][1].cmdFrameId = MAC_DATA_REQ_FRAME;
      }

      /* key secuirty level table */
      for (x =0 ; x < MAX_SECURITY_LEVEL_TABLE_ENTRIES; x++)
      {
          macSecurityPib.macSecurityLevelTable[x].frameType = MAC_FRAME_TYPE_DATA;
          macSecurityPib.macSecurityLevelTable[x].commandFrameIdentifier = MAC_DATA_REQ_FRAME;
          macSecurityPib.macSecurityLevelTable[x].securityMinimum = MAC_SEC_LEVEL_ENC_MIC_32;
          macSecurityPib.macSecurityLevelTable[x].securityOverrideSecurityMinimum = TRUE;
      }

      macSecurityPib.macSecurityLevelTable[1].frameType = MAC_FRAME_TYPE_COMMAND;

      /* device short address to 0xFFFF */
      for (x = 0; x < MAX_DEVICE_TABLE_ENTRIES; x++)
      {
          macSecurityPib.macDeviceTable[x].shortAddress = 0xFFFF;
      }
  }
#endif

#endif /* FEATURE_MAC_PIB_PTR */

  pMacSecurityPib->securityLevelTableEntries = MAX_SECURITY_LEVEL_TABLE_ENTRIES;
}

/**************************************************************************************************
 * @fn          macSecurityPibIndex
 *
 * @brief       This function takes an security related PIB attribute and returns the index in to
 *              macSecurityPibTbl for the attribute.
 *
 * input parameters
 *
 * @param       pibAttribute - Security related PIB attribute to look up.
 *
 * output parameters
 *
 * None.
 *
 * @return      Index in to macSecurityPibTbl for the attribute or MAC_SECURITY_PIB_INVALID.
 **************************************************************************************************
 */
MAC_INTERNAL_API uint8 macSecurityPibIndex(uint8 pibAttribute)
{
  if ((pibAttribute >= MAC_ATTR_SECURITY_SET1_START) && (pibAttribute <= MAC_ATTR_SECURITY_SET1_END))
  {
    return (pibAttribute - MAC_ATTR_SECURITY_SET1_START + MAC_ATTR_SECURITY_SET1_OFFSET);
  }
  else
  {
    return MAC_SECURITY_PIB_INVALID;
  }
}


/**************************************************************************************************
 * @fn          MAC_MlmeGetSecurityReq
 *
 * @brief       This direct execute function retrieves an attribute value
 *              from the MAC security related PIB.
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
uint8 MAC_MlmeGetSecurityReq(uint8 pibAttribute, void *pValue)
{
  uint8         i, keyIndex, entry;
  halIntState_t intState;

  /* Special handling for proprietary Security PIB Get and Set Attributes */
  switch (pibAttribute)
  {
    case MAC_KEY_TABLE:
    case MAC_DEVICE_TABLE:
    case MAC_SECURITY_LEVEL_TABLE:
      /* It does not make sense to return a target pointer. */
      return MAC_INVALID_PARAMETER;

    case MAC_FRAME_COUNTER:
      return MAC_UNSUPPORTED_ATTRIBUTE;

    case MAC_KEY_ID_LOOKUP_ENTRY:
      keyIndex = ((macSecurityPibKeyIdLookupEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyIdLookupEntry_t *)pValue)->key_id_lookup_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= pMacSecurityPib->macKeyTable[keyIndex].keyIdLookupEntries)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&((macSecurityPibKeyIdLookupEntry_t *)pValue)->macKeyIdLookupEntry, &pMacSecurityPib->macKeyIdLookupList[keyIndex][entry], sizeof(keyIdLookupDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_DEVICE_ENTRY:
      keyIndex = ((macSecurityPibKeyDeviceEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyDeviceEntry_t *)pValue)->key_device_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= pMacSecurityPib->macKeyTable[keyIndex].keyDeviceListEntries)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&((macSecurityPibKeyDeviceEntry_t *)pValue)->macKeyDeviceEntry, &pMacSecurityPib->macKeyDeviceList[keyIndex][entry], sizeof(keyDeviceDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_USAGE_ENTRY:
      keyIndex = ((macSecurityPibKeyUsageEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyUsageEntry_t *)pValue)->key_key_usage_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= pMacSecurityPib->macKeyTable[keyIndex].keyUsageListEntries)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&((macSecurityPibKeyUsageEntry_t *)pValue)->macKeyUsageEntry, &pMacSecurityPib->macKeyUsageList[keyIndex][entry], sizeof(keyUsageDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_ENTRY:
      keyIndex = ((macSecurityPibKeyEntry_t *)pValue)->key_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(((macSecurityPibKeyEntry_t *)pValue)->keyEntry, pMacSecurityPib->macKeyTable[keyIndex].key, MAC_KEY_MAX_LEN);
      ((macSecurityPibKeyEntry_t *)pValue)->frameCounter = pMacSecurityPib->macKeyTable[keyIndex].frameCounter;
      return MAC_SUCCESS;

    case MAC_DEVICE_ENTRY:
      entry = ((macSecurityPibDeviceEntry_t *)pValue)->device_index;
      if (entry >= MAX_DEVICE_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&((macSecurityPibDeviceEntry_t *)pValue)->macDeviceEntry, &pMacSecurityPib->macDeviceTable[entry], sizeof(deviceDescriptor_t));
      return MAC_SUCCESS;

    case MAC_SECURITY_LEVEL_ENTRY:
      entry = ((macSecurityPibSecurityLevelEntry_t *)pValue)->security_level_index;
      if (entry >= MAX_SECURITY_LEVEL_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&((macSecurityPibSecurityLevelEntry_t *)pValue)->macSecurityLevelEntry, &pMacSecurityPib->macSecurityLevelTable[entry], sizeof(securityLevelDescriptor_t));
      return MAC_SUCCESS;
  }

  if ((i = macSecurityPibIndex(pibAttribute)) == MAC_SECURITY_PIB_INVALID)
  {
    return MAC_UNSUPPORTED_ATTRIBUTE;
  }

  HAL_ENTER_CRITICAL_SECTION(intState);
  osal_memcpy(pValue, (uint8 *) pMacSecurityPib + macSecurityPibTbl[i].offset, macSecurityPibTbl[i].len);
  HAL_EXIT_CRITICAL_SECTION(intState);
  return MAC_SUCCESS;
}


/**************************************************************************************************
 * @fn          MAC_MlmeGetPointerSecurityReq
 *
 * @brief       This direct execute function gets the MAC security PIB attribute pointers
 *
 * input parameters
 *
 * @param       pibAttribute - The attribute identifier.
 *
 * output parameters
 *
 * @param       pValue will contain a pointer to the attribute in macPib
 *
 * @return      MAC_SUCCESS
 *
 **************************************************************************************************
 */
MAC_INTERNAL_API uint8 MAC_MlmeGetPointerSecurityReq(uint8 pibAttribute, void **pValue)
{
  switch (pibAttribute)
  {
  case MAC_KEY_TABLE:
    *pValue = pMacSecurityPib->macKeyTable;
    return MAC_SUCCESS;
  case MAC_DEVICE_TABLE:
    *pValue =  pMacSecurityPib->macDeviceTable;
    return MAC_SUCCESS;
  case MAC_SECURITY_LEVEL_TABLE:
    *pValue = pMacSecurityPib->macSecurityLevelTable;
    return MAC_SUCCESS;
  default:
    return MAC_INVALID_PARAMETER;
  }
}


/**************************************************************************************************
 * @fn          MAC_MlmeGetSecurityReqSize
 *
 * @brief       This direct execute function gets the MAC security PIB attribute size
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
uint8 MAC_MlmeGetSecurityReqSize( uint8 pibAttribute )
{
  uint8 len = 0;

  /* Special handling for proprietary Security PIB Get and Set Attributes */
  switch (pibAttribute)
  {
    case MAC_KEY_ID_LOOKUP_ENTRY:
      len = (uint8)(sizeof(macSecurityPibKeyIdLookupEntry_t));
      break;

    case MAC_KEY_DEVICE_ENTRY:
      len = (uint8)(sizeof(macSecurityPibKeyDeviceEntry_t));
      break;

    case MAC_KEY_USAGE_ENTRY:
      len = (uint8)(sizeof(macSecurityPibKeyUsageEntry_t));
      break;

    case MAC_KEY_ENTRY:
      len = (uint8)(sizeof(macSecurityPibKeyEntry_t));
      break;

    case MAC_DEVICE_ENTRY:
      len = (uint8)(sizeof(macSecurityPibDeviceEntry_t));
      break;

    case MAC_SECURITY_LEVEL_ENTRY:
      len = (uint8)(sizeof(macSecurityPibSecurityLevelEntry_t));
      break;

    default:
      {
        uint8 i;
        if ((i = macSecurityPibIndex(pibAttribute)) != MAC_SECURITY_PIB_INVALID)
        {
          len = macSecurityPibTbl[i].len;
        }
      }
      break;
  }

  return ( len );
}


/**************************************************************************************************
 * @fn          MAC_MlmeSetSecurityReq
 *
 * @brief       This direct execute function sets an attribute value
 *              in the MAC security related PIB.
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
uint8 MAC_MlmeSetSecurityReq(uint8 pibAttribute, void *pValue)
{
  uint8         i, keyIndex, entry;
  halIntState_t intState;

  /* Special handling for proprietary Security PIB Get and Set Attributes */
  switch (pibAttribute)
  {
    case MAC_FRAME_COUNTER:
      return MAC_UNSUPPORTED_ATTRIBUTE;

    case MAC_KEY_TABLE:
      if (pValue == NULL)
      {
        /* If the pValue is null, the PIB entries are already created by the
         * proprietary security PIB set requests. This call simply builds the table.
         */
        for(keyIndex = 0; keyIndex < MAX_KEY_TABLE_ENTRIES; keyIndex++)
        {
          /* Build the key table by assigning the corresponding pointers. Note that the number
           * of entries are filled when the individual entry is received. This also assume the list
           * must be received in sequential order from small to large.
           */
          pMacSecurityPib->macKeyTable[keyIndex].keyIdLookupList = pMacSecurityPib->macKeyIdLookupList[keyIndex];
          pMacSecurityPib->macKeyTable[keyIndex].keyDeviceList   = pMacSecurityPib->macKeyDeviceList[keyIndex];
          pMacSecurityPib->macKeyTable[keyIndex].keyUsageList    = pMacSecurityPib->macKeyUsageList[keyIndex];
        }
      }
      else
      {
        osal_memcpy(&pMacSecurityPib->macKeyTable, pValue, sizeof(pMacSecurityPib->macKeyTable));
      }

      return MAC_SUCCESS;

    case MAC_KEY_ID_LOOKUP_ENTRY:
      keyIndex = ((macSecurityPibKeyIdLookupEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyIdLookupEntry_t *)pValue)->key_id_lookup_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= MAX_KEY_ID_LOOKUP_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      pMacSecurityPib->macKeyTable[keyIndex].keyIdLookupEntries = entry+1;
      osal_memcpy(&pMacSecurityPib->macKeyIdLookupList[keyIndex][entry], &((macSecurityPibKeyIdLookupEntry_t *)pValue)->macKeyIdLookupEntry, sizeof(keyIdLookupDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_DEVICE_ENTRY:
      keyIndex = ((macSecurityPibKeyDeviceEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyDeviceEntry_t *)pValue)->key_device_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= MAX_KEY_DEVICE_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      /* Key device entry may be overwritten individually and hence table size must not be
       * determined based on last entry written.
       * However the following variable is still used just to ensure that only initialized entries
       * are accessed. */
      if (pMacSecurityPib->macKeyTable[keyIndex].keyDeviceListEntries <= entry)
      {
        pMacSecurityPib->macKeyTable[keyIndex].keyDeviceListEntries = entry+1;
      }
      osal_memcpy(&pMacSecurityPib->macKeyDeviceList[keyIndex][entry], &((macSecurityPibKeyDeviceEntry_t *)pValue)->macKeyDeviceEntry, sizeof(keyDeviceDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_USAGE_ENTRY:
      keyIndex = ((macSecurityPibKeyUsageEntry_t *)pValue)->key_index;
      entry    = ((macSecurityPibKeyUsageEntry_t *)pValue)->key_key_usage_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES || entry >= MAX_KEY_USAGE_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      pMacSecurityPib->macKeyTable[keyIndex].keyUsageListEntries = entry+1;
      osal_memcpy(&pMacSecurityPib->macKeyUsageList[keyIndex][entry], &((macSecurityPibKeyUsageEntry_t *)pValue)->macKeyUsageEntry, sizeof(keyUsageDescriptor_t));
      return MAC_SUCCESS;

    case MAC_KEY_ENTRY:
      keyIndex = ((macSecurityPibKeyEntry_t *)pValue)->key_index;
      if (keyIndex >= MAX_KEY_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(pMacSecurityPib->macKeyTable[keyIndex].key, ((macSecurityPibKeyEntry_t *)pValue)->keyEntry, MAC_KEY_MAX_LEN);
      pMacSecurityPib->macKeyTable[keyIndex].frameCounter = ((macSecurityPibKeyEntry_t *)pValue)->frameCounter;
      return MAC_SUCCESS;

    case MAC_DEVICE_TABLE:
      if (pValue != NULL)
      {
        /* If the pValue is null, the PIB entries are already created by the
         * proprietary security PIB set requests. This call simply builds the table.
         */
        osal_memcpy(&pMacSecurityPib->macDeviceTable, pValue, sizeof(pMacSecurityPib->macDeviceTable));
      }
      return MAC_SUCCESS;

    case MAC_SECURITY_LEVEL_TABLE:
      if (pValue != NULL)
      {
        /* If the pValue is null, the PIB entries are already created by the
         * proprietary security PIB set requests. This call simply builds the table.
         */
        osal_memcpy(&pMacSecurityPib->macSecurityLevelTable, pValue, sizeof(pMacSecurityPib->macSecurityLevelTable));
      }
      return MAC_SUCCESS;

    case MAC_DEVICE_ENTRY:
      entry = ((macSecurityPibDeviceEntry_t *)pValue)->device_index;
      if (entry >= MAX_DEVICE_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&pMacSecurityPib->macDeviceTable[entry], &((macSecurityPibDeviceEntry_t *)pValue)->macDeviceEntry, sizeof(deviceDescriptor_t));
      return MAC_SUCCESS;

    case MAC_SECURITY_LEVEL_ENTRY:
      entry = ((macSecurityPibSecurityLevelEntry_t *)pValue)->security_level_index;
      if (entry >= MAX_SECURITY_LEVEL_TABLE_ENTRIES)
      {
        return MAC_INVALID_PARAMETER;
      }
      osal_memcpy(&pMacSecurityPib->macSecurityLevelTable[entry], &((macSecurityPibSecurityLevelEntry_t *)pValue)->macSecurityLevelEntry, sizeof(securityLevelDescriptor_t));
      return MAC_SUCCESS;

  }

  /* look up attribute in security related PIB table */
  if ((i = macSecurityPibIndex(pibAttribute)) == MAC_SECURITY_PIB_INVALID)
  {
    return MAC_UNSUPPORTED_ATTRIBUTE;
  }

  /* do range check; no range check if min and max are zero */
  if ((macSecurityPibTbl[i].min != 0) || (macSecurityPibTbl[i].max != 0))
  {
    /* if min == max, this is a read-only attribute */
    if (macSecurityPibTbl[i].min == macSecurityPibTbl[i].max)
    {
      return MAC_READ_ONLY;
    }

    /* range check for general case */
    if ((*((uint8 *) pValue) < macSecurityPibTbl[i].min) || (*((uint8 *) pValue) > macSecurityPibTbl[i].max))
    {
      return MAC_INVALID_PARAMETER;
    }
  }

  /* set value in security related PIB */
  HAL_ENTER_CRITICAL_SECTION(intState);
  osal_memcpy((uint8 *) pMacSecurityPib + macSecurityPibTbl[i].offset, pValue, macSecurityPibTbl[i].len);
  HAL_EXIT_CRITICAL_SECTION(intState);

  return MAC_SUCCESS;
}

#endif
