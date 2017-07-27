/******************************************************************************

 @file  macwrapper.c

 @brief MAC Wrapper function interface defintion used by MLE.

 Group: WCS, LPC
 Target Device: CC13xx

 ******************************************************************************
 
 Copyright (c) 2014-2017, Texas Instruments Incorporated
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
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "macwrapper.h"

#include "hal_mcu.h"
#include "mac_api.h"
#include "mac_security_pib.h"
#include "mac_spec.h"
#include "mac_mgmt.h"

#include "osal.h"

/**
 * Invalid key index to be used to mark that a specific key lookup entry is
 * not used.
 */
#define MACWRAPPER_INVALID_KEY_INDEX        0

#ifdef FEATURE_MAC_SECURITY

static int8 macWrapper8ByteUnused(const unsigned char *extaddr)
{
  int8 k;

  for (k = 0; k < 8; k++)
  {
    if (extaddr[k] != 0xff)
    {
      return 0;
    }
  }
  return 1;
}

/* See macwrapper.h for documentation */
unsigned char macWrapperAddDevice(ApiMac_secAddDevice_t *param)
{
  uint8 i;
  halIntState_t is;

  HAL_ENTER_CRITICAL_SECTION(is);
  MAC_MlmeGetSecurityReq(MAC_DEVICE_TABLE_ENTRIES, &i);
  if (i == 0)
  {
    /* In case device descriptor table is not initialized,
     * initialize the device descriptor table. */
    for (i = 0; i < MAX_DEVICE_TABLE_ENTRIES; i++)
    {
      macSecurityPibDeviceEntry_t deviceEntry;

      deviceEntry.device_index = i;
      deviceEntry.macDeviceEntry.panID = 0xffffu;
      deviceEntry.macDeviceEntry.shortAddress = 0xffffu;
      osal_memset(deviceEntry.macDeviceEntry.extAddress, 0xff, 8);
      deviceEntry.macDeviceEntry.exempt = FALSE;
      MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);
    }
    MAC_MlmeSetSecurityReq(MAC_DEVICE_TABLE_ENTRIES, &i);
  }

  /* Search for the matching extended address */
  for (i = 0; i < MAX_KEY_TABLE_ENTRIES; i++)
  {
    uint8 j, matchingKey = FALSE, unusedKey = TRUE;

    for (j = 0; j < MAX_KEY_ID_LOOKUP_ENTRIES; j++)
    {
      macSecurityPibKeyIdLookupEntry_t lookupEntry;
      uint8 lookupLen;

      lookupEntry.key_index = i;
      lookupEntry.key_id_lookup_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &lookupEntry) != MAC_SUCCESS)
      {
        continue;
      }
      if (lookupEntry.macKeyIdLookupEntry.lookupDataSize == 1 &&
          lookupEntry.macKeyIdLookupEntry.lookupData[8] == 0)
      {
        if (macWrapper8ByteUnused(lookupEntry.macKeyIdLookupEntry.lookupData))
        {
          /* This key lookup entry is invalid */
          unusedKey = TRUE;
          break;
        }
      }
      /* This key has at least one valid lookup entry */
      unusedKey = FALSE;
      if (param->keyIdLookupDataSize == 0)
      {
        /* Key Id Lookup Data length in bytes */
        lookupLen = 5;
      }
      else
      {
        lookupLen = 9;
      }
      if (lookupEntry.macKeyIdLookupEntry.lookupDataSize == param->keyIdLookupDataSize &&
          osal_memcmp(lookupEntry.macKeyIdLookupEntry.lookupData, param->keyIdLookupData,
                      lookupLen))
      {
        /* Key matches */
        matchingKey = TRUE;
        break;
      }
    }
    if (unusedKey)
    {
      continue;
    }
    if (!matchingKey && !param->duplicateDevFlag)
    {
      /* No need to create a device entry for this key */
      continue;
    }

    /* Now search for the key device table entries for the matching device */
    for (j = 0; j < MAX_KEY_DEVICE_TABLE_ENTRIES; j++)
    {
      macSecurityPibKeyDeviceEntry_t keyDeviceEntry;
      macSecurityPibDeviceEntry_t deviceEntry;
      keyDeviceEntry.key_index = i;
      keyDeviceEntry.key_device_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry) != MAC_SUCCESS ||
          keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
      {
        continue;
      }
      deviceEntry.device_index = keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle;
      if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry) != MAC_SUCCESS)
      {
        /* Security PIB is corrupt */
        HAL_EXIT_CRITICAL_SECTION(is);
        return MAC_BAD_STATE;
      }
      if (osal_memcmp(deviceEntry.macDeviceEntry.extAddress, param->extAddr, 8))
      {
        /* Matching device */
        /* Update the device descriptor */
        deviceEntry.macDeviceEntry.panID = param->panID;
        deviceEntry.macDeviceEntry.shortAddress = param->shortAddr;
        if (matchingKey)
        {
          deviceEntry.macDeviceEntry.exempt = param->exempt;
          deviceEntry.macDeviceEntry.frameCounter[i] = param->frameCounter;
        }
        MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);
        /* Update the key device descriptor */
        if (matchingKey && (keyDeviceEntry.macKeyDeviceEntry.uniqueDevice != param->uniqueDevice))
        {
          keyDeviceEntry.macKeyDeviceEntry.uniqueDevice = param->uniqueDevice;
          MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry);
        }
        break;
      }
    }

    if (j == MAX_KEY_DEVICE_TABLE_ENTRIES)
    {
      /* Matching device is not found. Add a new device descriptor
       * and key device descriptor */
      for (j = 0; j < MAX_DEVICE_TABLE_ENTRIES; j++)
      {
        macSecurityPibDeviceEntry_t deviceEntry;
        uint8 k;

        deviceEntry.device_index = j;
        if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry) != MAC_SUCCESS)
        {
          /* PIB is corrupt */
          HAL_EXIT_CRITICAL_SECTION(is);
          return MAC_BAD_STATE;
        }

        if (macWrapper8ByteUnused(deviceEntry.macDeviceEntry.extAddress) ||
            osal_memcmp(deviceEntry.macDeviceEntry.extAddress, param->extAddr, 8))
        {
          /* Empty slot found */
          deviceEntry.macDeviceEntry.panID = param->panID;
          deviceEntry.macDeviceEntry.shortAddress = param->shortAddr;
          deviceEntry.macDeviceEntry.exempt = param->exempt;
          if (matchingKey)
          {
            deviceEntry.macDeviceEntry.frameCounter[i] = param->frameCounter;
          }
          else
          {
            deviceEntry.macDeviceEntry.frameCounter[i] = 0;
          }
          osal_memcpy(deviceEntry.macDeviceEntry.extAddress, param->extAddr, 8);

#ifdef FEATURE_FREQ_HOP_MODE
          /* call back for FH add device */
          if ( (MAC_FH_ENABLED) && (MAC_FH_ADD_DEVICE_FN) )
          {
            MAC_FH_ADD_DEVICE_FN((sAddrExt_t *)param->extAddr, deviceEntry.device_index);
          }
#endif /* FEATURE_FREQ_HOP_MODE */

          /* Look for an empty slot in key device descriptor table */
          for (k = 0; k < MAX_KEY_DEVICE_TABLE_ENTRIES; k++)
          {
            macSecurityPibKeyDeviceEntry_t keyDeviceEntry;

            keyDeviceEntry.key_index = i;
            keyDeviceEntry.key_device_index = k;

            if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry) ==
                MAC_INVALID_PARAMETER ||
                keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
            {
              /* Empty slot found */
              keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle = deviceEntry.device_index;
              keyDeviceEntry.macKeyDeviceEntry.uniqueDevice = param->uniqueDevice;
              keyDeviceEntry.macKeyDeviceEntry.blackListed = FALSE;

              MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);
              MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry);
              break;
            }
          }
          if (k < MAX_KEY_DEVICE_TABLE_ENTRIES)
          {
            /* Empty slot was found */
            break;
          }
        }
      }
      if (j == MAX_DEVICE_TABLE_ENTRIES)
      {
        /* Empty slot was not found */
        HAL_EXIT_CRITICAL_SECTION(is);
        return MAC_NO_RESOURCES;
      }
    }
  }
  HAL_EXIT_CRITICAL_SECTION(is);
  return MAC_SUCCESS;
}

/* See macwrapper.h for documentation */
unsigned char macWrapperDeleteDevice(ApiMac_sAddrExt_t *param)
{
  uint8 i;
  halIntState_t is;

  HAL_ENTER_CRITICAL_SECTION(is);

  /* Search for the matching extended address */
  for (i = 0; i < MAX_KEY_TABLE_ENTRIES; i++)
  {
    uint8 j, unusedKey = TRUE;

    /* Check if the key is used */
    for (j = 0; j < MAX_KEY_ID_LOOKUP_ENTRIES; j++)
    {
      macSecurityPibKeyIdLookupEntry_t lookupEntry;

      lookupEntry.key_index = i;
      lookupEntry.key_id_lookup_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &lookupEntry) != MAC_SUCCESS)
      {
        /* If none of the key ID look up entries associated with a key are initialized,
         * the entry is considered as unused.
         * Hence the default value of unusedKey was set to TRUE above.
         */
        continue;
      }
      if (lookupEntry.macKeyIdLookupEntry.lookupDataSize == 1 &&
          lookupEntry.macKeyIdLookupEntry.lookupData[8] == 0)
      {
        uint8 k;

        for (k = 0; k < 8; k++)
        {
          if (lookupEntry.macKeyIdLookupEntry.lookupData[k] != 0xff)
          {
            break;
          }
        }
        if (k == 8)
        {
          /* This key lookup entry is marked to indicate unused key.
           * If any single lookup entry per key has this mark,
           * the key is considered as unused regardless of the state of
           * other key lookup entries associated with the same key.
           */
          unusedKey = TRUE;
          break;
        }
      }
      /* This key has at least one valid lookup entry,
       * which tentatively indicates that the key may be in use,
       * unless there is another lookup entry associated with the key
       * which has the mark of unused key.
       */
      unusedKey = FALSE;
    }
    if (unusedKey)
    {
      continue;
    }

    /* Now search for the key device table entries for the matching device */
    for (j = 0; j < MAX_KEY_DEVICE_TABLE_ENTRIES; j++)
    {
      macSecurityPibKeyDeviceEntry_t keyDeviceEntry;
      macSecurityPibDeviceEntry_t deviceEntry;

      keyDeviceEntry.key_index = i;
      keyDeviceEntry.key_device_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry) != MAC_SUCCESS ||
          keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
      {
        continue;
      }
      deviceEntry.device_index = keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle;
      if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry) != MAC_SUCCESS)
      {
        /* Security PIB is corrupt */
        HAL_EXIT_CRITICAL_SECTION(is);
        return MAC_BAD_STATE;
      }
      if (osal_memcmp(deviceEntry.macDeviceEntry.extAddress, param, 8))
      {
        /* Matching device */

        /* Update the device descriptor */
        deviceEntry.macDeviceEntry.panID = 0xffffu;
        deviceEntry.macDeviceEntry.shortAddress = 0xffffu;
        osal_memset(deviceEntry.macDeviceEntry.extAddress, 0xff, 8);
        MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);

        /* Update the key device descriptor */
        keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle = 0xff;
        MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry);
        break;
      }
      if (macWrapper8ByteUnused(deviceEntry.macDeviceEntry.extAddress))
      {
        /* Device entry is already cleared. It would happen for i > 0,
         * since the entry must have been cleared at i = 0, iteration. */
        keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle = 0xff;
        MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry);
      }
    }
  }
  HAL_EXIT_CRITICAL_SECTION(is);
  return MAC_SUCCESS;
}


/* See macwrapper.h for documentation */
unsigned char macWrapperDeleteKeyAndAssociatedDevices( uint8 keyIndex )
{
  uint8 j;
  halIntState_t is;

  HAL_ENTER_CRITICAL_SECTION(is);

  /* Set the key lookup list associated with this key to default value */
  for (j = 0; j < MAX_KEY_ID_LOOKUP_ENTRIES; j++)
  {
    macSecurityPibKeyIdLookupEntry_t lookupEntry;

    lookupEntry.key_index = keyIndex;
    lookupEntry.key_id_lookup_index = j;
    if ( MAC_MlmeGetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &lookupEntry) != MAC_SUCCESS )
    {
      continue;
    }

    if ( (lookupEntry.macKeyIdLookupEntry.lookupDataSize == 1 &&
          lookupEntry.macKeyIdLookupEntry.lookupData[MAC_KEY_LOOKUP_LONG_LEN - 1] !=
            MACWRAPPER_INVALID_KEY_INDEX)
         || (lookupEntry.macKeyIdLookupEntry.lookupDataSize == 0 &&
             lookupEntry.macKeyIdLookupEntry.lookupData[MAC_KEY_LOOKUP_SHORT_LEN - 1] !=
               MACWRAPPER_INVALID_KEY_INDEX) )
    {
      uint8 k;
      uint8 size = 4;
      if (lookupEntry.macKeyIdLookupEntry.lookupDataSize)
      {
        size = 8;
      }
      for( k = 0; k < size; k++ )
      {
        lookupEntry.macKeyIdLookupEntry.lookupData[k] = 0xff;
      }
      lookupEntry.macKeyIdLookupEntry.lookupData[k] = 0x00;
      MAC_MlmeSetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &lookupEntry);
    }
  }

  /* Search for the device table enteries associated with this key and delete (re-initialize) the entry */
  for (j = 0; j < MAX_KEY_DEVICE_TABLE_ENTRIES; j++)
  {
    macSecurityPibKeyDeviceEntry_t keyDeviceEntry;
    macSecurityPibDeviceEntry_t deviceEntry;

    keyDeviceEntry.key_index = keyIndex;
    keyDeviceEntry.key_device_index = j;
    if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry) != MAC_SUCCESS ||
        keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
    {
      continue;
    }
    deviceEntry.device_index = keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle;
    if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry) != MAC_SUCCESS)
    {
      /* Security PIB is corrupt */
      HAL_EXIT_CRITICAL_SECTION(is);
      return MAC_BAD_STATE;
    }

    /* Valid device */
    /* Update the device descriptor only when the other key
     * is not using the device. */
    {
      uint8 k;
      for (k = 0; k < MAX_KEY_DEVICE_TABLE_ENTRIES; k++)
      {
        keyDeviceEntry.key_index ^= 1;
        keyDeviceEntry.key_device_index = k;
        if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry)
            == MAC_SUCCESS &&
            keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle
            == deviceEntry.device_index)
        {
          break;
        }
      }
      if (k == MAX_KEY_DEVICE_TABLE_ENTRIES)
      {
        deviceEntry.macDeviceEntry.panID = 0xffffu;
        deviceEntry.macDeviceEntry.shortAddress = 0xffffu;
        osal_memset(deviceEntry.macDeviceEntry.extAddress, 0xff, 8);
        deviceEntry.macDeviceEntry.frameCounter[keyIndex] = 0;
        MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);
      }
    }

    /* No need to update Key Device Descriptor..it will be updated later */
  }
  HAL_EXIT_CRITICAL_SECTION(is);
  return MAC_SUCCESS;
} /* macWrapperDeleteKeyAndAssociatedDevices() */


/* See macwrapper.h for documentation */
unsigned char macWrapperDeleteAllDevices(void)
{
  uint8 i;
  halIntState_t is;

  HAL_ENTER_CRITICAL_SECTION(is);

  /* Search for the valid device descriptors */
  for (i = 0; i < MAX_KEY_TABLE_ENTRIES; i++)
  {
    uint8 j, unusedKey = TRUE;

    /* Check if the key is used */
    for (j = 0; j < MAX_KEY_ID_LOOKUP_ENTRIES; j++)
    {
      macSecurityPibKeyIdLookupEntry_t lookupEntry;

      lookupEntry.key_index = i;
      lookupEntry.key_id_lookup_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &lookupEntry) != MAC_SUCCESS)
      {
        /* If none of the key ID look up entries associated with a key are initialized,
         * the entry is considered as unused.
         * Hence the default value of unusedKey was set to TRUE above.
         */
        continue;
      }
      if (lookupEntry.macKeyIdLookupEntry.lookupDataSize == 1 &&
          lookupEntry.macKeyIdLookupEntry.lookupData[MAC_KEY_LOOKUP_LONG_LEN - 1] ==
            MACWRAPPER_INVALID_KEY_INDEX)
      {
        uint8 k;

        for (k = 0; k < 8; k++)
        {
          if (lookupEntry.macKeyIdLookupEntry.lookupData[k] != 0xff)
          {
            break;
          }
        }
        if (k == 8)
        {
          /* This key lookup entry is marked to indicate unused key.
           * If any single lookup entry per key has this mark,
           * the key is considered as unused regardless of the state of
           * other key lookup entries associated with the same key.
           */
          unusedKey = TRUE;
          break;
        }
      }
      /* This key has at least one valid lookup entry,
       * which tentatively indicates that the key may be in use,
       * unless there is another lookup entry associated with the key
       * which has the mark of unused key.
       */
      unusedKey = FALSE;
    }
    if (unusedKey)
    {
      continue;
    }

    /* Now search for the key device table entries for the matching device */
    for (j = 0; j < MAX_KEY_DEVICE_TABLE_ENTRIES; j++)
    {
      macSecurityPibKeyDeviceEntry_t keyDeviceEntry;
      macSecurityPibDeviceEntry_t deviceEntry;

      keyDeviceEntry.key_index = i;
      keyDeviceEntry.key_device_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry) != MAC_SUCCESS ||
          keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
      {
        continue;
      }
      deviceEntry.device_index = keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle;
      if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry) != MAC_SUCCESS)
      {
        /* Security PIB is corrupt */
        HAL_EXIT_CRITICAL_SECTION(is);
        return MAC_BAD_STATE;
      }
      /* Valid device */

      /* Update the device descriptor */
      deviceEntry.macDeviceEntry.panID = 0xffffu;
      deviceEntry.macDeviceEntry.shortAddress = 0xffffu;
      osal_memset(deviceEntry.macDeviceEntry.extAddress, 0xff, 8);
      MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);

      /* Update the key device descriptor */
      keyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle = 0xff;
      MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &keyDeviceEntry);
    }
  }
  HAL_EXIT_CRITICAL_SECTION(is);
  return MAC_SUCCESS;
}

/* See macwrapper.h for documentation */
unsigned char macWrapperGetDefaultSourceKey(unsigned char keyid,
                                            unsigned long *pFrameCounter)
{
  halIntState_t is;
  uint8 numKeys, lookupData[9], i, j;

  HAL_ENTER_CRITICAL_SECTION(is);

  MAC_MlmeGetSecurityReq(MAC_DEFAULT_KEY_SOURCE, lookupData);
  /* Note that default key source length is alway 8 octets */
  lookupData[8] = keyid;

  MAC_MlmeGetSecurityReq(MAC_KEY_TABLE_ENTRIES, &numKeys);
  for (i = 0; i < numKeys; i++)
  {
    for (j = 0; j < MAX_KEY_ID_LOOKUP_ENTRIES; j++)
    {
      macSecurityPibKeyIdLookupEntry_t buf;

      buf.key_index = i;
      buf.key_id_lookup_index = j;
      if (MAC_MlmeGetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &buf) == MAC_SUCCESS &&
          buf.macKeyIdLookupEntry.lookupDataSize == 1 &&
          osal_memcmp(buf.macKeyIdLookupEntry.lookupData, lookupData, 9) == TRUE)
      {
        macSecurityPibKeyEntry_t keyentry;
        uint8 result;

        keyentry.key_index = i;
        result = MAC_MlmeGetSecurityReq(MAC_KEY_ENTRY, &keyentry);
        HAL_EXIT_CRITICAL_SECTION(is);
        *pFrameCounter = keyentry.frameCounter;
        return result;
      }
    }
  }

  HAL_EXIT_CRITICAL_SECTION(is);
  return MAC_UNAVAILABLE_KEY;
}

/* See macwrapper.h for documentation */
unsigned char macWrapperAddKeyInitFCtr( ApiMac_secAddKeyInitFrameCounter_t *param )
{
  halIntState_t is;

  macSecurityPibKeyEntry_t mtKeyEntry;
  macSecurityPibKeyIdLookupEntry_t mtKeyIdLookupEntry;
  uint8 numKeys;
  uint8 i;
  unsigned char result;

  uint8 lookupList[MAC_MAX_KEY_LOOKUP_LEN+2];

  /* The byte that indicates number of lookup entries is skipped to
   * set the following pointer to the start of a lookup entry. */
  uint8* pLookUpData = &lookupList[1];

  lookupList[0] = 1;
  lookupList[1] = param->lookupDataSize;
  osal_memcpy( &lookupList[2], param->lookupData, MAC_MAX_KEY_LOOKUP_LEN);

  HAL_ENTER_CRITICAL_SECTION( is );

  MAC_MlmeGetSecurityReq(MAC_KEY_TABLE_ENTRIES, &numKeys);

  /* If the security table has not been updated previously, update it now.
   * It can have only two enteries one per key */
  if ( param->newKeyFlag )
  {
    if ( numKeys < 2 )
    {
      macSecurityPibKeyUsageEntry_t mtKeyUsageEntry = {0, 0, {MAC_FRAME_TYPE_DATA, MAC_DATA_REQ_FRAME}};
      numKeys++;

      /* Set the key usage for this key */
      mtKeyUsageEntry.key_index = param->replaceKeyIndex;
      mtKeyUsageEntry.key_key_usage_index = 0;
      MAC_MlmeSetSecurityReq(MAC_KEY_USAGE_ENTRY, &mtKeyUsageEntry);

      /* Set the number of keys */
      MAC_MlmeSetSecurityReq(MAC_KEY_TABLE_ENTRIES, &numKeys);
    }

    /* Need to duplicate the key Device Table Entries from the other key */
    for ( i = 0; i < MAX_KEY_DEVICE_TABLE_ENTRIES; i++)
    {
      macSecurityPibKeyDeviceEntry_t mtKeyDeviceEntry;
      macSecurityPibDeviceEntry_t deviceEntry;

      mtKeyDeviceEntry.key_index = param->replaceKeyIndex ^ 1;
      mtKeyDeviceEntry.key_device_index = i;

      if (MAC_MlmeGetSecurityReq(MAC_KEY_DEVICE_ENTRY, &mtKeyDeviceEntry) != MAC_SUCCESS ||
          mtKeyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle == 0xff)
      {
        mtKeyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle = 0xff;
      }
      else
      {
        /* Update the frame counter for the device descriptor */
        deviceEntry.device_index =
            mtKeyDeviceEntry.macKeyDeviceEntry.deviceDescriptorHandle;

        if (MAC_MlmeGetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry)
            != MAC_SUCCESS)
        {
          /* Security PIB is corrupt */
          HAL_EXIT_CRITICAL_SECTION(is);
          return MAC_BAD_STATE;
        }

        /* Valid device */
        /* Frame counter for the device descriptor for the replaceKeyIndex is
         * set to 0x00 to indicate a new key */
        deviceEntry.macDeviceEntry.frameCounter[param->replaceKeyIndex] = 0;

        /* Update the device descriptor */
        MAC_MlmeSetSecurityReq(MAC_DEVICE_ENTRY, &deviceEntry);
      }

      /* update the key device entry for this key and device */
      mtKeyDeviceEntry.key_index = param->replaceKeyIndex;
      mtKeyDeviceEntry.macKeyDeviceEntry.blackListed = FALSE;
      mtKeyDeviceEntry.macKeyDeviceEntry.uniqueDevice = FALSE;

      result =  MAC_MlmeSetSecurityReq(MAC_KEY_DEVICE_ENTRY, &mtKeyDeviceEntry);
    }
  }

  /* Compile flag to compile code piece that supports only one lookup
   * entry but minimize the code size in such a way. */
  /* Initialize the key Id look up data  */
  mtKeyIdLookupEntry.key_index = param->replaceKeyIndex;
  mtKeyIdLookupEntry.key_id_lookup_index = 0;

  osal_memcpy( mtKeyIdLookupEntry.macKeyIdLookupEntry.lookupData, &pLookUpData[1], pLookUpData[0] );
  mtKeyIdLookupEntry.macKeyIdLookupEntry.lookupDataSize = 0x01;

  result = MAC_MlmeSetSecurityReq(MAC_KEY_ID_LOOKUP_ENTRY, &mtKeyIdLookupEntry);
  if ( result != MAC_SUCCESS )
  {
    HAL_EXIT_CRITICAL_SECTION(is);
    return result;
  }
  /* Add the key entry */
  mtKeyEntry.key_index = param->replaceKeyIndex;
  osal_memcpy( mtKeyEntry.keyEntry, param->key, MAC_KEY_MAX_LEN );
  mtKeyEntry.frameCounter = param->frameCounter;

  result = MAC_MlmeSetSecurityReq(MAC_KEY_ENTRY, &mtKeyEntry);
  if ( result != MAC_SUCCESS )
  {
    HAL_EXIT_CRITICAL_SECTION(is);
    return result;
  }

  HAL_EXIT_CRITICAL_SECTION(is);
  return result;
}/* macWrapperAddKeyInitFCtr() */

#endif /* FEATURE_MAC_SECURITY */
