/* USER CODE BEGIN Header */
/***************************************************************************
**                                                                        **
**  FlySight 2 firmware                                                   **
**  Copyright 2023 Bionic Avionics Inc.                                   **
**                                                                        **
**  This program is free software: you can redistribute it and/or modify  **
**  it under the terms of the GNU General Public License as published by  **
**  the Free Software Foundation, either version 3 of the License, or     **
**  (at your option) any later version.                                   **
**                                                                        **
**  This program is distributed in the hope that it will be useful,       **
**  but WITHOUT ANY WARRANTY; without even the implied warranty of        **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         **
**  GNU General Public License for more details.                          **
**                                                                        **
**  You should have received a copy of the GNU General Public License     **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>. **
**                                                                        **
****************************************************************************
**  Contact: Bionic Avionics Inc.                                         **
**  Website: http://flysight.ca/                                          **
****************************************************************************/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */
#include <string.h>

#include "common.h"
#include "stm32_adafruit_sd.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
void (*beginActivityCallback)(void) = 0;
void (*endActivityCallback)(void) = 0;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
typedef enum
{
  USB_STORAGE_CACHE_INVALID = 0,
  USB_STORAGE_CACHE_READ,
  USB_STORAGE_CACHE_WRITE
} USB_StorageCacheMode_t;

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  0x10000
#define STORAGE_BLK_SIZ                  0x200

/* USER CODE BEGIN PRIVATE_DEFINES */
#define STORAGE_CACHE_BLK_NBR          (FS_SHARED_BUFFER_SIZE / STORAGE_BLK_SIZ)
#define STORAGE_WAIT_TIMEOUT           100000U

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */
static USB_StorageCacheMode_t storageCacheMode = USB_STORAGE_CACHE_INVALID;
static uint32_t storageCacheBaseBlock = 0;
static uint16_t storageCacheBlockCount = 0;
static uint64_t storageCacheDirtyMask = 0;
static uint32_t storageCacheTotalBlocks = 0;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static uint64_t STORAGE_CacheBlockMask(uint16_t block_count);
static void STORAGE_ResetCache(void);
static int8_t STORAGE_EnsureCapacity(void);
static int8_t STORAGE_WaitCardReady(void);
static int8_t STORAGE_ReadBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_WriteBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_FlushCache(void);
static int8_t STORAGE_LoadReadCache(uint32_t blk_addr);
static int8_t STORAGE_EnsureWriteCache(uint32_t blk_addr);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  if (BSP_SD_Init() != BSP_SD_OK)
  {
    return (USBD_FAIL);
  }
  if (STORAGE_FlushCache() < 0)
  {
    return (USBD_FAIL);
  }
  STORAGE_ResetCache();
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  SD_CardInfo info;
  int8_t ret = 0;

  if (BSP_SD_GetCardInfo(&info) != BSP_SD_OK)
  {
	ret = -1;
  }
  else
  {
    storageCacheTotalBlocks = info.LogBlockNbr;
    *block_num = info.LogBlockNbr;
    *block_size = info.LogBlockSize;
  }

  return ret;
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  static int8_t prev_status = 0;
  int8_t ret = -1;

  if(prev_status < 0)
  {
	BSP_SD_Init();
	prev_status = 0;
  }
  if(BSP_SD_GetCardState() == BSP_SD_OK)
  {
	ret = 0;
  }

  return ret;
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
  uint16_t blocksLeft = blk_len;

  UNUSED(lun);

  if (STORAGE_EnsureCapacity() < 0)
  {
    return -1;
  }

  while (blocksLeft > 0U)
  {
    uint32_t offset;
    uint16_t chunkBlocks;

    if (STORAGE_LoadReadCache(blk_addr) < 0)
    {
      return -1;
    }

    offset = blk_addr - storageCacheBaseBlock;
    if (offset >= storageCacheBlockCount)
    {
      return -1;
    }

    chunkBlocks = (uint16_t) (storageCacheBlockCount - offset);
    if (chunkBlocks > blocksLeft)
    {
      chunkBlocks = blocksLeft;
    }

    memcpy(buf, FS_Common_GetSharedBuffer() + (offset * STORAGE_BLK_SIZ),
           chunkBlocks * STORAGE_BLK_SIZ);

    buf += chunkBlocks * STORAGE_BLK_SIZ;
    blk_addr += chunkBlocks;
    blocksLeft -= chunkBlocks;
  }

  return 0;
  /* USER CODE END 6 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
  uint16_t blocksLeft = blk_len;

  UNUSED(lun);

  if (STORAGE_EnsureCapacity() < 0)
  {
    return -1;
  }

  while (blocksLeft > 0U)
  {
    uint32_t offset;
    uint16_t chunkBlocks;

    if (STORAGE_EnsureWriteCache(blk_addr) < 0)
    {
      return -1;
    }

    offset = blk_addr - storageCacheBaseBlock;
    if (offset >= storageCacheBlockCount)
    {
      return -1;
    }

    chunkBlocks = (uint16_t) (storageCacheBlockCount - offset);
    if (chunkBlocks > blocksLeft)
    {
      chunkBlocks = blocksLeft;
    }

    memcpy(FS_Common_GetSharedBuffer() + (offset * STORAGE_BLK_SIZ), buf,
           chunkBlocks * STORAGE_BLK_SIZ);

    for (uint16_t i = 0; i < chunkBlocks; ++i)
    {
      storageCacheDirtyMask |= 1ULL << (offset + i);
    }

    if (storageCacheDirtyMask == STORAGE_CacheBlockMask(storageCacheBlockCount))
    {
      if (STORAGE_FlushCache() < 0)
      {
        return -1;
      }
    }

    buf += chunkBlocks * STORAGE_BLK_SIZ;
    blk_addr += chunkBlocks;
    blocksLeft -= chunkBlocks;
  }

  return 0;
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
static uint64_t STORAGE_CacheBlockMask(uint16_t block_count)
{
  if (block_count >= STORAGE_CACHE_BLK_NBR)
  {
    return UINT64_MAX;
  }

  return (1ULL << block_count) - 1ULL;
}

static uint32_t STORAGE_CacheBaseBlock(uint32_t blk_addr)
{
  return blk_addr - (blk_addr % STORAGE_CACHE_BLK_NBR);
}

static uint16_t STORAGE_CacheBlockCount(uint32_t baseBlock)
{
  uint32_t remainingBlocks;

  if (baseBlock >= storageCacheTotalBlocks)
  {
    return 0;
  }

  remainingBlocks = storageCacheTotalBlocks - baseBlock;

  if (remainingBlocks > STORAGE_CACHE_BLK_NBR)
  {
    remainingBlocks = STORAGE_CACHE_BLK_NBR;
  }

  return (uint16_t) remainingBlocks;
}

static void STORAGE_ResetCache(void)
{
  storageCacheMode = USB_STORAGE_CACHE_INVALID;
  storageCacheBaseBlock = 0;
  storageCacheBlockCount = 0;
  storageCacheDirtyMask = 0;
  storageCacheTotalBlocks = 0;
}

static int8_t STORAGE_EnsureCapacity(void)
{
  SD_CardInfo info;

  if (storageCacheTotalBlocks != 0U)
  {
    return 0;
  }

  if (BSP_SD_GetCardInfo(&info) != BSP_SD_OK)
  {
    return -1;
  }

  storageCacheTotalBlocks = info.LogBlockNbr;

  return 0;
}

static int8_t STORAGE_WaitCardReady(void)
{
  uint32_t timeout = STORAGE_WAIT_TIMEOUT;

  while (BSP_SD_GetCardState() != BSP_SD_OK)
  {
    if (timeout-- == 0U)
    {
      return -1;
    }
  }

  return 0;
}

static int8_t STORAGE_ReadBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  uint8_t res;

  if (blk_len == 0U)
  {
    return 0;
  }

  if (beginActivityCallback) beginActivityCallback();
  res = BSP_SD_ReadBlocks((uint32_t *) buf, blk_addr, blk_len, SD_DATATIMEOUT);
  if (endActivityCallback) endActivityCallback();

  if (res != BSP_SD_OK)
  {
    return -1;
  }

  return STORAGE_WaitCardReady();
}

static int8_t STORAGE_WriteBlocks(uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  uint8_t res;

  if (blk_len == 0U)
  {
    return 0;
  }

  if (beginActivityCallback) beginActivityCallback();
  res = BSP_SD_WriteBlocks((uint32_t *) buf, blk_addr, blk_len, SD_DATATIMEOUT);
  if (endActivityCallback) endActivityCallback();

  if (res != BSP_SD_OK)
  {
    return -1;
  }

  return STORAGE_WaitCardReady();
}

static int8_t STORAGE_FlushCache(void)
{
  uint8_t *cache = FS_Common_GetSharedBuffer();
  uint16_t block = 0;

  if ((storageCacheMode != USB_STORAGE_CACHE_WRITE) || (storageCacheDirtyMask == 0U))
  {
    return 0;
  }

  while (block < storageCacheBlockCount)
  {
    uint16_t startBlock;
    uint16_t blockCount;

    while ((block < storageCacheBlockCount)
        && ((storageCacheDirtyMask & (1ULL << block)) == 0U))
    {
      ++block;
    }

    startBlock = block;

    while ((block < storageCacheBlockCount)
        && ((storageCacheDirtyMask & (1ULL << block)) != 0U))
    {
      ++block;
    }

    blockCount = block - startBlock;
    if (blockCount == 0U)
    {
      continue;
    }

    if (STORAGE_WriteBlocks(cache + (startBlock * STORAGE_BLK_SIZ),
                            storageCacheBaseBlock + startBlock,
                            blockCount) < 0)
    {
      return -1;
    }

    for (uint16_t i = 0; i < blockCount; ++i)
    {
      storageCacheDirtyMask &= ~(1ULL << (startBlock + i));
    }
  }

  return 0;
}

static int8_t STORAGE_LoadReadCache(uint32_t blk_addr)
{
  uint32_t baseBlock = STORAGE_CacheBaseBlock(blk_addr);

  if ((storageCacheMode == USB_STORAGE_CACHE_READ)
      && (storageCacheBaseBlock == baseBlock))
  {
    return 0;
  }

  if (STORAGE_FlushCache() < 0)
  {
    return -1;
  }

  storageCacheMode = USB_STORAGE_CACHE_INVALID;
  storageCacheBaseBlock = baseBlock;
  storageCacheBlockCount = STORAGE_CacheBlockCount(baseBlock);
  storageCacheDirtyMask = 0;

  if (storageCacheBlockCount == 0U)
  {
    return -1;
  }

  if (STORAGE_ReadBlocks(FS_Common_GetSharedBuffer(),
                         storageCacheBaseBlock,
                         storageCacheBlockCount) < 0)
  {
    STORAGE_ResetCache();
    return -1;
  }

  storageCacheMode = USB_STORAGE_CACHE_READ;

  return 0;
}

static int8_t STORAGE_EnsureWriteCache(uint32_t blk_addr)
{
  uint32_t baseBlock = STORAGE_CacheBaseBlock(blk_addr);

  if ((storageCacheMode == USB_STORAGE_CACHE_WRITE)
      && (storageCacheBaseBlock == baseBlock))
  {
    return 0;
  }

  if (STORAGE_FlushCache() < 0)
  {
    return -1;
  }

  storageCacheMode = USB_STORAGE_CACHE_WRITE;
  storageCacheBaseBlock = baseBlock;
  storageCacheBlockCount = STORAGE_CacheBlockCount(baseBlock);
  storageCacheDirtyMask = 0;

  if (storageCacheBlockCount == 0U)
  {
    STORAGE_ResetCache();
    return -1;
  }

  return 0;
}

int8_t USBD_FlushStorageCache(void)
{
  return STORAGE_FlushCache();
}

void USBD_SetActivityCallbacks(void (*begin)(void), void (*end)(void))
{
  beginActivityCallback = begin;
  endActivityCallback = end;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

