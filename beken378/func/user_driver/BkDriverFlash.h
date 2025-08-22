// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef __BEKENDRIVERFLASH_H__
#define __BEKENDRIVERFLASH_H__

#pragma once

#include "include.h"
#include "rtos_pub.h"
#include "flash_pub.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_FLASH _BK_ Flash Driver
* @brief  Flash operation Functions
* @{
*/
/******************************************************
 *                   Macros
 ******************************************************/
#define BK_PARTITION_USER_MAX ( 1 )
#define PAR_OPT_READ_POS      ( 0 )
#define PAR_OPT_WRITE_POS     ( 1 )

#define PAR_OPT_READ_MASK  	  ( 0x1u << PAR_OPT_READ_POS )
#define PAR_OPT_WRITE_MASK 	  ( 0x1u << PAR_OPT_WRITE_POS )

#define PAR_OPT_READ_DIS      ( 0x0u << PAR_OPT_READ_POS )
#define PAR_OPT_READ_EN       ( 0x1u << PAR_OPT_READ_POS )
#define PAR_OPT_WRITE_DIS     ( 0x0u << PAR_OPT_WRITE_POS )
#define PAR_OPT_WRITE_EN      ( 0x1u << PAR_OPT_WRITE_POS )

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    BK_PARTITION_BOOTLOADER = 0,
    BK_PARTITION_APPLICATION,
    BK_PARTITION_OTA,
    BK_PARTITION_RF_FIRMWARE,
    BK_PARTITION_NET_PARAM,
    BK_PARTITION_USR_CONFIG,
    #if (CFG_SUPPORT_MATTER)
    BK_PARTITION_MATTER_FLASH,
    BK_PARTITION_MATTER_FACTORY_FLASH,
    #endif
    BK_PARTITION_BLE_SVR_CONFIG,
    BK_PARTITION_BLE_BONDING_FLASH,
    BK_PARTITION_MAX,
} bk_partition_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef enum
{
    BK_FLASH_EMBEDDED,
    BK_FLASH_SPI,
    BK_FLASH_MAX,
    BK_FLASH_NONE,
} bk_flash_t;

typedef struct
{
    bk_flash_t                 partition_owner;
    const char                *partition_description;
    uint32_t                   partition_start_addr;
    uint32_t                   partition_length;
    uint32_t                   partition_options;
} platform_logic_partition_t;

typedef platform_logic_partition_t  bk_logic_partition_t;


/******************************************************
 *                 Global Variables
 ******************************************************/

extern const char  *flash_name[];  /**< A name string of a Flash drive */

/******************************************************
*                 Function Declarations
******************************************************/
OSStatus BkFlashInit(void);
OSStatus BkFlashUninit(void);

/**@brief   Get the infomation of the specified flash area
 *
 * @param   inPartition:  The target flash logical partition which should be erased
 *
 * @return  bk_logi_partition struct
 */
bk_logic_partition_t *bk_flash_get_info( bk_partition_t inPartition );


/**@brief   Erase an area on a Flash logical partition
 *
 * @note    Erase on an address will erase all data on a sector that the
 *          address is belonged to, this function does not save data that
 *          beyond the address area but in the affected sector, the data
 *          will be lost.
 *
 * @param  inPartition     : The target flash logical partition which should be erased
 * @param  off_set         : Start address of the erased flash area
 * @param  size    	   : Size of the erased flash area
 *
 * @return  kNoErr        : On success.
 * @return  kGeneralErr   : If an error occurred with any step
 */
OSStatus bk_flash_erase(bk_partition_t inPartition, uint32_t off_set, uint32_t size);

/**@brief  Write data to an area on a Flash logical partition
 *
 * @param  inPartition    : The target flash logical partition which should be read which should be written
 * @param  off_set        :
 * @param  inBuffer       : point to the data buffer that will be written to flash
 * @param  inBufferLength : The length of the buffer
 *
 * @return  kNoErr        : On success.
 * @return  kGeneralErr   : If an error occurred with any step
 */
OSStatus bk_flash_write( bk_partition_t inPartition, volatile uint32_t off_set, uint8_t *inBuffer, uint32_t inBufferLength);

/**@brief    Read data from an area on a Flash to data buffer in RAM
 *
 * @param    inPartition    : The target flash logical partition which should be read
 * @param    off_set        :
 * @param    outBuffer      : Point to the data buffer that stores the data read from flash
 * @param    inBufferLength : The length of the buffer
 *
 * @return    kNoErr        : On success.
 * @return    kGeneralErr   : If an error occurred with any step
 */
OSStatus bk_flash_read( bk_partition_t inPartition, volatile uint32_t off_set, uint8_t *outBuffer, uint32_t inBufferLength);



/**@brief    Set security options on a logical partition
 *
 * @param    partition     : The target flash logical partition
 * @param    offset        : Point to the start address that the data is read, and
 *                          point to the last unread address after this function is
 *                          returned, so you can call this function serval times without
 *                          update this start address.
 * @param    size          : Size of enabled flash area
 *
 * @return    kNoErr        : On success.
 * @return    kGeneralErr   : If an error occurred with any step
 */
OSStatus bk_flash_enable_security(PROTECT_TYPE type );
OSStatus bk_flash_get_security(PROTECT_TYPE *protect_flag);
int bk_flash_abs_addr_write(unsigned int  off_set, const unsigned char *inBuffer, unsigned int  size,unsigned char eraseflag);
int bk_flash_abs_addr_erase(unsigned int flashOffset, unsigned int size);
int bk_flash_abs_addr_read(unsigned int off_set, unsigned char *outBuffer, unsigned int size);

/**@brief    Get flash unique ID
 *
 * @param    uid_buf       : The read buf to save uid
 * @param    uid_buf_len   : Size of buf to read
 *
 * @return    > 0          : how many bytes is valid in uid_buf
 * @return    = 0          : para is error, or read nothing
 * @return    < 0          : some err happened
 */
uint32_t bk_flash_get_uid(uint8_t *uid_buf, uint32_t uid_buf_len);

#ifdef BOOTLOADER
/**@brief    Disable security options on a logical partition
 *
 * @param    partition     : The target flash logical partition
 * @param    offset        :  Point to the start address that the data is read, and
 *                          point to the last unread address after this function is
 *                          returned, so you can call this function serval times without
 *                          update this start address.
 * @param    size          : Size of disabled flash area
 *
 * @return   kNoErr        : On success.
 * @return   kGeneralErr   : If an error occurred with any step
 */
OSStatus BkFlashDisableSecurity( bk_partition_t partition, uint32_t off_set, uint32_t size );
#endif


int hal_flash_init(void);
uint32_t bk_flash_read_otp(uint32_t off_set, uint8_t *out_buf, uint32_t buf_len);


/** @} */
/** @} */

#endif


