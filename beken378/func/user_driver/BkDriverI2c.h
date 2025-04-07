/**
 ******************************************************************************
 * @file    BkDriverI2C.h
 * @brief   This file provides all the headers of I2C operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2024 BEKEN Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */

#ifndef __BEKENDRIVERI2C_H__
#define __BEKENDRIVERI2C_H__

#pragma once
#include "include.h"
#include "rtos_pub.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_I2C _BK_ I2C Driver
* @brief  Inter-IC bus (I2C) Functions
* @{
*/

/******************************************************
 *                   Macros
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    I2C_MASTER_MODE,     /**< I2C device is master */
    I2C_SLAVE_MODE       /**< I2C device is slave */
} bk_i2c_ms_mode_t;

typedef enum
{
    I2C_ADDRESS_WITH_INNER,     /**< I2C device with inner address*/
    I2C_ADDRESS_WITHOUT_INNER    /**< I2C device without inner address */
} bk_i2c_address_mode_t;

typedef enum
{
    I2C_ADDRESS_WIDTH_7BIT,     /**< I2C device has 7bit address */
    I2C_ADDRESS_WIDTH_10BIT    /**< I2C device has 10bit address */
} bk_i2c_bus_address_width_t;

//typedef enum
//{
//    I2C_LOW_SPEED_MODE,         /**< I2C clock speed for 10Khz devices */
//    I2C_STANDARD_SPEED_MODE,    /**< I2C clock speed for 100Khz devices */
//    I2C_HIGH_SPEED_MODE         /**< I2C clock speed for 400Khz devices */
//} bk_i2c_speed_mode_t;

/******************************************************
 *                    Structures
******************************************************/

typedef struct
{
    bk_i2c_ms_mode_t            ms_mode;        /**< I2c device master or slave */
    bk_i2c_address_mode_t       address_mode;   /**< The address with inner or not */
    bk_i2c_bus_address_width_t  address_width;  /**< I2C device's address length */
    uint32                      baudrate;     /**< Speed mode the device operates in */
} bk_i2c_device_t;

// typedef struct
// {
//     const void*  tx_buffer;  /**< A pointer to the data to be transmitted. If NULL, the message is an RX message when 'combined' is FALSE */
//     void*        rx_buffer;  /**< A pointer to the data to be transmitted. If NULL, the message is an TX message when 'combined' is FALSE */
//     uint16_t     tx_length;  /**< Number of bytes to transmit */
//     uint16_t     rx_length;  /**< Number of bytes to receive */
//     uint16_t     retries;    /**< Number of times to retry the message */
//     bool combined;           /**< If set, this message is used for both tx and rx. */
// } bk_i2c_message_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/



/******************************************************
 *                 Function Declarations
 ******************************************************/
/**@brief Bk I2c BkI2cOpen
* 
* @param i2cdevice  : i2c device config paramter
* @return    i2c_hdl        : on success.
*/
uint32 bk_i2c_open(bk_i2c_device_t i2cdevice);

/**@brief Bk I2c BkI2cClose
 * @param i2c_hdl    : i2c handle
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        : if an error occurred during I2c BkI2cClose
 */
OSStatus bk_i2c_close(uint32 i2c_hdl);

/**@brief Bk I2c set slave addr
 * @param slave_addr    : i2c slave addr(for slave is is must) 
 * @return
 */
void bk_i2c_set_slave_addr(uint32 slave_addr);

/**@brief Bk I2c Master Write
 * @param i2c_hdl    : i2c handle
 * @param tx_buffer  : buffer to tx
 * @param data_length : tx buffer length
 * @param salve_id   : i2c slave addr (for master send)
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_master_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint8_t salve_id, uint32 timeout);

/**@brief Bk I2c Master Read
 * @param i2c_hdl    : i2c handle
 * @param tx_buffer  : buffer for rx
 * @param data_length : rx msg length
 * @param salve_id   : i2c slave addr (for master send)
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_master_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint8_t salve_id, uint32 timeout);

/**@brief Bk I2c Slave Write
 * @param i2c_hdl    : i2c handle
 * @param tx_buffer  : buffer to tx
 * @param data_length : tx buffer length
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_slave_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint32 timeout);

/**@brief Bk I2c Slave Read
 * @param i2c_hdl    : i2c handle
 * @param tx_buffer  : buffer for rx
 * @param data_length : rx msg length
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_slave_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint32 timeout);

/**@brief Bk I2c memory Write
 * @param i2c_hdl    : i2c handle
 * @param tx_buffer  : buffer to write
 * @param data_length : write buffer length
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_memory_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint8_t salve_id, uint8_t inner_addr, uint32 timeout);

/**@brief Bk I2c memory Read
 * @param i2c_hdl    : i2c handle
 * @param rx_buffer  : buffer to read
 * @param data_length : read buffer length
 * @param timeout : timeout for msg trans
 * @return    DRV_SUCCESS        : on success.
 *            DRV_FAILURE        :txbuffer null
 *            DD_HANDLE_UNVALID  :i2c handle err
 *            others(1)          :i2c write or read timeout
 */
OSStatus bk_i2c_memory_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint8_t salve_id, uint8_t inner_addr, uint32 timeout);

/** @} */
/** @} */

#endif


