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

#ifndef __BEKENDRIVERUART_H__
#define __BEKENDRIVERUART_H__

#pragma once
#include "include.h"
#include "uart_pub.h"
#include "rtos_pub.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_UART _BK_ UART Driver
* @brief  Universal Asynchronous Receiver Transmitter (UART) Functions
* @{
*/

/******************************************************
 *					 Enumerations
 ******************************************************/
typedef enum
{
    BK_UART_1 = 0,
    BK_UART_2,
    #if (CFG_SOC_NAME == SOC_BK7252N)
    BK_UART_3,
    #endif
    BK_UART_MAX,
} bk_uart_t;

/******************************************************
 *					  Structures
 ******************************************************/
/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t  size;
    uint32_t  head;
    uint32_t  tail;
    uint8_t  *buffer;
} ring_buffer_t;

/******************************************************
 *                 Function Declarations
 ******************************************************/
OSStatus bk_uart_initialize_test( bk_uart_t uart, uint8_t config, ring_buffer_t *optional_rx_buffer );
/**@brief Initialises a UART interface
 *
 * @note Prepares an UART hardware interface for communications
 *
 * @param  uart     : the interface which should be initialised
 * @param  config   : UART configuration structure
 * @param  optional_rx_buffer : Pointer to an optional RX ring buffer
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_uart_initialize( bk_uart_t uart, const bk_uart_config_t *config, ring_buffer_t *optional_rx_buffer );

/**@brief Deinitialises a UART interface
 *
 * @param  uart : the interface which should be deinitialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_uart_finalize( bk_uart_t uart );


/**@brief Transmit data on a UART interface
 *
 * @param  uart     : the UART interface
 * @param  data     : pointer to the start of data
 * @param  size     : number of bytes to transmit
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_uart_send( bk_uart_t uart, const void *data, uint32_t size );


/**@brief Receive data on a UART interface
 *
 * @param  uart     : the UART interface
 * @param  data     : pointer to the buffer which will store incoming data
 * @param  size     : number of bytes to receive
 * @param  timeout  : timeout in milisecond
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_uart_recv( bk_uart_t uart, void *data, uint32_t size, uint32_t timeout );
OSStatus bk_uart_recv_prefetch( bk_uart_t uart, void *data, uint32_t size, uint32_t timeout );

/**@brief Read the length of the data that is already recived by uart driver and stored in buffer
 *
 * @param uart     : the UART interface
 *
 * @return    Data length
 */
uint32_t bk_uart_get_length_in_buffer( bk_uart_t uart );

/**@brief set receive data callback on a UART interface, please notice that, the callback will beinvoked in ISR
 *
 * @param  uart     : the UART interface
 * @param  callback : callback to invoke when UART receive data, use NULL to clear callback
 * @param  param  : user param which will filled in callback as 2nd parameter
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_uart_set_rx_callback( bk_uart_t uart, uart_callback callback, void *param );

/** @} */
/** @} */

#endif
