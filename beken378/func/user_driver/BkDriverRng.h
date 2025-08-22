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

#ifndef __BEKENDRIVERRNG_H__
#define __BEKENDRIVERRNG_H__

#pragma once
#include "include.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_RNG _BK_ RNG Driver
 * @brief  Random Number Generater(RNG) Functions
 * @{
 */

/******************************************************
 *                  Macros
 ******************************************************/

#define PlatformRandomBytes BkRandomNumberRead   /**< For API compatible with older version */

/******************************************************
 *               Enumerations
 ******************************************************/


/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
*                 Function Declarations
******************************************************/



/**@brief Fill in a memory buffer with random data
 *
 * @param inBuffer        : Point to a valid memory buffer, this function will fill
                            in this memory with random numbers after executed
 * @param inByteCount     : Length of the memory buffer (bytes)
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkRandomNumberRead( void *inBuffer, int inByteCount );

int bk_rand(void);
/** @} */
/** @} */

#endif


