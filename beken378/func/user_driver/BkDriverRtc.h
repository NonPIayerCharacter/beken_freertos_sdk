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

#ifndef __BEKENDRIVERRTC_H__
#define __BEKENDRIVERRTC_H__

#pragma once
#include "include.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_RTC _BK_ RTC Driver
* @brief  Real-time clock (RTC) Functions
* @{
*/

/******************************************************
 *                   Macros
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef platform_rtc_time_t           bk_rtc_time_t;

/******************************************************
*                    Structures
******************************************************/

/******************************************************
 *                     Variables
 ******************************************************/

/******************************************************
                Function Declarations
 ******************************************************/



/**@brief This function will initialize the on board CPU real time clock
 *
 * @note  This function should be called by _BK_ system when initializing clocks, so
 *        It is not needed to be called by application
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
void BkRtcInitialize(void);

/**@brief This function will return the value of time read from the on board CPU real time clock. Time value must be given in the format of
 * the structure bk_rtc_time_t
 *
 * @param time        : pointer to a time structure
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkRtcGetTime(bk_rtc_time_t *time);

/**@brief This function will set MCU RTC time to a new value. Time value must be given in the format of
 * the structure bk_rtc_time_t
 *
 * @param time        : pointer to a time structure
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkRtcSetTime(bk_rtc_time_t *time);

/** @} */
/** @} */

#endif


