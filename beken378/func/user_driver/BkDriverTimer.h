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

#ifndef __BEKENDRIVERTIMER_H__
#define __BEKENDRIVERTIMER_H__

#pragma once
#include "include.h"
#include "rtos_pub.h"
#include "bk_timer_pub.h"

#define BKDRIVERTIMRE_TEST_DEMO           0

/** @addtogroup BK_PLATFORM
  * @{
  */

/** @defgroup BK_TIMER _BK_ TIMER Driver
  * @brief  Pulse-Width Modulation (PWM) Functions
  * @{
  */

/******************************************************
 *                   Enumerations
 ******************************************************/


/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
*                 Function Declarations
******************************************************/


/**@brief Initialises a timer and start
 *
 * @note  user can user timer0 timer1 timer2 timer4 timer5
 *
 * @param timer_id      : the timer id
 * @param time_ms       : time value(unit:ms)
 * @param callback      : callback
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_timer_initialize(uint8_t timer_id, uint32_t time_ms, void *callback);

/**@brief stop timer
 *
 * @note  user can user timer0 timer1 timer2 timer4 timer5
 *
 * @param timer_id      : the timer id
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_timer_stop(uint8_t timer_id);

extern UINT32 bk_get_timer_cnt(uint8_t timer_id);

/** @} */
/** @} */

#endif
