/**
 ******************************************************************************
 * @file    BkDriverPwm.h
 * @brief   This file provides all the headers of PWM operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 BEKEN Inc.
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

#ifndef __BEKENDRIVERPWM_H__
#define __BEKENDRIVERPWM_H__

#pragma once
#include "include.h"
#include "rtos_pub.h"
#include "pwm_pub.h"

/** @addtogroup BK_PLATFORM
  * @{
  */

/** @defgroup BK_PWM _BK_ PWM Driver
  * @brief  Pulse-Width Modulation (PWM) Functions
  * @{
  */

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    BK_PWM_0,
    BK_PWM_1,
    BK_PWM_2,
    BK_PWM_3,
    BK_PWM_4,
    BK_PWM_5,
    BK_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    BK_PWM_NONE,
} bk_pwm_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
*                 Function Declarations
******************************************************/
/**@brief Initialises a PWM
 *
 * @note  Prepares a Pulse-Width Modulation pin for use.
 *
 * @param pwm         : the PWM interface which should be initialised
 * @param frequency   : Output signal frequency in Hertz
 * @param duty_cycle  : Set pwm first level reversal time
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */

OSStatus bk_pwm_initialize(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle);

/**@brief Starts PWM output on a PWM interface
 *
 * @note  Starts Pulse-Width Modulation signal output on a PWM pin
 *
 * @param pwm        : the PWM interface which should be started
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_start(bk_pwm_t pwm);

/**@brief updat PWM
 *
 * @note  Update pwm cycle and duty_cycle when pwm working.
 *
 * @param pwm               : the PWM interface which should be started
 * @param frequency         : pwm frequency
 * @param duty_cycle        : set first level change time
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_update_param(bk_pwm_t pwm, uint32_t frequency, uint32_t duty_cycle);

/**@brief Stops output on a PWM pin
 *
 * @note  Stops Pulse-Width Modulation signal output on a PWM pin
 *
 * @param pwm        : the PWM interface which should be stopped
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_stop(bk_pwm_t pwm);

/**@brief Initialises capture PWM
 *
 * @note  Initialises pwm as capture mode
 *
 * @param pwm         : the PWM interface which should be initialised
 * @param cap_mode    : capture mode: 4:posedge, 5:negedge, 6:both
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_capture_initialize(bk_pwm_t pwm, uint8_t cap_mode);

/**@brief Start capture PWM
 *
 * @note  start pwm as capture
 *
 * @param pwm         : the PWM interface which should be started
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_capture_start(bk_pwm_t pwm);

/**@brief Stop capture PWM
 *
 * @note  stop pwm as capture
 *
 * @param pwm         : the PWM interface which should be stopped
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_capture_stop(bk_pwm_t pwm);

/**@brief Get capture PWM value
 *
 * @note  get pwm captured value
 *
 * @param pwm         : the PWM interface which should be get
 *
 * @return    other   : Capture value.
 * @return    0       : if an error occurred with any step
 */
UINT32 bk_pwm_get_capvalue(bk_pwm_t pwm);

/**@brief Initialises two PWMs with group mode
 *
 * @note  Initialises pwm-group mode
 *
 * @param pwm1        : the PWM1 interface which should be initialised  low level first on pin
 * @param pwm2        : the PWM2 interface which should be initialised, high level first on pin
 * @param frequency   : Output signal frequency in Hertz
 * @param duty_cycle1 : Set pwm1 first level reversal time
 * @param duty_cycle2 : Set pwm2 first level reversal time
 * @param dead_band   : Set reversal time between two pwms high level
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_cw_initialize(bk_pwm_t pwm1, bk_pwm_t pwm2,uint32_t frequency, uint32_t duty_cycle1, uint32_t duty_cycle2, uint32_t dead_band);

/**@brief Start two PWMs with group mode
 *
 * @note   Start pwm group mode
 *
 * @param pwm1        : the PWM1 interface which should be started
 * @param pwm2        : the PWM2 interface which should be started
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_cw_start(bk_pwm_t pwm1, bk_pwm_t pwm2);

/**@brief Updata two PWMs with group mode
 *
 * @note  Updata param pwm-group mode
 *
 * @param pwm1        : the PWM1 interface which should be updata, low level first on pin
 * @param pwm2        : the PWM2 interface which should be updata, high level first on pin
 * @param frequency   : Output signal frequency in Hertz
 * @param duty_cycle1 : Set pwm1 first level reversal time
 * @param duty_cycle2 : Set pwm2 first level reversal time
 * @param dead_band   : Set reversal time between two pwms high level
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_cw_update_param(bk_pwm_t pwm1, bk_pwm_t pwm2,uint32_t frequency, uint32_t duty_cycle1, uint32_t duty_cycle, uint32_t dead_band);

/**@brief Stop two PWMs with group mode
 *
 * @note   Stop pwm group mode
 *
 * @param pwm1        : the PWM1 interface which should be stopped
 * @param pwm2        : the PWM2 interface which should be stopped
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus bk_pwm_cw_stop(bk_pwm_t pwm1, bk_pwm_t pwm2);

/** @} */
/** @} */

#endif
