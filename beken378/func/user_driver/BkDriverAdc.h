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

#ifndef __BEKENDRIVERADC_H__
#define __BEKENDRIVERADC_H__

#pragma once
#include "include.h"

/** @addtogroup BK_PLATFORM
* @{
*/

/** @defgroup BK_ADC _BK_ ADC Driver
  * @brief  Analog to Digital Converter (ADC) Functions
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

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                     Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**@biref Initialises an ADC interface
 *
 * Prepares an ADC hardware interface for sampling
 *
 * @param adc            : the interface which should be initialised
 * @param sampling_cycle : sampling period in number of ADC clock cycles. If the
 *                         MCU does not support the value provided, the closest
 *                         supported value is used.
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkAdcInitialize( bk_adc_t adc, uint32_t sampling_cycle );


/**@biref Takes a single sample from an ADC interface
 *
 * Takes a single sample from an ADC interface
 *
 * @param adc    : the interface which should be sampled
 * @param output : pointer to a variable which will receive the sample
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkAdcTakeSample( bk_adc_t adc, uint16_t *output );


/**@biref Takes multiple samples from an ADC interface
 *
 * Takes multiple samples from an ADC interface and stores them in
 * a memory buffer
 *
 * @param adc           : the interface which should be sampled
 * @param buffer        : a memory buffer which will receive the samples
 *                        Each sample will be uint16_t little endian.
 * @param buffer_length : length in bytes of the memory buffer.
 *
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus BkAdcTakeSampleStreram( bk_adc_t adc, void *buffer, uint16_t buffer_length );


/**@biref     De-initialises an ADC interface
 *
 * @abstract Turns off an ADC hardware interface
 *
 * @param  adc : the interface which should be de-initialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus  BkAdcFinalize( bk_adc_t adc );

/** @} */
/** @} */
#endif
