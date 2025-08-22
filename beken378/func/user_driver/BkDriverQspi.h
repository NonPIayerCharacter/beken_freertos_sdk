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

#ifndef __BEKENDRIVERPWM_H__
#define __BEKENDRIVERPWM_H__
#include "include.h"
#include "rtos_pub.h"
#include "qspi_pub.h"


#if (CFG_SOC_NAME != SOC_BK7252N)
/**@brief Initialises a QSPI pin
 *
 *
 * @param qspi_dcache_drv_desc        : the qspi configure which should be initialised
 * @param linemode  				  : linemode : 1 :1 linemode
 												   4: 4 linemode
 * @param div   					  : QSPI div freqence
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */

OSStatus bk_qspi_dcache_initialize(qspi_dcache_drv_desc *qspi_config);

/**@brief Start QSPI
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */



OSStatus bk_qspi_start(void);

/**@brief Stops QSPI
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */



OSStatus bk_qspi_stop(void);
#else
/**@brief Initialize qspi psram
 *
 * @param     linemode  	: 0 = single mode, 1 = 4 linemode
 * @return    kNoErr        : on success.
 *            others        : other error.
 */
OSStatus bk_qspi_psram_initialize(uint8_t line_mode);

/**@brief Enable quad mode
 *
 * @return    kNoErr        : on success.
 *            others        : other error.
 */
OSStatus bk_qspi_psram_enter_quad_mode(void);

/**@brief Enable quad write mode
 *
 * @return    kNoErr        : on success.
 *            others        : other error.
 */
OSStatus bk_qspi_psram_quad_write(void);

/**@brief Enable quad read mode
 *
 * @return    kNoErr        : on success.
 *            others        : other error.
 */
OSStatus bk_qspi_psaram_quad_read(void);

/**@brief Enable mcu read/write mode
 *
 * @return    kNoErr        : on success.
 *            others        : other error.
 */
OSStatus bk_qspi_psram_switch_mcu_mode(void);
#endif
#endif
