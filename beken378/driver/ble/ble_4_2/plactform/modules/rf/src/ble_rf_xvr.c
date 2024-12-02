/**
****************************************************************************************
*
* @file rf_xvras.c
*
* @brief Atlas radio initialization and specific functions
*
* Copyright (C) Beken 2009-2015
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup RF_XVR
* @ingroup RF
* @brief Radio Driver
*
* This is the driver block for radio
* @{
****************************************************************************************
*/

/**
 *****************************************************************************************
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>        // for memcpy
#include "common_endian.h"     // endian definitions
#include "common_math.h"       // common math functions
#include "common_utils.h"      // common utility definition
#include "ble_rf.h"            // RF interface
#include "em_map.h"        // RF area mapping

#include "rwip.h"          // for RF API structure definition
#include "reg_blecore.h"   // ble core registers
#include "reg_ble_em_cs.h" // control structure definitions

#include "BK3435_reg.h"           // Platform register
#include "ble_icu.h"
#include "rwip_config.h" // stack configuration

/**
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 **/

/**
****************************************************************************************
* MACROS
*****************************************************************************************
*/


#define RPL_POWER_MAX               0x0c

/**
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 **/


/**
 ****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */

static void rf_reset(void)
{
}

/**
 ****************************************************************************************
 * @brief ISR to be called in BLE ISR routine when RF Interrupt occurs.
 *****************************************************************************************
 */


/**
 *****************************************************************************************
 * @brief Get TX power in dBm from the index in the control structure
 *
 * @param[in] txpwr_idx  Index of the TX power in the control structure
 * @param[in] modulation Modulation: 1 or 2 or 3 MBPS
 *
 * @return The TX power in dBm
 *
 *****************************************************************************************
 */

static uint8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
	// Power table should be provided
	return(0);
}

/**
 *****************************************************************************************
 * @brief Sleep function for  RF.
 *****************************************************************************************
 */

static void rf_sleep(void)
{
#if defined(CFG_BLE)
	ble_deepslcntl_set(ble_deepslcntl_get() |
			BLE_DEEP_SLEEP_ON_BIT |    // RW BLE Core sleep
			BLE_RADIO_SLEEP_EN_BIT |   // Radio sleep
			BLE_OSC_SLEEP_EN_BIT);     // Oscillator sleep
#endif // CFG_BLE
}


/**
 *****************************************************************************************
 * @brief Convert RSSI to dBm
 *
 * @param[in] rssi_reg RSSI read from the HW registers
 *
 * @return The converted RSSI
 *
 *****************************************************************************************
 */

static int8_t rf_rssi_convert (uint8_t rssi_reg)
{
	uint8_t RssidBm = 0; 

	RssidBm = ((rssi_reg) >> 1) - 118;

	return(RssidBm);
}


static uint32_t rf_rpl_reg_rd (uint16_t addr)
{
	uint32_t ret;

	ret = REG_PL_RD((uint32_t)addr);

	return ret;
}

static void rf_rpl_reg_wr (uint16_t addr, uint32_t value)
{
	REG_PL_WR((uint32_t)addr, value);
}
/**
 ****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 **/

static void rf_force_agc_enable(bool en)
{
  
    ble_forceagc_en_setf(en);
  
}
void rf_init(struct rwip_rf_api *api)
{
    // Initialize the RF driver API structure
    api->reg_rd = rf_rpl_reg_rd;
    api->reg_wr = rf_rpl_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_max = RPL_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;

    #if defined(CFG_BLE)
        api->force_agc_enable = rf_force_agc_enable;
    #endif //CFG_BLE

    api->rssi_convert = rf_rssi_convert;
}

