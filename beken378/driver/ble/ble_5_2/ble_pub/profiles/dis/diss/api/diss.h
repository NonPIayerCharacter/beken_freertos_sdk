/**
 ****************************************************************************************
 *
 * @file diss.h
 *
 * @brief Header file - Device Info Service Server - Native API.
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

#ifndef DISS_H_
#define DISS_H_

/**
 ****************************************************************************************
 * @addtogroup DISS Device Information Service Server
 * @brief Device Information Service Server - Native API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPES DEFINITIONS
 ****************************************************************************************
 */
/// DIS server callback set
typedef struct diss_cb
{
    /**
     ****************************************************************************************
     * @brief This function is called when GATT server user has initiated event send to peer
     *        device or if an error occurs.
     *
     * @param[in] token         Procedure token that must be returned in confirmation function
     * @param[in] val_id        Requested value identifier (@see enum diss_val_id)
     ****************************************************************************************
     */
    void (*cb_value_get) (uint32_t token, uint8_t val_id);
} diss_cb_t;


/*
 * API
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Store DIS value information that can be used by service.
 *
 * @param[in] value_id Value identifier (@see enum diss_val_id)
 * @param[in] length   Data value length
 * @param[in] p_data   Pointer to the value data
 *
 * @return Function execution status (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t diss_value_set(uint8_t val_id, uint8_t length, const uint8_t* p_data);

/**
 ****************************************************************************************
 * @brief Provide DIS value information requested by a peer device
 *
 * @param[in] token    Procedure token provided in request
 * @param[in] length   Data value length
 * @param[in] p_data   Pointer to the value data
 *
 * @return Function execution status (@see enum hl_err)
 ****************************************************************************************
 */
uint16_t diss_value_cfm(uint32_t token, uint8_t length, const uint8_t* p_data);

/// @} DISSTASK
#endif // DISS_H_
