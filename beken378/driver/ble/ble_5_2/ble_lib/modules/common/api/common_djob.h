/**
 ****************************************************************************************
 *
 * @file common_djob.h
 *
 * @brief Common delayed job definitions
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */
#ifndef _COMMON_DJOB_H_
#define _COMMON_DJOB_H_

/**
 ****************************************************************************************
 * @defgroup COMMON_DJOB Utilities
 * @ingroup COMMON
 * @brief  Delayed job utilities
 *
 * This module contains the delayed job utilities functions and macros.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>       // standard definitions
#include <stddef.h>       // standard definitions
#include "common_list.h"      // common bt definitions


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * ENUMERATIONS DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Job function to called into a background context
 *
 * @param[in] p_env   Pointer to environment that will be used as callback parameter.
 ****************************************************************************************
 */
typedef void (*common_djob_cb)(void* p_env);

/// Job element structure
typedef struct common_djob
{
    /// List element header
    common_list_hdr_t hdr;
    /// Pointer to environment that will be used as callback parameter.
    void*         p_env;
    /// Callback to execute in background context
    common_djob_cb    cb;
} common_djob_t;


/*
 * CONSTANT DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * Delayed Job functions
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Prepare Delayed job structure
 *
 * @param[in] p_djob   Pointer to the delayed job structure
 * @param[in] cb       Callback to execute in background context
 * @param[in] p_env    Pointer to environment that will be used as callback parameter.
 ****************************************************************************************
 */
void common_djob_prepare(common_djob_t* p_djob, common_djob_cb cb, void* p_env);

/**
 ****************************************************************************************
 * @brief Register to execute a job delayed in background
 *
 * @param[in] p_djob   Pointer to the delayed job structure
 ****************************************************************************************
 */
void common_djob_reg(common_djob_t* p_djob);

/**
 ****************************************************************************************
 * @brief Un-register a job that waits to be executed
 *
 * @param[in] p_djob   Pointer to the delayed job structure
 ****************************************************************************************
 */
void common_djob_unreg(common_djob_t* p_djob);

/**
 ****************************************************************************************
 * @brief Initialize Common delayed job module.
 *
 * @param[in] init_type  Type of initialization (@see enum rwip_init_type)
 ****************************************************************************************
 */
void common_djob_init(uint8_t init_type);
/// @} COMMON_DJOB

#endif // _COMMON_DJOB_H_
