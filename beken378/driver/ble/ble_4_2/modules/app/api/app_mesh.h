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

#ifndef APP_MESH_H_
#define APP_MESH_H_
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup BEKEN
 *
 * @brief OADS Application Module entry point
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration


#include <stdint.h>          // Standard Integer Definition
#include "kernel_task.h"         // Kernel Task Definition
#include "m_tb_state.h"
#include "mesh_tb_timer.h"
#if (NVDS_SUPPORT)
#include "nvds.h"
#endif
#include "application.h"


/*
 *  DEFINITION
 ****************************************************************************************
 */

#define MESH_UNPROV_ADV_TIME  (12 * 60 * 1000)
/*
 * STRUCTURES DEFINITION
 ****************************************************************************************
 */


/// mesh Application Module Environment Structure
typedef struct app_mesh_env_tag
{
    /// Connection handle
    uint8_t conidx;

    mesh_tb_timer_t               timer_upd;
} app_mesh_env_tag_t;
/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */

/// mesh Application environment
extern struct app_mesh_env_tag app_mesh_env;

/// Table of message handlers
extern const struct kernel_state_handler app_mesh_table_handler;
/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *
 * braces Application Functions
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize braces Application Module
 ****************************************************************************************
 */
void app_mesh_init(void);
/**
 ****************************************************************************************
 * @brief Add a mesh  instance in the DB
 ****************************************************************************************
 */
void app_mesh_add_mesh(void);
/**
 ****************************************************************************************
 * @brief Enable the oad Service
 ****************************************************************************************
 */

void app_check_power_reset_state(void);

void app_unprov_adv_timeout_set(uint32_t timer);
/**
 ****************************************************************************************
 * @brief Send a step_info
 ****************************************************************************************
 */

/*********************************************************************
* LOCAL FUNCTIONS
*/

void bk_ble_mesh_set_cid(uint16_t cid);
void bk_ble_mesh_set_crpl(uint16_t crpl);
void bk_ble_mesh_set_feature(uint32_t feature);
void bk_ble_mesh_set_pid(uint16_t pid);
void bk_ble_mesh_set_vid(uint16_t vid);
void bk_ble_mesh_set_dev_uuid(uint8_t *device_uuid);
void bk_ble_mesh_set_auth_data(uint8_t *auth_data);
void bk_ble_mesh_set_capabilities(uint8_t number_of_elements,
                                  uint8_t public_key_type, uint8_t static_oob_type, uint8_t output_oob_size,
                                  uint16_t output_oob_action, uint8_t input_oob_size, uint16_t input_oob_action);

#endif // APP_MESH_H_

