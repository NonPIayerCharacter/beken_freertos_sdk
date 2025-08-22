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

#ifndef MESH_GENERAL_API_H_
#define MESH_GENERAL_API_H_
/**
 ****************************************************************************************
 * @addtogroup APP
 * @ingroup BEKEN
 *
 * @brief Mesh Application Module entry point
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
#include "mesh_api.h"


#define SYS_RESET_BY_POWER_ON    0
#define SYS_RESET_BY_WDT         1

#define UART_CMD_PROV_EN         1

#define LED_SHOW_P1_ENABLE       1

#define POWER_3P5_ENABLE         0

typedef uint8_t sys_reset_src_t;

void app_mesh_enable(void);
void app_mesh_disable(void);
void app_store_mesh_info(void);


uint8_t app_relay_user_adv(uint16_t interval, uint8_t nb_tx, uint8_t data_len, const uint8_t *data);
sys_reset_src_t sys_check_reset_src(void);

void app_set_dev_key_param(uint8_t *p_dev_key);
void app_set_net_key_param(uint8_t *p_net_key, uint16 key_id);
void app_set_app_key_param(uint8_t *p_app_key, uint16 key_id);


#if (UART_CMD_PROV_EN)
void app_test_add_key(void);
#endif /* UART_CMD_PROV_EN */

#endif // MESH_GENERAL_API_H_




