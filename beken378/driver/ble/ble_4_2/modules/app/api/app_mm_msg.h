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

#ifndef APP_MM_MSG_H_
#define APP_MM_MSG_H_
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

int app_models_msg_pro_handler(kernel_msg_id_t const msgid,
                               void const *param,
                               kernel_task_id_t const dest_id,
                               kernel_task_id_t const src_id);

void app_mesh_add_models_server(void);

#endif // APP_MM_MSG_H_

