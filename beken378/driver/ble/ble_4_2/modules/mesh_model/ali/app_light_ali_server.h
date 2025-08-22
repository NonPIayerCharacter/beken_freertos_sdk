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

#ifndef _APP_LIGHT_LIGHT_ALI_SERVER_H
#define _APP_LIGHT_LIGHT_ALI_SERVER_H

#include "mal.h"
#include "m_fnd_int.h"

extern m_lid_t g_vdr_lid, g_ln_mdl_lid, g_ctl_ln_mdl_lid;
extern m_lid_t g_hsl_ln_mdl_lid, g_ctl_mdl_lid;
extern m_lid_t g_ctlt_mdl_lid, g_oo_mdl_lid;
extern m_lid_t g_hsl_mdl_lid, g_hslh_mdl_lid, g_hslsat_mdl_lid;

void light_ali_app_init(void);

void light_unBind_complete(void);

void light_prov_start(void);

void app_ai_lights_models_init(uint8_t elmt_idx);

int32_t light_scene_server_data(const m_fnd_model_env_p pmodel_info, uint32_t type, void *pargs);

void ali_light_status_report(uint16_t attr, uint8_t len, uint8_t status);

#endif // _APP_LIGHT_LIGHT_ALI_SERVER_H

