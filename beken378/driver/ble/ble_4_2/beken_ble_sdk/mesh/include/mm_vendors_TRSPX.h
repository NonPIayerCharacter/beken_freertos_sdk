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

#ifndef _MM_VENDORS_TRSPX_H_
#define _MM_VENDORS_TRSPX_H_

#include "mm_api_int.h"       // Mesh Model Application Programming Interface Internal Definitions
#include "mesh_tb_timer.h"    // Mesh Timer Manager Definitions
#include "mesh_api.h"       // Mesh Model API Definitions
#include "mm_defines.h"     // Mesh Model Definitions
#include "mm_tb.h"          // Mesh Model Tool Boxes Definitions
#include "mm_route.h"       // Mesh Model Routing Module Definitions
#include "mesh_tb_buf.h"    // Mesh Buffer Manager Definitions
#include "mm_vendors_morenode.h"

#define MM_ID_VENDORS_TRSPX               (0x06A80000) // 0x06A80000 invild id

#define TRSPX_MSG_OPCODE                          (0xC006A8)



uint16_t mm_vendors_trspx_register(uint8_t elmt_idx, m_lid_t *p_mdl_lid);




#endif // _MM_VENDORS_H_
