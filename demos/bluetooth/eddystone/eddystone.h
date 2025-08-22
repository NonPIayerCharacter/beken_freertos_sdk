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

#ifndef _EDDYSTONE_H_
#define _EDDYSTONE_H_
#include "bk_err.h"
#include "ble_api_5_x.h"
#include "app_ble.h"

extern int demo_start(void);
extern ble_err_t ble_eddystone_post_msg(uint16_t msg_id, void *data,uint32_t len);

#endif // _EDDYSTONE_H_
// EOF

