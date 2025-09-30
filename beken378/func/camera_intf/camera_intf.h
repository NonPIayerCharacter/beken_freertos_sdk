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

#ifndef __CAMERA_INTF_H__
#define __CAMERA_INTF_H__

#include "camera_intf_config.h"

#define CAMERA_RESET_GPIO_INDEX		GPIO16
#define CAMERA_RESET_HIGH_VAL       1
#define CAMERA_RESET_LOW_VAL        0

#define EJPEG_DMA_CHNAL             GDMA_CHANNEL_5
#define EJPEG_DELAY_HTIMER_CHNAL    5
#define EJPEG_DELAY_HTIMER_VAL      (2)  // 2ms
#define USE_JTAG_FOR_DEBUG          1 // USE I2C2 and not I2C1!!!
#define I2C_WIRTE_TIMEOUT_COUNT     20

void camera_intf_sccb_write(UINT8 addr, UINT8 data);
void camera_intf_sccb_read(UINT8 addr, UINT8* data);
uint8_t camera_intf_sccb_write2(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len);
uint8_t camera_intf_sccb_read2(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len);

#endif
