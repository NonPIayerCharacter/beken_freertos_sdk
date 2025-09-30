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

#ifndef __CAMERA_INTF_PUB_H__
#define __CAMERA_INTF_PUB_H__

#include "drv_model_pub.h"
#include "i2c_pub.h"
#include "jpeg_encoder_pub.h"

typedef struct camera_sensor_t camera_sensor_t;

typedef void (*sensor_init_func_t)(DD_HANDLE, DD_HANDLE, camera_sensor_t*);

struct camera_sensor_t
{
	char* name;
	char* i2c_bus;
	I2C_OP_ST* i2c_cfg;
	DJPEG_DESC_ST* ejpeg_cfg;
	sensor_init_func_t init;;
	void* flip;
	void* effect;

};

//void camera_flip(UINT8 n);
void camera_intfer_init(void* ejpeg_data, camera_sensor_t* sensor);
void camera_intfer_deinit(camera_sensor_t* sensor);

camera_sensor_t* camera_detect(void);

//UINT32 camera_intfer_set_video_param(UINT32 ppi_type, UINT32 pfs_type);

#endif // __CAMERA_INTF_PUB_H__

