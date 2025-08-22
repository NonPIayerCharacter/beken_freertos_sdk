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

#ifndef _WIFI_SENSOR_H_
#define _WIFI_SENSOR_H_

#define BK_WSD_OK               0
#define BK_WSD_FAIL            -1


/** @brief  bk wifi senser detecting movement callback.
 *
 *  @return   1        : something move.
 *  @return   0        : no movement in there
 */
typedef void (*bk_wsd_cb)(int status);

int bk_wifi_detect_movement_start(bk_wsd_cb callback);
void bk_wifi_detect_movement_stop(void);

#endif

