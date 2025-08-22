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

#ifndef __APP_LED_H__
#define __APP_LED_H__

typedef enum
{
    STA_NONE         = 0,
    POWER_ON,
    LED_CONNECT,
    LED_DISCONNECT,
    MONITOR_MODE,
    SOFTAP_MODE,
    TIMER_POLL,
} DEV_STATE;


UINT32 app_led_init(void);
void app_led_send_msg(DEV_STATE new_msg);

#endif
