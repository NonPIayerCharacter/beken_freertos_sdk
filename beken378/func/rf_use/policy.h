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

typedef enum
{
    SCEN_BLE_NORMAL,
    SCEN_BLE_CONNECTING,
    SCEN_BLE_MAX,
} ble_scenarios_t;

typedef enum
{
    SCEN_WIFI_NORMAL,
    SCEN_WIFI_CONNECTING,
    SCEN_WIFI_MAX,
} wifi_scenarios_t;

typedef enum
{
    RF_EVENT_WIFI_DATA_SEND,
    RF_EVENT_MAX,
} rf_event_type;

OSStatus get_event_priority(uint8_t event, uint8_t *priority);
OSStatus set_event_priority(uint8_t event, uint8_t priority);
void set_ble_scenarios(ble_scenarios_t scenarios);
void set_wifi_scenarios(wifi_scenarios_t scenarios);
ble_scenarios_t get_ble_scanarios(void);
wifi_scenarios_t get_wifi_scanarios(void);
