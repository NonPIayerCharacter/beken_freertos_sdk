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

#ifndef _MAC_CONFIG_H_
#define _MAC_CONFIG_H_

#define CONFIG_ROLE_NULL        0
#define CONFIG_ROLE_AP          1
#define CONFIG_ROLE_STA         2
#define CONFIG_ROLE_COEXIST     3

extern uint8_t system_mac[];

void cfg_load_mac(u8 *mac);
uint32_t cfg_param_init(void);

void wifi_get_mac_address(char *mac, u8 type);
int wifi_set_mac_address(char *mac);
int wifi_set_mac_address_to_efuse(UINT8 *mac);
int wifi_get_mac_address_from_efuse(UINT8 *mac);

int wifi_write_efuse(UINT8 addr, UINT8 data);
UINT8 wifi_read_efuse(UINT8 addr);

#endif /*_MAC_CONFIG_H_*/
