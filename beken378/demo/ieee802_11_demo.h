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

#ifndef _IEEE802_11_DEMO_H_
#define _IEEE802_11_DEMO_H_


#define WLAN_DEFAULT_IP         "192.168.0.1"
#define WLAN_DEFAULT_GW         "192.168.0.1"
#define WLAN_DEFAULT_MASK       "255.255.255.0"

void demo_scan_app_init(void);
void demo_scan_adv_app_init(uint8_t *oob_ssid);
void demo_softap_app_init(char *ap_ssid, char *ap_key);
void demo_sta_app_init(char *oob_ssid,char *connect_key);
void demo_sta_bssid_app_init(uint8_t *bssid, char *connect_key);
void demo_sta_adv_app_init(char *oob_ssid,char *connect_key);
void demo_wlan_app_init(VIF_ADDCFG_PTR cfg);
void demo_state_app_init(void);
void demo_ip_app_init(void);
#if (CFG_SOC_NAME == SOC_BK7252N)
void ate_ip_app_init(void);
#endif

//static const char *crypto_str[];
#endif

