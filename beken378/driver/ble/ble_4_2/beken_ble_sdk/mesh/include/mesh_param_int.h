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

#ifndef MESH_PARAM_INT_H_
#define MESH_PARAM_INT_H_
#include <stdint.h>

typedef struct mesh_stack_param_int
{
    /// Default beacon interval for transmissions of unprovisioned device beacons (in milliseconds)
    uint32_t m_bcn_default_unprov_bcn_intv_ms; // M_BCN_DEFAULT_UNPROV_BCN_INTV_MS
    /// Default TTL to use for transmission  TODO Need to be managed by Foundation layer
    uint32_t m_ttl_default; //M_TTL_DEFAULT
    /// Number of advertising transmission to perform
    uint32_t m_adv_nb_tx;  // M_ADV_NB_TX
    /// Number of advertising transmission to perform for the network packet.
    uint32_t m_adv_nb_net_tx;
    /// Advertising interval slots
    uint32_t m_adv_interval; //M_ADV_INTERVAL
    /// Advertising interval slots for the network packet.
    uint32_t m_adv_net_interval;
    /// Advertising connectable interval slots
    uint32_t m_adv_con_interval;//M_ADV_CON_INTERVAL
    /// Scanning interval  slots
    uint32_t m_adv_scan_interval;//M_ADV_SCAN_INTERVAL
    /// Duration between two update of Connectable advertising data
    uint32_t m_proxy_con_adv_update_dur; //M_PROXY_CON_ADV_UPDATE_DUR

    uint32_t m_prov_link_timeout; //M_PROV_LINK_TIMEOUT

    uint32_t m_proxy_con_adv_always_on;

    uint32_t m_proxy_conn_keep_en;//PROXY_KEEP_CONN_EN;

} mesh_stack_param_int_t;


extern mesh_stack_param_int_t m_stack_param;
void mesh_stack_param_init(void);

#endif //
