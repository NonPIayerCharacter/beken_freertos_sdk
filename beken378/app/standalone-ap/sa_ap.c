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

#include "include.h"
#include "sa_ap.h"
#include "schedule_pub.h"
#include "drv_model_pub.h"
#include "uart_pub.h"
#include "rw_pub.h"
#include "rxu_task.h"
#include "mm_task.h"
#include "me_task.h"
#include "apm_task.h"
#include "vif_mgmt.h"

#include "hostapd_cfg.h"
#include "rw_ieee80211.h"

#if CFG_USE_LWIP_NETSTACK
#include "lwip_intf.h"
#endif

#if CFG_USE_TEMPERATURE_DETECT
#include "temp_detect_pub.h"
#endif
#include "sys_ctrl_pub.h"

extern void mm_bcn_init(void);
void sa_ap_init(void)
{
    if (rwm_mgmt_is_vif_first_used() == NULL)
    {
        SAAP_PRT("[saap]MM_RESET_REQ\r\n");
        rw_msg_send_reset();

        SAAP_PRT("[saap]ME_CONFIG_REQ\r\n");
        rw_msg_send_me_config_req();

        SAAP_PRT("[saap]ME_CHAN_CONFIG_REQ\r\n");
        rw_msg_send_me_chan_config_req();

        SAAP_PRT("[saap]MM_START_REQ\r\n");
        rw_msg_send_start();
    }
    #if !CFG_WPA_CTRL_IFACE
    else
    {
        SAAP_PRT("[saap]mm_bcn_init\r\n");
        mm_bcn_init();
    }
    #endif
}

void sa_ap_uninit(void)
{

}
// eof

