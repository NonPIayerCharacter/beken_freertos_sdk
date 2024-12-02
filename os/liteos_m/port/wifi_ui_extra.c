#include "include.h"
#include "wlan_ui_pub.h"
#include "rw_pub.h"
#include "vif_mgmt.h"
#include "sa_station.h"
#include "param_config.h"
#include "common/ieee802_11_defs.h"
#include "driver_beken.h"
#include "mac_ie.h"
#include "sa_ap.h"
#include "main_none.h"
#include "sm.h"
#include "mac.h"
#include "sm_task.h"
#include "scan_task.h"
#include "hal_machw.h"
#include "error.h"
#include "lwip_netif_address.h"
#include "lwip/inet.h"
#include <stdbool.h>
#include "rw_pub.h"
#include "power_save_pub.h"
#include "rwnx.h"
#include "net.h"
#include "mm_bcn.h"
#include "mm_task.h"
#include "mcu_ps_pub.h"
#include "manual_ps_pub.h"
#include "gpio_pub.h"
#include "phy_trident.h"
#include "rw_msg_rx.h"
#include "app.h"
#include "ate_app.h"
#include "wdt_pub.h"
#include "start_type_pub.h"
#include "wpa_psk_cache.h"
#include "drv_model_pub.h"
#include "str_pub.h"
#include "bk7011_cal_pub.h"
#include "wifi_ui_extra.h"
#include "common_utils.h"

extern void bk_wlan_set_coexist_at_init_phase(uint8_t current_role);

void los_wlan_ap_para_info_get(network_InitTypeDef_st *ap_info, unsigned char *security, unsigned char *chann)
{
	if ((NULL == ap_info)
		|| (NULL == security)
		|| (NULL == chann)
		|| (NULL == g_ap_param_ptr))
		return;

	os_memcpy(ap_info->wifi_ssid, g_ap_param_ptr->ssid.array, 32);
	os_memcpy(ap_info->wifi_key, g_ap_param_ptr->key, g_ap_param_ptr->key_len);
	g_ap_param_ptr->key_len = g_ap_param_ptr->key_len;

	*chann = g_ap_param_ptr->chann;
	*security = g_ap_param_ptr->cipher_suite;
}

void los_wlan_ap_para_info_set(network_InitTypeDef_st *ap_info, unsigned char security, unsigned char chann)
{
	if (ap_info == NULL)
		return;

	os_memcpy(g_ap_param_ptr->ssid.array, ap_info->wifi_ssid, os_strlen(ap_info->wifi_ssid));
	g_ap_param_ptr->ssid.length = os_strlen(ap_info->wifi_ssid);

	os_memcpy(g_ap_param_ptr->key, ap_info->wifi_key, os_strlen(ap_info->wifi_key));
	g_ap_param_ptr->key_len = os_strlen(ap_info->wifi_key);

	g_ap_param_ptr->chann = chann;
	g_ap_param_ptr->cipher_suite = security;
}

static void los_wlan_ap_init(network_InitTypeDef_st *inNetworkInitPara, unsigned char security, unsigned char chann)
{
	if (!g_ap_param_ptr)
		ASSERT(g_ap_param_ptr);

	if (MAC_ADDR_NULL((u8 *)&g_ap_param_ptr->bssid))
		wifi_get_mac_address((char *)(&g_ap_param_ptr->bssid), CONFIG_ROLE_AP);

	if (!g_wlan_general_param) {
		g_wlan_general_param = (general_param_t *)os_zalloc(sizeof(general_param_t));
		ASSERT(g_wlan_general_param);
	}
	g_wlan_general_param->role = CONFIG_ROLE_AP;
	bk_wlan_set_coexist_at_init_phase(CONFIG_ROLE_AP);

	if (inNetworkInitPara) {
		if (inNetworkInitPara->dhcp_mode == DHCP_SERVER)
			g_wlan_general_param->dhcp_enable = 1;
		else
			g_wlan_general_param->dhcp_enable = 0;
		inet_aton(inNetworkInitPara->local_ip_addr, &(g_wlan_general_param->ip_addr));
		inet_aton(inNetworkInitPara->net_mask, &(g_wlan_general_param->ip_mask));
		inet_aton(inNetworkInitPara->gateway_ip_addr, &(g_wlan_general_param->ip_gw));
	}

	if (inNetworkInitPara) {
		UINT32 reg = RF_HOLD_BY_AP_BIT;
		sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_SET, &reg);
	}

	sa_ap_init();
}

void los_wlan_start_ap(network_InitTypeDef_st *inNetworkInitParaAP, unsigned char security, unsigned char chann)
{
	/* stop lwip netif */
	uap_ip_down();

	/* set AP parameter, ssid, akm, etc. */
	los_wlan_ap_init(inNetworkInitParaAP, security, chann);

	// enable hostapd
	wlan_ap_enable();

	// reload bss configuration
	wlan_ap_reload();

	/* now ap has started, set ip address to this interface */
	ip_address_set(BK_SOFT_AP,
				   inNetworkInitParaAP->dhcp_mode,
				   inNetworkInitParaAP->local_ip_addr,
				   inNetworkInitParaAP->net_mask,
				   inNetworkInitParaAP->gateway_ip_addr,
				   inNetworkInitParaAP->dns_server_ip_addr);

	/* restart lwip network */
	uap_ip_start();

}

int los_wlan_start_sta(network_InitTypeDef_st *inNetworkInitPara, char *psk, unsigned int psk_len, int chan)
{
#if CFG_WPA_CTRL_IFACE
	/* diconnect previous connection if may */
	sta_ip_down();	// XXX: WLAN_DISCONNECT_EVENT may handle this
	wlan_sta_disconnect();
#endif
	mhdr_set_station_status(RW_EVT_STA_CONNECTING);

#if (RF_USE_POLICY == BLE_DEFAULT_WIFI_REQUEST)
	wifi_station_status_event_notice(0, RW_EVT_STA_CONNECTING);
#endif

#if (CFG_SOC_NAME == SOC_BK7231N)
	if (get_ate_mode_state()) {
		// cunliang20210407 set blk_standby_cfg with blk_txen_cfg like txevm, qunshan confirmed
		rwnx_cal_en_extra_txpa();
	}
#endif
	bk_wlan_sta_init(inNetworkInitPara);
	if (psk == NULL)
		psk_len = 0;
	wpa_psk_request(g_sta_param_ptr->ssid.array, g_sta_param_ptr->ssid.length, (char *)g_sta_param_ptr->key, (unsigned char *)psk, psk_len);

#if CFG_WPA_CTRL_IFACE
	/* enable wpa_supplicant */
	wlan_sta_enable();
	/* set network parameters: ssid, passphase */
	wlan_sta_set((uint8_t *)inNetworkInitPara->wifi_ssid, os_strlen(inNetworkInitPara->wifi_ssid), (uint8_t *)inNetworkInitPara->wifi_key);
	/* connect to AP */
	wlan_sta_connect(chan);
#endif

	/* set IP mode */
	ip_address_set(inNetworkInitPara->wifi_mode,
				   inNetworkInitPara->dhcp_mode,
				   inNetworkInitPara->local_ip_addr,
				   inNetworkInitPara->net_mask,
				   inNetworkInitPara->gateway_ip_addr,
				   inNetworkInitPara->dns_server_ip_addr);

	return kNoErr;
}

UINT8* bk_ble_get_mac_addr(void)
{
    return (UINT8*)&common_default_bdaddr;
}

// eof

