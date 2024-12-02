#ifndef _ARCH_CONFIG_H_
#define _ARCH_CONFIG_H_

#include "mac.h"
#if (CFG_SUPPORT_ALIOS)
#include "mac_config.h"
#endif

#define PARAM_CFG_DEBUG

#ifdef PARAM_CFG_DEBUG
#define PARAM_CFG_PRT      os_printf
#define PARAM_CFG_WARN     warning_prf
#define PARAM_CFG_FATAL    fatal_prf
#else
#define PARAM_CFG_PRT      null_prf
#define PARAM_CFG_WARN     null_prf
#define PARAM_CFG_FATAL    null_prf
#endif

#define CONFIG_ROLE_NULL        0
#define CONFIG_ROLE_AP          1
#define CONFIG_ROLE_STA         2
#define CONFIG_ROLE_COEXIST     3

#define MAC_EFUSE                0
#define MAC_ITEM                 1
#define MAC_RF_OTP_FLASH         2
#define WIFI_MAC_POS             MAC_RF_OTP_FLASH

#define DEFAULT_CHANNEL_AP      11

typedef struct fast_connect_param
{
    uint8_t bssid[6];
    uint8_t chann;
} fast_connect_param_t;

typedef struct general_param
{
    uint8_t role;
    uint8_t dhcp_enable;
    uint32_t ip_addr;
    uint32_t ip_mask;
    uint32_t ip_gw;
} general_param_t;

typedef struct ap_param
{
    struct mac_addr bssid;
    struct mac_ssid ssid;

    uint8_t chann;
    uint8_t cipher_suite;
    uint8_t key[65];
    uint8_t key_len;
    bool hidden_ssid;
#if CFG_WIFI_AP_VSIE
	uint8_t vsie[255];
	uint8_t vsie_len;
#endif
#if CFG_WIFI_AP_CUSTOM_RATES
	/* make last basic_rates be zero */
	int basic_rates[16];
	/* make last supported_rates be zero */
	int supported_rates[16];
	/* don't change mcs_set size */
	uint8_t mcs_set[16];
#endif
#if CFG_WIFI_AP_HW_MODE
	int hw_mode;
#endif
} ap_param_t;

typedef struct sta_param
{
    struct mac_addr own_mac;
    struct mac_ssid ssid;
    uint8_t cipher_suite;
    uint8_t key[108]; //The maximum supported key length is 107 bits.If it exceeds 107 bits,the error "input buffer overflow" will return
    uint8_t key_len;
    uint8_t fast_connect_set;
    fast_connect_param_t fast_connect;

#if CFG_WPA2_ENTERPRISE
	/* starts of WPA2-Enterprise/WPA3-Enterprise EAP-TLS configuration */
	char eap[16];					   /**< phase1 authType: TLS/TTLS/SIM */
	char identity[32];				   /**< user identity */
	char ca[32];					   /**< CA certificate filename */
	char client_cert[32];			   /**< client's Certification filename in PEM,DER format */
	char private_key[32];			   /**< client's private key filename in PEM,DER format */
	char private_key_passwd[32];	   /**< client's private key password */
	char phase1[32];				   /**< client's phase1 parameters */
#endif
} sta_param_t;

extern general_param_t *g_wlan_general_param;
extern ap_param_t *g_ap_param_ptr;
extern sta_param_t *g_sta_param_ptr;

uint32_t cfg_param_init(void);

#if (CFG_OS_FREERTOS) || (CFG_SUPPORT_RTT) || (CFG_SUPPORT_LITEOS)
extern uint8_t system_mac[];

void cfg_load_mac(u8 *mac);
uint32_t cfg_ap_is_open_system(void);
void wifi_get_mac_address(char *mac, u8 type);
int wifi_set_mac_address(char *mac);
int wifi_set_mac_address_to_efuse(UINT8 *mac);
int wifi_get_mac_address_from_efuse(UINT8 *mac);
int wifi_write_efuse(UINT8 addr, UINT8 data);
UINT8 wifi_read_efuse(UINT8 addr);
#endif
#endif
