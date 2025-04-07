#ifndef _RW_MSG_PUB_H_
#define _RW_MSG_PUB_H_

typedef enum {
    WIFI_COUNTRY_POLICY_AUTO,   /**< Country policy is auto, use the country info of AP to which the station is connected */
    WIFI_COUNTRY_POLICY_MANUAL, /**< Country policy is manual, always use the configured country info */
} wifi_country_policy_t;

typedef enum {
    /* for station mode */
    RW_EVT_STA_IDLE = 0,
    RW_EVT_STA_CONNECTING,
    RW_EVT_STA_BEACON_LOSE,
    RW_EVT_STA_PASSWORD_WRONG,
    RW_EVT_STA_NO_AP_FOUND,
    RW_EVT_STA_ASSOC_FULL,
    RW_EVT_STA_DEAUTH,
    RW_EVT_STA_AUTH_FAILED,
    RW_EVT_STA_DISASSOC,
    RW_EVT_STA_ASSOC_FAILED,
    RW_EVT_STA_ACTIVE_DISCONNECTED,
    RW_EVT_STA_CONNECTED,
    RW_EVT_STA_GOT_IP,
    /* for softap mode */
    RW_EVT_AP_CONNECTED,          /* a client association success */
    RW_EVT_AP_DISCONNECTED,    /* a client disconnect */
    RW_EVT_AP_CONNECT_FAILED, /* a client association failed */
    RW_EVT_AP_GOT_IP,
    RW_EVT_MAX
}rw_evt_type;

typedef enum {
    RW_STG_STA_IDLE = 0,
    RW_STG_STA_SCAN,
    RW_STG_STA_AUTH,
    RW_STG_STA_EXT_AUTH,
    RW_STG_STA_ASSOC,
    RW_STG_STA_KEY_HANDSHARK,
    RW_STG_STA_GET_IP,
    RW_STG_STA_COMPLETE
}rw_stage_type;

typedef struct wlan_status_s {
    rw_evt_type evt_type;
    uint8_t mac[6];
    uint16_t reason_code;
    uint32_t ipaddr;
}wlan_status_t;

/** @brief Structure describing WiFi country-based regional restrictions. */
typedef struct {
    char                  cc[3];          /**< country code string */
    uint8_t               schan;          /**< start channel */
    uint8_t               nchan;          /**< total channel number */
    int8_t                max_tx_power;   /**< maximum tx power */
    wifi_country_policy_t policy;         /**< country policy */
} wifi_country_t;

typedef struct sta_scan_res
{
    UINT8 bssid[6];
    char ssid[32];  /**< The SSID of an access point. */
    char on_channel; // 1: ds IE channel=center_freq, 0: !=   
    char channel;
    UINT16 beacon_int;
    UINT16 caps;
    int level;
	int security; // security type
    UINT8 tsf[8];
    UINT32 ie_len;
    /* Followed by ie_len of IE data */
}SCAN_RST_ITEM_T, *SCAN_RST_ITEM_PTR;

typedef struct scanu_rst_upload
{
    UINT16 ref;
    UINT16 scanu_num;
    struct sta_scan_res **res;
}SCAN_RST_UPLOAD_T, *SCAN_RST_UPLOAD_PTR;

extern void mhdr_set_station_status(rw_evt_type val);
extern rw_evt_type mhdr_get_station_status(void);
extern void mhdr_set_station_stage(rw_stage_type val);
extern rw_stage_type mhdr_get_station_stage(void);
UINT32 rw_ieee80211_set_country(const wifi_country_t *country);
UINT32 rw_ieee80211_get_country(wifi_country_t *country);

#endif /*_RW_MSG_PUB_H_*/
