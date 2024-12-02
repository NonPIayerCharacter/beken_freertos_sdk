#ifndef _ATSVR_COMM_H_
#define _ATSVR_COMM_H_
#include "typedef.h"
#include "BkDriverUart.h"
#include "atsvr_ble.h"


#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#define AT_VERSION     "1.1"

#define ATSVR_AT_CFG    1

/* MAX size of product ID */
#define MAX_SIZE_OF_PRODUCT_ID (10)

/* MAX size of product secret */
#define MAX_SIZE_OF_PRODUCT_SECRET (32)

/* MAX size of device name */
#define MAX_SIZE_OF_DEVICE_NAME (48)

/* MAX size of device secret */
#define MAX_SIZE_OF_DEVICE_SECRET (64)

/* MAX size of device cert file name */
#define MAX_SIZE_OF_DEVICE_CERT_FILE_NAME (128)

/* MAX size of device key file name */
#define MAX_SIZE_OF_DEVICE_SECRET_FILE_NAME (128)

/* MAX size of region len */
#define MAX_SIZE_OF_REGION (64)

/* MAX size of hostname len */
#define MAX_SIZE_OF_HOSTNMAE (64)

#define MAX_SIZE_MQTT_USER  (64)

#define MAX_SIZE_MQTT_PWD  (64)

#define MAX_SIZE_MQTT_CLIENTID  (256)

#pragma pack (1)

typedef struct __WIFI_MODE_INFO
{
    uint8 mode;
    uint8 autoconnect;
}WIFI_MODE_INFO;

typedef struct _SOFTAP_ENV_INFO
{
    char ap_ssid[33];
    char ap_key[64];
    uint8 channel;
    uint8 ap_enc;
    uint8 hidden;
    uint8 proto;
    char ap_local_ip[16];
    char ap_mask[16];
    char ap_gate[16];
    uint8 dhcp;
}SOFTAP_ENV_INFO;

typedef struct _STA_ENV_INFO
{
    char con_ssid[33];
    char con_key[64];
    uint8 dhcp;
    char sta_local_ip[16];
    char sta_mask[16];
    char sta_gate[16];
}STA_ENV_INFO;

typedef struct _SERIAL_ENV_INFO
{
    uint32 baudrate;
    uint8 databits;
    uint8 stopbits;
    uint8 parity;
    uint8 flow_control;
}SERIAL_ENV_INFO;


typedef struct {
    char product_id[MAX_SIZE_OF_PRODUCT_ID + 1];
    char device_name[MAX_SIZE_OF_DEVICE_NAME + 1];
    char region[MAX_SIZE_OF_REGION];
} DEV_INFO;


typedef struct _NTP_INFO{
    char enable;
    char hostname[MAX_SIZE_OF_HOSTNMAE];
    int  timezone;
} NTP_INFO;

typedef struct _DNS_INFO{
    char enable;
    char dns1[16];
    char  dns2[16];
    char  dns3[16];
} DNS_INFO;

typedef struct __PSK_INFO{
    int linkid;
    char psk[MAX_SIZE_OF_DEVICE_SECRET +1];
    char hint[MAX_SIZE_OF_DEVICE_SECRET +1];
}PSK_INFO;

typedef struct _ENV_PARAM{
    uint8 sysstore;
    uint8 workmode;  //0:factory mode,1:common mode
    WIFI_MODE_INFO wifimode;
    SOFTAP_ENV_INFO apinfo;
    STA_ENV_INFO stainfo;
    SERIAL_ENV_INFO uartinfo;
    uint8 net_transmission_mode;   //0:普通模式，1：透传模式
    DEV_INFO deviceinfo;
    NTP_INFO ntpinfo;
    DNS_INFO dnsinfo;
    PSK_INFO pskinfo;
    BLE_PARAM_T ble_param;
}ENV_PARAM;


#pragma pack ()


/// List of Application NVDS TAG identifiers
enum _AT_ENV_TAG
{
    /// SYSSTORE Address
    TAG_SYSSTORE_OFFSET                   = 0,
    LEN_SYSSTORE_VALUE                    = 1,

    TAG_WORKMOD_OFFSET                    = TAG_SYSSTORE_OFFSET + LEN_SYSSTORE_VALUE,
    LEN_WORKMOD_VALUE                     = 1,

    TAG_WIFIMODED_OFFSET                  = TAG_WORKMOD_OFFSET + LEN_WORKMOD_VALUE,
    LEN_WIFIMODED_VALUE                   = sizeof(WIFI_MODE_INFO),

    TAG_WIFI_AP_INFO_OFFSET               = TAG_WIFIMODED_OFFSET + LEN_WIFIMODED_VALUE,
    LEN_WIFI_AP_INFO_VALUE                = sizeof(SOFTAP_ENV_INFO),

    TAG_WIFI_STA_INFO_OFFSET              = TAG_WIFI_AP_INFO_OFFSET + LEN_WIFI_AP_INFO_VALUE,
    LEN_WIFI_STA_INFO_VALUE               = sizeof(STA_ENV_INFO),

    TAG_UART_DEF_INFO_OFFSET              = TAG_WIFI_STA_INFO_OFFSET + LEN_WIFI_STA_INFO_VALUE,
    LEN_UART_DEF_INFO_VALUE               = sizeof(SERIAL_ENV_INFO),

    TAG_NET_PASSTHROUGH_OFFSET            = TAG_UART_DEF_INFO_OFFSET + LEN_UART_DEF_INFO_VALUE,
    LEN_NET_PASSTHROUGH_VALUE             = 1,

    TAG_DEVICED_INFO_OFFSET               = TAG_NET_PASSTHROUGH_OFFSET + LEN_NET_PASSTHROUGH_VALUE,
    LEN_DEVICED_INFO_VALUE                = sizeof(DEV_INFO),

    TAG_NTP_INFO_OFFSET                   = TAG_DEVICED_INFO_OFFSET + LEN_DEVICED_INFO_VALUE,
    LEN_NTP_INFO_VALUE                    = sizeof(NTP_INFO),

    TAG_DNS_INFO_OFFSET                   = TAG_NTP_INFO_OFFSET + LEN_NTP_INFO_VALUE,
    LEN_DNS_INFO_VALUE                    = sizeof(DNS_INFO),

    TAG_PSK_INFO_OFFSET                 = TAG_DNS_INFO_OFFSET + LEN_DNS_INFO_VALUE,
    LEN_PSK_INFO_VALUE                  = sizeof(PSK_INFO),

    TAG_BLE_INFO_OFFSET                 = TAG_PSK_INFO_OFFSET + LEN_PSK_INFO_VALUE,
    LEN_BLE_INFO_VALUE                  = sizeof(BLE_PARAM_T),
};

typedef   enum _AT_ENV_TAG  AT_ENV_TAG;

extern ENV_PARAM g_env_param;

#define AT_DEBUG


#ifdef AT_DEBUG

#define AT_FUNC_ENTRY                                              \
    {                                                               \
        bk_printf("FUNC_ENTRY:   %s L#%d \n", __FUNCTION__, __LINE__); \
    }
#define AT_FUNC_EXIT                                              \
    {                                                              \
        bk_printf("FUNC_EXIT:   %s L#%d \n", __FUNCTION__, __LINE__); \
        return;                                                    \
    }
#define AT_FUNC_EXIT_RC(x)                                                                     \
    {                                                                                           \
        bk_printf("FUNC_EXIT:   %s L#%d Return Code : %ld \n", __FUNCTION__, __LINE__, (long)(x)); \
        return x;                                                                               \
    }
#else
#define AT_FUNC_ENTRY
#define AT_FUNC_EXIT \
    {                 \
        return;       \
    }
#define AT_FUNC_EXIT_RC(x) \
    {                       \
        return x;           \
    }
#endif

#define REG_READ(addr)          (*((volatile UINT32 *)(addr)))
#define REG_WRITE(addr, _data)  (*((volatile UINT32 *)(addr)) = (_data))

int write_env_to_flash(AT_ENV_TAG tag, int datalen ,uint8* buf);
int read_env_from_flash(AT_ENV_TAG tag, int len ,uint8* buf);

void *at_malloc(unsigned int size);
void at_free(void *p);
void log_output_state(int flag);

#endif
