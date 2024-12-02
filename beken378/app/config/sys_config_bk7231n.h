#ifndef _SYS_CONFIG_H_
#define _SYS_CONFIG_H_

/*SUMMARY: macro--1: OPEN; --0:CLOSE*/

/* uart2 for debug, and generally, uart1 is used for communication.
   what is more, uart1 maybe is not bound out*/
#define CFG_USE_UART1                              1
#define CFG_JTAG_ENABLE                            0
#define OSMALLOC_STATISTICAL                       0
#define CFG_MEM_DEBUG                              0

/*section 0-----app macro config-----*/
#define CFG_IEEE80211N                             1
#define CFG_IEEE80211N_HT40                        0

/*section 1-----OS macro config-----*/
#define RTOS_ALIOS_THINGS                          1
#define RTOS_RT_THREAD                             2
#define RTOS_FREERTOS                              3
#define RTOS_LITEOS                                4

/* CFG_OS_FREERTOS----RTOS_FREERTOS
   CFG_SUPPORT_RTT----RTOS_RT_THREAD
   CFG_SUPPORT_LITEOS--RTOS_LITEOS
   CFG_SUPPORT_ALIOS---RTOS_ALIOS_THINGS
 */
#define CFG_SUPPORT_RTOS                           RTOS_FREERTOS

#define FREERTOS_V9                                1
#define FREERTOS_V10                               2
#define CFG_FREERTOS_VER                           FREERTOS_V9

#define ENC_METHOD_NULL                            1
#define ENC_METHOD_XOR                             2
#define ENC_METHOD_AES                             3

#define FAST_CONNECT_INFO_ENC_METHOD               ENC_METHOD_NULL

#define THD_APPLICATION_PRIORITY                   3
#define THD_CORE_PRIORITY                          2
#define THD_UMP3_PRIORITY                          4
#define THD_UBG_PRIORITY                           5
#define THD_LWIP_PRIORITY                          4
#define THD_INIT_PRIORITY                          4
#define THD_RECONNECT_PRIORITY                     4
#define THD_MEDIA_PRIORITY                         4
#define THD_WPAS_PRIORITY                          5
#define THD_EXTENDED_APP_PRIORITY                  5
#define THD_HOSTAPD_PRIORITY                       5
#define THDD_KEY_SCAN_PRIORITY                     7

/*section 2-----function macro config-----*/
#define CFG_SUPPORT_MATTER                         0

#define CFG_TX_EVM_TEST                            1
#define CFG_RX_SENSITIVITY_TEST                    1
#define CFG_AP_MONITOR_COEXIST                     0
#if CFG_AP_MONITOR_COEXIST
#define CFG_AP_MONITOR_COEXIST_DEMO                0
/*AP will switch to ori channel when tbtt arrive*/
#define CFG_AP_MONITOR_COEXIST_TBTT                0
#endif

#define CFG_WIFI_TX_KEYDATA_USE_LOWEST_RATE        1

#define CfG_MACRO_MAYBE_UNUSED                     1
#if CfG_MACRO_MAYBE_UNUSED
#define CFG_WIFI_RSSI                              1
#define CFG_WIFI_VERSION                           1
#define CFG_WIFI_CHANNEL                           1
#define CFG_WIFI_SLOTTIME                          1
#define CFG_WIFI_DBG_TROGGER                       1
#define CFG_MODE_SET                               1
#define CFG_FILTER_SET                             1
#define CFG_RC_STATS                               1
#endif

#define CFG_ROLE_LAUNCH                            0
#define CFG_USE_WPA_29                             1
#define CFG_WPA_CTRL_IFACE                         1
#define CFG_RWNX_QOS_MSDU                          1
#define CFG_WLAN_FAST_CONNECT                      0
#if CFG_WLAN_FAST_CONNECT
/* fast connect will connects to AP without any scan */
#define CFG_WLAN_FAST_CONNECT_WITHOUT_SCAN         0
/* fast connect will disconnect with AP first before AUTH */
#define CFG_WLAN_FAST_CONNECT_DEAUTH_FIRST         0
#endif
#define CFG_WPA2_ENTERPRISE                        0
#define CFG_WPA3_ENTERPRISE                        0
/* WPS(WSC) Support */
#define CFG_WIFI_WPS                               0
/* WiFi Direct Support, CFG_WIFI_WPS must be enabled */
#define CFG_WIFI_P2P                               0
#define CFG_WIFI_P2P_GO                            0
/* Vendor Specific IEs when STA Probe Req/Association Req*/
#define CFG_WIFI_STA_VSIE                          0
/* Vendor Specific IEs when AP Beacon  */
#define CFG_WIFI_AP_VSIE                           0
/* Custom softap basic rates, supported rates, ht mcs set */
#define CFG_WIFI_AP_CUSTOM_RATES                   0
/* repush txdesc when txl_reset happens */
#define CFG_WIFI_REPUSH_WHEN_RESET                 0

/*Use macro to shut down some unused functions*/
#define CFG_WPA_MAYBE_UNUSED                       1
#if CFG_WPA_MAYBE_UNUSED
#define CONFIG_NOTIFICATION                        1
#define CONFIG_EID_FLAG                            1
#define CONFIG_PMKSA_EXISTS                        1
#define CONFIG_GTK_REKEY                           1
#endif

/*
 * Support set softap modes: BGN, BG, B. Macro
 * CFG_AP_SUPPORT_HT_IE must be enabled to support N mode
 */
#define CFG_WIFI_AP_HW_MODE                        0
#define CFG_WIFI_FORCE_LEGACY_AS_LOWEST_RATE       1
/* Support 802.11mc FTM */
#define CFG_WIFI_FTM                               0
#if CFG_WIFI_FTM
/* FTM initiator */
#define CFG_WIFI_FTM_INITIATOR                     1
/* FTM responder: fake ftm responder */
#define CFG_WIFI_FTM_RESPONDER                     0
#endif

/* PMF */
#define CFG_IEEE80211W                             0

#define CFG_WIFI_OCV                               0
#if CFG_WIFI_OCV
#undef CFG_IEEE80211W
#define CFG_IEEE80211W                             1
#endif

#if CFG_WPA_CTRL_IFACE
#undef CFG_ROLE_LAUNCH
#define CFG_ROLE_LAUNCH                            0
#endif
#define CFG_WPA3                                   1
#if CFG_WPA3
#undef CFG_USE_WPA_29
#define CFG_USE_WPA_29                             1
#undef CFG_IEEE80211W
#define CFG_IEEE80211W                             1
#define CFG_OWE                                    1
/* use wpa2 instead of wpa3-sae if in wpa3 transition mode */
#define CFG_CFG_WPA2_PREFER_TO_SAE                 0
#if CFG_WLAN_FAST_CONNECT
/*
 * wpa3 fast connect support, use with caution. If system power off
 * for a long time (24h for example), first connection will fail because
 * pmksa is timedout.
 */
#define CFG_WLAN_FAST_CONNECT_WPA3                 0
#endif
#endif
#define CFG_WFA_CERT                               0
#define CFG_ENABLE_BUTTON                          0
#define CFG_UDISK_MP3                              0
#define CFG_EASY_FLASH                             0
#define CFG_AP_SUPPORT_HT_IE                       0
#define CFG_SUPPORT_BSSID_CONNECT                  0
#define CFG_USE_CONV_UTF8                          0
#define CFG_BK_AWARE                               0
#define CFG_BK_AWARE_OUI                           "\xC8\x47\x8C"
#define CFG_RESTORE_CONNECT                        0
#define CFG_QUICK_TRACK                            0

/* use mbedtls as wpa crypto functions */
#if( ( CFG_SUPPORT_ALIOS ) || ( CFG_SUPPORT_RTT ) )
#define CFG_USE_MBEDTLS                            0
#else
#define CFG_USE_MBEDTLS                            1
#endif
#if CFG_USE_MBEDTLS
#define CFG_MBEDTLS                                1
#endif
#if CFG_QUICK_TRACK
#define _DUT_                                      1
#endif
/*section 3-----driver macro config-----*/
#define CFG_MAC_PHY_BAPASS                         1
#define CFG_SUPPORT_SARADC                         1
#define CFG_SARADC_INTFACE                         1
#define CFG_SARADC_CALIBRATE                       1

#define CFG_SDIO                                   0
#define CFG_SDIO_TRANS                             0
#define CFG_REAL_SDIO                              0

#if CFG_REAL_SDIO
#define FOR_SDIO_BLK_512                           0
#endif

#define CFG_MSDU_RESV_HEAD_LEN                     96
#define CFG_MSDU_RESV_TAIL_LEN                     16

#define CFG_USE_USB_HOST                           0

#define CFG_USB                                    0
#if CFG_USB
#define CFG_SUPPORT_MSD                            1
#define CFG_SUPPORT_HID                            0
#define CFG_SUPPORT_CCD                            0
#define CFG_SUPPORT_UVC                            0
#endif
#define CFG_USE_USB_CHARGE                         0

/*section 4-----DEBUG macro config-----*/
#define CFG_UART_DEBUG                             0
#define CFG_SUPPORT_BKREG                          1
#define CFG_ENABLE_WPA_LOG                         0
#define IPERF_CLOSE                                0  /* close iperf */
#define IPERF_OPEN_WITH_ACCEL                      1  /* open iperf and accel */
#define IPERF_OPEN_ONLY                            2  /* open iperf, but no open accel */
#if(CFG_SUPPORT_MATTER == 1)
#define CFG_IPERF_TEST                             IPERF_CLOSE
#else
#define CFG_IPERF_TEST                             IPERF_OPEN_ONLY
#endif
#if (CFG_IPERF_TEST == IPERF_OPEN_WITH_ACCEL)
#define CFG_IPERF_TEST_ACCEL                       1
#define CFG_IPERF_DONT_MALLOC_BUFFER               1
#endif
#define CFG_TCP_SERVER_TEST                        0
#define CFG_AIRKISS_TEST                           0
#define CFG_ENABLE_DEMO_TEST                       0
#define CFG_WIFI_SENSOR                            0
#define CFG_WIFI_RAW_TX_CMD                        0

#define ASSERT_HALT                                1
#define ASSERT_IGNORE                              2
#define ASSERT_REBOOT                              3
#define CFG_ASSERT_OPTION                          ASSERT_IGNORE

/*section 5-----PRODUCT macro config-----*/
#define CFG_RELEASE_FIRMWARE                       0

/*section 6-----for platform*/
#define SOC_PLATFORM                               1
#define FPGA_PLATFORM                              0
#define CFG_RUNNING_PLATFORM                       SOC_PLATFORM

#define SOC_BK7231                                 1
#define SOC_BK7231U                                2
#define SOC_BK7221U                                3
#define SOC_BK7271                                 4
#define SOC_BK7231N                                5
#define CFG_SOC_NAME                               SOC_BK7231N

#if ((SOC_BK7231N == CFG_SOC_NAME) && CFG_JTAG_ENABLE && CFG_SUPPORT_SARADC)
#error "jtag and saradc are using the same gpio"
#endif

/*section 7-----calibration*/
#if (CFG_RUNNING_PLATFORM == FPGA_PLATFORM)
#define CFG_SUPPORT_CALIBRATION                    0
#define CFG_SUPPORT_MANUAL_CALI                    0
#else
#define CFG_SUPPORT_CALIBRATION                    1
#define CFG_SUPPORT_MANUAL_CALI                    1
//tpc rf pa map power for bk7231u
#define CFG_SUPPORT_TPC_PA_MAP                     1
#endif

/*section 8-----for netstack*/
#define CFG_USE_LWIP_NETSTACK                      1

/*section 9-----for DHCP servicers and client*/
#define CFG_USE_DHCP                               1
#define CFG_USE_DHCPD                              1 // for servicers in ap mode

/*section 11-----temperature detect*/
#define CFG_USE_TEMPERATURE_DETECT                 1
#define CFG_USE_VOLTAGE_DETECT                     0

/*section 12-----for SPIDMA interface*/
#define CFG_USE_SPIDMA                             0
#define CFG_USE_CAMERA_INTF                        0

/*section 13-----for GENERRAL DMA */
#define CFG_GENERAL_DMA                            1

/*section 14-----for FTPD UPGRADE*/
#define CFG_USE_FTPD_UPGRADE                       0

/*section 15-----support customer macro*/
#define CFG_SUPPORT_TIANZHIHENG_DRONE              0

/*section 16-----support mcu & deep sleep*/
#define CFG_USE_MCU_PS                             1

#define CFG_USE_DEEP_PS                            1
#define CFG_USE_BLE_PS                             1
#define CFG_USE_AP_IDLE                            0
#define CFG_USE_FAKERTC_PS                         0
#define CFG_LOW_VOLTAGE_PS                         0
#define CFG_LOW_VOLTAGE_PS_32K_DIV                 0
#define CFG_LOW_VOLTAGE_PS_TEST                    1

/*section 17-----support sta power sleep*/
#define CFG_USE_STA_PS                             1

/*section 18-----AP support stas in power save*/
#define CFG_USE_AP_PS                              0

/*section 19-----for SDCARD HOST*/
#define CFG_USE_SDCARD_HOST                        0

/*section 20 ----- support mp3 decoder*/
#define CONFIG_APP_MP3PLAYER                       0

/*section 21 ----- support ota*/
#if( ( CFG_SUPPORT_ALIOS ) || ( CFG_SUPPORT_RTT ) || (CFG_SUPPORT_MATTER == 1))
#define CFG_SUPPORT_OTA_HTTP                       0
#else
#define CFG_SUPPORT_OTA_HTTP                       1
#endif
#define CFG_SUPPORT_OTA_TFTP                       0

/*section 23 ----- support reduce nomal power*/
#define CFG_SYS_REDUCE_NORMAL_POWER                0

/*section 24 ----- less memery in rwnx*/
#define CFG_LESS_MEMERY_IN_RWNX                    1
#if CFG_IPERF_TEST_ACCEL
#undef CFG_LESS_MEMERY_IN_RWNX
#define CFG_LESS_MEMERY_IN_RWNX                    0
#endif

/*section 25 ----- use audio*/
#define CFG_USE_AUDIO                              0
#define CFG_USE_AUD_DAC                            0
#define CFG_USE_AUD_ADC                            0

/*section 25 ----- use tick time calibrate*/
#define CFG_USE_TICK_CAL                           1

#define CFG_SUPPORT_BLE                            1
#define CFG_BLE_USE_CLI                            1
#define CFG_SUPPORT_BLE_MESH                       0
#define CFG_USE_PTA                                0

#if ((0 == CFG_SUPPORT_BLE) && (CFG_USE_BLE_PS))
#error "check the ble macro, thx!"
#endif

#define BLE_VERSION_4_2                            1
#define BLE_VERSION_5_1                            2
#define BLE_VERSION_5_2                            3

#define CFG_BLE_VERSION                            BLE_VERSION_5_1

#define WIFI_DEFAULT_BLE_REQUEST                   1
#define BLE_DEFAULT_WIFI_REQUEST                   2
#define BLE_WIFI_CO_REQUEST                        3
#define RF_USE_POLICY                              WIFI_DEFAULT_BLE_REQUEST

#define CFG_BLE_ADV_NUM                            1
#if (CFG_SUPPORT_MATTER == 1)
#define CFG_BLE_SCAN_NUM                           0
#else
#define CFG_BLE_SCAN_NUM                           1
#endif
#define CFG_BLE_USE_DYN_RAM                        1

// 0 mean do not support ble master
#define CFG_BLE_INIT_NUM                           0

#define CFG_BLE_CONN_NUM                           1

#if (CFG_BLE_ADV_NUM == 0)
#error "ADV NUM should not be 0"
#endif

#if (CFG_BLE_CONN_NUM == 0)
#error "CONN NUM should not be 0"
#endif

#define CFG_XTAL_FREQUENCE_40M                     40000000 //40MHz
#define CFG_XTAL_FREQUENCE_26M                     26000000 //26MHz
#if (CFG_SOC_NAME == SOC_BK7231N)
#define CFG_XTAL_FREQUENCE                         CFG_XTAL_FREQUENCE_26M//CFG_XTAL_FREQUENCE_40M
#else
#define CFG_XTAL_FREQUENCE                         CFG_XTAL_FREQUENCE_26M
#endif
#define CFG_XTAL_85DEGREE                          0

#if (CFG_SUPPORT_ALIOS)
#undef  CFG_JTAG_ENABLE
#define CFG_JTAG_ENABLE                            0
#undef  CFG_ROLE_LAUNCH
#define CFG_ROLE_LAUNCH                            0
#undef  CFG_USE_MCU_PS
#define CFG_USE_MCU_PS                             RHINO_CONFIG_CPU_PWR_MGMT
#endif

#define LWIP_DEFAULT_MEM_POLICY                    1
#define LWIP_REDUCE_THE_PLAN                       2
#if CFG_IPERF_TEST
/* for iperf test, temporary enlarge lwip MEM_SIZE */
#define CFG_LWIP_MEM_POLICY                        LWIP_DEFAULT_MEM_POLICY
#else
#define CFG_LWIP_MEM_POLICY                        LWIP_REDUCE_THE_PLAN
#endif

#define FLASH_SELECTION_TYPE_DYNAMIC               0 //select with flashID runtime
#define FLASH_SELECTION_TYPE_2M                    0x200000 //2MBytes
#define FLASH_SELECTION_TYPE_4M                    0x400000 //4MBytes
#define FLASH_SELECTION_TYPE_8M                    0x800000 //8MBytes
#define CFG_FLASH_SELECTION_TYPE                   FLASH_SELECTION_TYPE_2M


#if (1 == CFG_SUPPORT_MATTER)
#undef CFG_MBEDTLS
#define CFG_MBEDTLS                                1
#endif

/* watchdog, freertos only */
#define CFG_INT_WDG_ENABLED                        1
#define CFG_INT_WDG_PERIOD_MS                      10000
#define CFG_TASK_WDG_ENABLED                       1
#define CFG_TASK_WDG_PERIOD_MS                     60000
#define DUMP_THREAD_WHEN_TASK_WDG_TIGGERED         1
#define DUMP_STACK_WHEN_TASK_WDG_TIGGERED          1

/*section 29 -----  peripheral interface open  */
#define CFG_USE_I2C1                               0
#define CFG_USE_I2C2                               0

#define CFG_USE_SPI                                0
#define CFG_USE_SPI_MASTER                         0
#define CFG_USE_SPI_SLAVE                          0
#define CFG_USE_SPI_DMA                            0

/*section 30 ----- peripheral interface test case */
#define CFG_PERIPHERAL_TEST                        0
#define CFG_SUPPORT_SPI_TEST                       0


#define AT_SERVICE_CFG                             0

#define CFG_USE_FORCE_LOWVOL_PS                    1

#define CFG_BK_NX_GET_WIFI_SNR                     0

#define CFG_WRAP_LIBC                              1
#endif // _SYS_CONFIG_H_
