#ifndef _WLAN_UI_PUB_
#define _WLAN_UI_PUB_


/**
  * @brief     configure country info
  *
  * @attention 
  *     1. The default country is {.cc="CN", .schan=1, .nchan=13, policy=WIFI_COUNTRY_POLICY_AUTO}
  *     2. When the country policy is WIFI_COUNTRY_POLICY_AUTO, the country info of the AP to which
  *               the station is connected is used. E.g. if the configured country info is {.cc="USA", .schan=1, .nchan=11}
  *               and the country info of the AP to which the station is connected is {.cc="JP", .schan=1, .nchan=14}
  *               then the country info that will be used is {.cc="JP", .schan=1, .nchan=14}. If the station disconnected
  *               from the AP the country info is set back back to the country info of the station automatically,
  *               {.cc="USA", .schan=1, .nchan=11} in the example.
  *     3. When the country policy is WIFI_COUNTRY_POLICY_MANUAL, always use the configured country info.
  *     4. When the country info is changed because of configuration or because the station connects to a different
  *               external AP, the country IE in probe response/beacon of the soft-AP is changed also.
  *     5. The country configuration is not stored into flash
  *     6. This API doesn't validate the per-country rules, it's up to the user to fill in all fields according to
  *               local regulations.
  *
  * @param     country   the configured country info
  *
  * @return
  *    - kNoErr: succeed
  *    - kNotInitializedErr: WiFi is not initialized
  *    - kParamErr: invalid argument
  */
OSStatus bk_wlan_set_country(const wifi_country_t *country);

/**
  * @brief     get the current country info
  *
  * @param     country  country info
  *
  * @return
  *    - kNoErr: succeed
  *    - kNotInitializedErr: WiFi is not initialized
  *    - kParamErr: invalid argument
  */
OSStatus bk_wlan_get_country(wifi_country_t *country);


/** @brief  Connect or establish a Wi-Fi network in normal mode (station or soft ap mode).
 *
 *  @attention 
 *          1. This function can establish a Wi-Fi connection as a station or create
 *          a soft AP that other staions can connect (2 stations Max). 
 *          2. In station mode,_BK_ first scan all of the supported Wi-Fi channels to find a wlan that
 *          matchs the input SSID, and read the security mode. Then try to connect to the target wlan. 
 *          If any error occurs in the connection procedure or disconnected after a successful connection, 
 *          _BK_ start the reconnection procedure in backgound after a time interval defined in inNetworkInitPara.
 *          3. Call this function twice when setup coexistence mode (staion + soft ap).
 *          This function retruns immediately in station mode, and the connection will be executed in background.
 *
 * 
 * 
 * User example:
 * @code
 *   //start a soft_AP 
 *    network_InitTypeDef_st wNetConfig;
 *    os_memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));
 *    ssid_len = os_strlen(ap_ssid);
 *    key_len = os_strlen(ap_key);
 *    os_strcpy((char *)wNetConfig.wifi_ssid, ap_ssid);
 *    os_strcpy((char *)wNetConfig.wifi_key, ap_key);
 *    wNetConfig.wifi_mode = BK_SOFT_AP;
 *    wNetConfig.dhcp_mode = DHCP_SERVER;
 *    wNetConfig.wifi_retry_interval = 100;
 *    os_strcpy((char *)wNetConfig.local_ip_addr, WLAN_DEFAULT_IP);
 *    os_strcpy((char *)wNetConfig.net_mask, WLAN_DEFAULT_MASK);
 *    os_strcpy((char *)wNetConfig.gateway_ip_addr, WLAN_DEFAULT_GW);
 *    os_strcpy((char *)wNetConfig.dns_server_ip_addr, WLAN_DEFAULT_GW);
 *    bk_wlan_ap_set_default_channel(channel);
 *    bk_printf("ssid:%s  key:%s\r\n", wNetConfig.wifi_ssid, wNetConfig.wifi_key);
 *    bk_wlan_start(&wNetConfig);
 *
 *    //start a STA mode,connect the targe AP
 *    network_InitTypeDef_st wNetConfig;
 *    int ssid_len, key_len;
 *    os_memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));
 *    ssid_len = os_strlen(oob_ssid);
 *    key_len = os_strlen(connect_key);
 *    os_strlcpy((char *)wNetConfig.wifi_ssid, oob_ssid, sizeof(wNetConfig.wifi_ssid));
 *    os_strlcpy((char *)wNetConfig.wifi_key, connect_key, sizeof(wNetConfig.wifi_key));
 *    wNetConfig.wifi_mode = BK_STATION;
 *    wNetConfig.dhcp_mode = DHCP_CLIENT;
 *    wNetConfig.wifi_retry_interval = 100;
 *    bk_printf("ssid:%s key:%s\r\n", wNetConfig.wifi_ssid, wNetConfig.wifi_key);
 *    bk_wlan_start(&wNetConfig);
 * @endcode
 *  @param  inNetworkInitPara: Specifies wlan parameters.
 *
 *  @return
 *    - kNoErr :succeed
 *    - others: other errors.
 */
OSStatus bk_wlan_start(network_InitTypeDef_st* inNetworkInitPara);



/** @brief  disconect and stop WiFi in sta/ap mode
 *
 *
 *  @param
 *      - mode: Specifies wlan interface.
 *             @arg Soft_AP: soft-ap mode
 *             @arg Station: sta mode
 *
 *  @return
 *    - kNoErr :succeed
 *    - others: other errors.
 */
int bk_wlan_stop(char mode);


/** @brief  Connect to a Wi-Fi network with advantage settings (station mode only)
 *
 *  @attention 
 *          1. This function can connect to an access point with precise settings,
 *          that greatly speed up the connection if the input settings are correct
 *          and fixed. If this fast connection is failed for some reason, _BK_
 *          change back to normal: scan + connect mode refer to @ref bkWlanStart.
 *          This function returns after the fast connection try.
 *
 *          2. This function cannot establish a soft ap, use StartNetwork() for this
 *          purpose.
 *          If input SSID length is 0, _BK_ use BSSID to connect the target wlan.
 *          If both SSID and BSSID are all wrong, the connection will be failed.
 * User example:
 * @code 
 *  network_InitTypeDef_adv_st  wNetConfigAdv;
 *   os_memset( &wNetConfigAdv, 0x0, sizeof(network_InitTypeDef_adv_st) );
 *   os_strcpy((char*)wNetConfigAdv.ap_info.ssid, oob_ssid);
 *   hwaddr_aton("48:ee:0c:48:93:12", (u8 *)(wNetConfigAdv.ap_info.bssid));
 *   wNetConfigAdv.ap_info.security = BK_SECURITY_TYPE_WPA2_MIXED;
 *   wNetConfigAdv.ap_info.channel = 11;
 *   os_strcpy((char*)wNetConfigAdv.key, connect_key);
 *   wNetConfigAdv.key_len = os_strlen(connect_key);
 *   wNetConfigAdv.dhcp_mode = DHCP_CLIENT;
 *   wNetConfigAdv.wifi_retry_interval = 100;
 *   bk_wlan_start_sta_adv(&wNetConfigAdv);
 * @endcode
 *  @param  inNetworkInitParaAdv: Specifies the precise wlan parameters.
 *
 *  @return
 *    - kNoErr :succeed
 *    - others: other errors.
 */
OSStatus bk_wlan_start_sta_adv(network_InitTypeDef_adv_st* inNetworkInitParaAdv);


/** @brief  Read current IP status on a network interface.
 *
 *  @param
 *      - outNetpara: Point to the buffer to store the IP address.
 *      - inInterface: Specifies wlan interface.
 *             @arg Soft_AP: The soft AP that established by bk_wlan_start()
 *             @arg Station: The interface that connected to an access point
 *
 *  @return
 *    - kNoErr :succeed
 *    - others: other errors.
 */
OSStatus bk_wlan_get_ip_status(IPStatusTypedef *outNetpara, WiFi_Interface inInterface);


/** @brief  Read current wireless link status on station interface.
 *
 *  @param  outStatus: Point to the buffer to store the link status.
 *
 *  @return
 *      - kNoErr        : on success.
 *      - kGeneralErr   : if an error occurred
 */
OSStatus bk_wlan_get_link_status(LinkStatusTypeDef *outStatus);

/**
 * @brief Get the information of connected stations
 * @param stas Pointer to the stations information
 * @return 0 on success, -1 on failure
 */
int wlan_ap_sta_info(wlan_ap_stas_t *stas);

/** @brief  Read current wireless link status on soft-AP interface.
 *
 *  @param  ap_info: Point to the buffer to AP info.
 *
 *  @return
 *      - kNoErr        : on success.
 *      - kGeneralErr   : if an error occurred
 */
void bk_wlan_ap_para_info_get(network_InitTypeDef_ap_st *ap_info);


/** @brief  whether the AP status is power on
 *
 *  @param  void
 *
 *  @return
 *      - 0      : AP is working.
 *      - others : AP is not started.
 */
OSStatus bk_wlan_ap_is_up(void);

/** @brief  as sta mode,whether the status is connected to the AP
 *
 *  @param  void
 *
 *  @return
 *      - 0      : AP is working.
 *      - others : AP is not started.
 */
OSStatus bk_wlan_sta_is_connected(void);

/** @brief  in soft-ap mode,set the channel for soft-ap
 *
 *  @param  channel the channel for soft-ap
 *
 *  @return
 *      - void
 */
void bk_wlan_ap_set_default_channel(uint8_t channel);


/**
 * @brief     Register wifi scan event notification callback
 *
 * @param
 *    - ind_cb: scan done event callback
 *
 * @attention 
 *  1. you must regist it before bk_wlan_start_scan, otherwise you cant get scan done event
 *
 * @return
 *    - void
 */
void bk_wlan_scan_ap_reg_cb(FUNC_2PARAM_PTR ind_cb);


/**
 * @brief     ger wifi scan results
 *
 * @param
 *    - results: scan results
 *
 * @attention 
 *  1. you must use it after  scan done event, otherwise you will get nothing
 *
 * @return
 *    - void
 */
int wlan_sta_scan_result(ScanResult_adv *results);


/** @brief  Start a wlan scanning in 2.4GHz in _BK_ backfround.
 *
 *  @attention 
 *          Once the scan is completed, _BK_ sends a notify with callback function:
 *  
 *          void (*function)((void *ctxt, uint8_t param))
 *          Register callback function using @ref mhdr_scanu_reg_cb(scan_cb, 0) before scan.
 *
 * User example:
 * @code
 * static const char *crypto_str[] = {
 *    "None",
 *    "WEP",
 *    "WPA_TKIP",
 *    "WPA_AES",
 *    "WPA_MIXED",
 *    "WPA2_TKIP",
 *    "WPA2_AES",
 *    "WPA2_MIXED",   
 *    "WPA3_SAE",        
 *    "WPA3_WPA2_MIXED", 
 *    "EAP",
 *    "OWE",
 *    "AUTO",
 *};
 * static void scan_cb(void *ctxt, uint8_t param)
 * {
 *     ScanResult_adv apList;
 *     if (bk_wlan_ap_is_up() > 0)
 *         ret = wlan_ap_scan_result(&apList);
 *     else
 *         ret = wlan_sta_scan_result(&apList);
 *
 *     if (!ret) {
 *         int ap_num = apList.ApNum;
 *         int i;
 *
 *         bk_printf("Got ap count: %d\r\n", apList.ApNum);
 *         for (i = 0; i < ap_num; i++)
 *             bk_printf("    \"%s\", %02x:%02x:%02x:%02x:%02x:%02x, %d, %s, %d\n",
 *                     apList.ApList[i].ssid, MAC2STR(apList.ApList[i].bssid),
 *                     apList.ApList[i].ApPower, crypto_str[apList.ApList[i].security],
 *                     apList.ApList[i].channel);
 *         os_free(apList.ApList);
 *     }
 * }
 *  
 * bk_wlan_scan_ap_reg_cb(scan_cb);
 * bk_wlan_start_scan();
 * @endcode
 *  @param
 *      - void
 *
 *  @return  
 *      - void
 */
void bk_wlan_start_scan(void);

/** @brief  Start a wlan scanning ,which scan the target AP
 *  @param
 *      - ssid_ary Point to the ssids list you want to scan
 *      - ssid_num number of ssids 
 *  @return  
 *      - void
 */
void bk_wlan_start_assign_scan(UINT8 **ssid_ary, UINT8 ssid_num);

/** @brief  set filter flag in monitor mode
 *
 *  @param
        - filter  
            @arg bit0:1:mulicast_brdcast is not filte
            @arg bit1:1: duplicate frame is not filte
 *  @return  
 *      - void
**/
void bk_wlan_set_monitor_filter(unsigned char filter);



/**
 * @brief     Register wifi monitor event notification callback
 *
 * @param
 *    - fn: monitor data event callback
 *
 * @attention 
 *  1. you must regist it before bk_wlan_start_monitor, otherwise you cant get monitor frame
 *
 * @return
 *    - void
 */
void bk_wlan_register_monitor_cb(monitor_cb_t fn);

/** @brief  Start wifi monitor mode
 *
 *  @attention 
 *     1. This function disconnect wifi station,can coexit with AP mode
 *
* User example:
 * @code
 *   void bk_demo_monitor_cb(uint8_t *data, int len, wifi_link_info_t *info)
 *   {
 *      //the frame data of monitor
 *   }
 *
 *   bk_wlan_register_monitor_cb(bk_demo_monitor_cb);
 *   bk_wlan_start_monitor();
 * @endcode
 *
 *  @param
 *      - void     
 *  @return  
 *      - void
 */
int bk_wlan_start_monitor(void);


/** @brief  Stop wifi monitor mode
 *
 *
 *  @param
 *      - void
 *
 *  @return  
 *      - void
 */
int bk_wlan_stop_monitor(void);


/** @brief  Set channel at the asynchronous method
 *
 *  @attention 
 *       1. This function change the monitor channel (from 1~13).
 *       2. it can change the channel dynamically, don't need restart monitor mode.
 *  @param  channel set the monitor channel
 *  @return
 *      - 0      : success
 *      - others : failed.
 */
int bk_wlan_set_channel(int channel);


/** @brief  whether the monitor mode is power on
 *
 *  @param  void
 *
 *  @return
 *      - 0      : monitor is working.
 *      - others : monitor is not started.
 */
int bk_wlan_is_monitor_mode(void);


/** @brief Send raw 802.11 frame
 *
 * @attention 
 *     1. This API can be used in WiFi station, softap, or monitor mode.
 *     2. Only support to send non-QoS frame.
 *     3. The frame sequence will be overwritten by WiFi driver.
 *     4. The API doesn't check the correctness of the raw frame, the
 *               caller need to guarantee the correctness of the frame.
 *  @param
 *       - buffer : the buffer of tx frame
 *       - len    : len of frame data
 *  @return
 *      - 0      : success.
 *      - others : failed.
 * 
 */
int bk_wlan_send_80211_raw_frame(uint8_t *buffer, int len);


/** @brief Send raw 802.11 frame and register tx result notoify
 *
 * @attention 
 *     1. This API can be used in WiFi station, softap, or monitor mode.
 *     2. Only support to send non-QoS frame.
 *     3. The frame sequence will be overwritten by WiFi driver.
 *     4. after tx done,will notify the result of tx 
 *
 *  @param
 *       - buffer : the buffer of tx frame
 *       - len    : len of frame data
         - cb     : callback of tx done
         - param  : param of tx frame,normally set NULL
 *  @return
 *      - 0      : success.
 *      - others : failed.
 * 
 */
int bk_wlan_send_raw_frame_with_cb(uint8_t *buffer, int len, void *cb, void *param);


#ifdef __cplusplus
}
#endif

#endif// _WLAN_UI_PUB_

