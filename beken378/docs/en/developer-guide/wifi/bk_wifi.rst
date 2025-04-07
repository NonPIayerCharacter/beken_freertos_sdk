
:link_to_translation:`zh_CN:[中文]`

Wi-Fi description
=======================================================

Wi-Fi features
-------------------------------------------------------

- 2.4 GHz IEEE 802.11b/g/n compliant
- Supports HT20 
- Supports 802.11n MCS0-7
- Supports STA, AP, and Direct Modes
- Supports concurrent AP + STA
- Supports WPA, WPA2, and WPA3
- Supports AMPDU and QoS
- Supports low-power DTIM sleep in STA mode


Description of Wi-Fi callback events
-------------------------------------------------------

.. code-block:: c++

    static void app_demo_sta_rw_event_func(void *new_evt)
    {
        rw_evt_type notice_event = *((rw_evt_type *)new_evt);

        switch(notice_event){
            case RW_EVT_STA_CONNECTED:
                bk_printf("conmected AP\r\n");
                break;

            case RW_EVT_STA_GOT_IP:
                bk_printf("got ip success\r\n");
                break;

            case RW_EVT_STA_DISCONNECTED:
            case RW_EVT_STA_BEACON_LOSE:
                bk_printf("wifi had disconnected\r\n");
                break;

            case RW_EVT_STA_NO_AP_FOUND:
            case RW_EVT_STA_PASSWORD_WRONG:
            case RW_EVT_STA_ASSOC_FULL:
            case RW_EVT_STA_CONNECT_FAILED:
                bk_printf("wifi connect failed\r\n");
                break;

            case RW_EVT_AP_CONNECTED:
                bk_printf("a station had connected\r\n");
                break;
            case RW_EVT_AP_DISCONNECTED:
                bk_printf("a station had disconnected\r\n");
                break;
            case RW_EVT_AP_CONNECT_FAILED:
                bk_printf("a station connect failed\r\n");
                break;
            default:
                break;
        }
    }

    bk_wlan_status_register_cb(app_demo_sta_rw_event_func)

As in the above example, Wi-Fi initialization requires registering event callbacks.

- RW_EVT_STA_CONNECTED

The Wi-Fi connection success event only means that the Wi-Fi four-way handshake is successful. It also needs to start the DHCP service to obtain an IP address or use a static IP address. Therefore, the user cannot start network operations at this event.


- RW_EVT_STA_GOT_IP

STA has successfully connected and obtained the IP address through DHCP. After receiving this event, the user can perform network interface operations.


- RW_EVT_STA_DISCONNECTED
- RW_EVT_STA_BEACON_LOSE

STA and AP disconnect event. After receiving this event, it is generally necessary to notify the client application written based on sockets to close the socket. If this event is not what the user expects, the reconnection process can be started.


- RW_EVT_STA_NO_AP_FOUND 
- RW_EVT_STA_PASSWORD_WRONG 
- RW_EVT_STA_ASSOC_FULL 
- RW_EVT_STA_CONNECT_FAILED

These events are notifications of connection failure. The user can obtain the reason for the connection failure and then reconnect.




