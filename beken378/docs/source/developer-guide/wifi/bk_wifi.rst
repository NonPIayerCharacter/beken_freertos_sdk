Wi-Fi说明
=======================================================

Wi-Fi功能列表
-------------------------------------------------------

- 兼容IEEE802.11 b/g/n 2.4GHz标准
- 支持HT20 
- 支持802.11N MCS0-7
- 支持STA、AP and Direct Modes
- 支持Concurrent AP+STA
- 支持WPA、WPA2及WPA3等加密方式
- 支持AMPDU、QoS
- Station模式下支持DTIM低功耗休眠


Wi-Fi callback 事件描述
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

如上面例子，wifi 初始化需要注册下事件回调，

- RW_EVT_STA_CONNECTED

wifi 连接成功事件，仅是wifi 四次握手成功，还需要启动动DHCP服务获取IP地址或者使用静态IP地址,故用户不能在此事件就开始进行网络操作。


- RW_EVT_STA_GOT_IP

STA已经成功连接并且DHCP 获取到IP 地址,接收到此事件后,用户可进行网络接口操作。


- RW_EVT_STA_DISCONNECTED
- RW_EVT_STA_BEACON_LOSE

STA与AP断开事件,接收到此事件后,一般需要通知针对基于套接字编写的客户端应用程序关闭套接字.如果此事件不是用户希望的,可以启动重连流程.


- RW_EVT_STA_NO_AP_FOUND 
- RW_EVT_STA_PASSWORD_WRONG 
- RW_EVT_STA_ASSOC_FULL 
- RW_EVT_STA_CONNECT_FAILED

这几个事件都是连接失败的通知，用户可获取连接失败原因，然后进行重连




