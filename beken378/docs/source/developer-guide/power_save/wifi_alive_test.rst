wifi 低压保活DTIM 休眠测试
=============================================

.. important::
   - 目前低压模式ble 无法唤醒工作，故进行wifi 保活测试时要先退出BLE 



wifi 保活睡眠代码调用流程
--------------------------------------------------------------------

- 1.当使用BLE后,需要把它关闭。

.. code-block:: c++

    bk_ble_disconnect(0,ble_cmd_cb);
    bk_ble_stop_advertising(0,ble_cmd_cb);
    ble_thread_exit()
  

- 2.设置DTIM count 和 HW TIM count

.. code-block:: c++

    power_save_set_listen_int(10);
    power_save_set_hw_tim_cnt_limit(5);

    // DTIM 10 : MCU 每10个beacon 周期唤醒起来接收beacon，进行保活
    // HW TIM 5: 以6次为一个周期，前5次唤醒只有硬件接收beacon，软件不处理beacon, 第6次唤醒后软件才会解析beacon 全部内容， 这个参数越大可以降低功耗，但是需要根据应用需求调整。


- 3.连接路由器

.. code-block:: c++

    demo_sta_app_init(ssid,password);
    // ssid     : 路由器名称
    // password : 路由器密码


- 4.进入mcu 和 rf dtim sleep

.. code-block:: c++

    bk_wlan_mcu_ps_mode_enable();
    bk_wlan_dtim_rf_ps_mode_enable();
    
    // 注意： 这两个接口需要在wifi已连接， 获取到IP 后调用，否则无法进入dtim 休眠



wifi 保活睡眠测试 cli 命令
--------------------------------------------------------------------

::
    
    //首先连接指定路由器、ssid为路由器名称，password 为路由器密码
    sta ssid password

    //等待连接上路由器后，再设置休眠

    //使能mcu 休眠
    ps mcuditm 1

    //设置仅硬件接收beacon tim 次数
    ps tim 5

    //设置listen interval 为10, 和使能rf ditm 休眠
    ps rfditm 1 10
