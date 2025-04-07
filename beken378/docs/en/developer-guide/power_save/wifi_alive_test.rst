
:link_to_translation:`zh_CN:[中文]`

Wi-Fi low voltage keep alive DTIM sleep test
=============================================

.. important::
   - Currently, Bluetooth LE cannot be woken up in low-voltage mode, so you need to exit Bluetooth LE first when performing a wifi keep alive test.




Wi-Fi keep alive sleep code calling process
--------------------------------------------------------------------

- 1.After using Bluetooth LE, you need to turn it off.

.. code-block:: c++

    bk_ble_disconnect(0,ble_cmd_cb);
    bk_ble_stop_advertising(0,ble_cmd_cb);
    ble_thread_exit()
  

- 2.Set DTIM count and HW TIM countt

.. code-block:: c++

    power_save_set_listen_int(10);
    power_save_set_hw_tim_cnt_limit(5);

    // DTIM 10: MCU wakes up every 10 beacon intervals to receive beacon and keep alive
    // HW TIM 5: Taking 6 times as a cycle, in the first 5 wake-ups, only the hardware receives the beacon, and the software does not process the beacon. After the 6th wake-up, the software will parse the entire content of the beacon. The larger this parameter is, the lower the power consumption will be. The parameter value needs to be adjusted based on application requirements.


- 3.Connect to the router

.. code-block:: c++

    demo_sta_app_init(ssid,password);
    // ssid: router name
    // password: router password


- 4.Enter MCU and RF DTIM sleep

.. code-block:: c++

    bk_wlan_mcu_ps_mode_enable();
    bk_wlan_dtim_rf_ps_mode_enable();
    
    // Note: These two APIs need to be called after Wi-Fi is connected and the IP is obtained, otherwise DTIM sleep cannot be entered.



Wi-Fi keep alive sleep test CLI commands
--------------------------------------------------------------------

::
    
    //First connect to the specified router, SSID is the router name, password is the router password.
    sta ssid password

    //Set sleep after connecting to the router.

    //Enable MCU sleep
    ps mcudtim 1

    //Set only the hardware to receive beacon tim times.
    ps tim 5

    //Set the listen interval to 10, and enable RF DTIM sleep.
    ps rfdtim 1 10
