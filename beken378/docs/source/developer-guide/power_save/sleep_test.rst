=======================================================
低功耗测试说明
=======================================================

.. important::
   - 低压休眠和深度休眠的区别：

    1. 唤醒后的状态： 深度睡眠唤醒后系统将会复位，低压睡眠唤醒后系统将在挂起处继续运行。
    2. 睡眠电流功耗： 深度睡眠电流在10UA,低压睡眠电流在75uA。



1. GPIO作为唤醒源
=======================================================


1.1. 系统进入深度睡眠调用说明
--------------------------------------------------------------------

- 设置 GPIO 作为唤醒源

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_GPIO;


- 2 设置 GPIO20 为唤醒管脚
 
.. code-block:: c++

    deep_sleep_param.gpio_index_map          = 0x1 << GPIO20;


- 设置 GPIO20 上升沿、下降沿唤醒方式

.. code-block:: c++

    deep_sleep_param.gpio_edge_map           = 0x1 << GPIO20;  //下降沿唤醒

    deep_sleep_param.gpio_edge_map           = 0x0 << GPIO20;  //上升沿唤醒


- 设置 GPIO21 在睡眠期间电压保持

.. code-block:: c++

     deep_sleep_param.gpio_stay_lo_map       =  0x1 << GPIO21;



- 其他参数未使用的，默认赋值0

.. code-block:: c++

    //这几个参数是GPIO32-GPIO39 的配置，BK7238 没有这些管脚，故不需要设置
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;

    //这些参数是RTC 作为唤醒源的，不需要设置
    deep_sleep_param.sleep_time             = 0;
    deep_sleep_param.lpo_32k_src            = 0;

- 6 开始调用深度睡眠API,进入睡眠

.. code-block:: c++
    
    bk_enter_deep_sleep_mode(&deep_sleep_param);





1.2 系统进入低压睡眠调用说明
--------------------------------------------------------------------

- 设置 GPIO 作为唤醒源 ,并且设置睡眠模式为低压睡眠

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = 1;  //PS_DEEP_WAKEUP_GPIO;
    deep_sleep_param.sleep_mode             = 1;  //MCU_LOW_VOLTAGE_SLEEP


- 设置 GPIO20 为唤醒管脚

.. code-block:: c++

    deep_sleep_param.gpio_index_map          = 0x1 << GPIO20;   //0x1 << 20 = 0x100000


- 设置 GPIO20 上升沿、下降沿唤醒方式

.. code-block:: c++

    deep_sleep_param.gpio_edge_map           = 0x1 << GPIO20;  //0x1 << 20 = 0x100000 下降沿唤醒

    deep_sleep_param.gpio_edge_map           = 0x0 << GPIO20;  //上升沿唤醒

     
- 其他参数未使用的，默认赋值0

.. code-block:: c++

    //这几个参数是GPIO32-GPIO39 的配置，BK7238 没有这些管脚，故不需要设置
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;

    deep_sleep_param.gpio_stay_lo_map       = 0;

    //这些参数是RTC 作为唤醒源的，不需要设置
    deep_sleep_param.sleep_time             = 0;
    deep_sleep_param.lpo_32k_src            = 0;

- 5 开始调用低压睡眠API,进入睡眠

.. code-block:: c++
    
    bk_wlan_instant_lowvol_sleep(&deep_sleep_param);
    
    
    
    
    
    
2. RTC作为唤醒源
=======================================================

2.1. 系统进入深度睡眠调用说明
--------------------------------------------------------------------

- 设置 RTC 作为唤醒源

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_RTC;


- 设置 RTC 睡眠时间

.. code-block:: c++

    deep_sleep_param.sleep_time             = 5;     //5s
    deep_sleep_param.lpo_32k_src            = 0;     //LPO_SELECT_ROSC



- 其他参数未使用的，默认赋值0

.. code-block:: c++

    //这些参数是GPIO 作为唤醒源的，不需要设置
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_stay_lo_map       = 0;
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;


- 开始调用深度睡眠API,进入睡眠

.. code-block:: c++
    
    bk_enter_deep_sleep_mode(&deep_sleep_param);




2.2. 系统进入低压睡眠调用说明
--------------------------------------------------------------------

- 设置 RTC 作为唤醒源，并设置为低压睡眠模式

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_RTC;
    deep_sleep_param.sleep_mode             = 1;  //MCU_LOW_VOLTAGE_SLEEP


- 设置 RTC 睡眠时间

.. code-block:: c++

    deep_sleep_param.sleep_time             = 5;     //5s
    deep_sleep_param.lpo_32k_src            = 0;     //LPO_SELECT_ROSC



- 其他参数未使用的，默认赋值0

.. code-block:: c++

    //这些参数是GPIO 作为唤醒源的，不需要设置
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_stay_lo_map       = 0;
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;




- 开始调用深度睡眠API,进入睡眠

.. code-block:: c++
    
    bk_wlan_instant_lowvol_sleep(&deep_sleep_param);
