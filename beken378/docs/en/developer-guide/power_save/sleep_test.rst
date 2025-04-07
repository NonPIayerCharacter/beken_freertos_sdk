
:link_to_translation:`zh_CN:[中文]`

=======================================================
Low power consumption test instructions
=======================================================

.. important::
   - The difference between low-voltage sleep and deep sleep:

    1. State after waking up: The system will be reset after waking up from deep sleep, and the system will continue running where it was suspended after waking up from low-voltage sleep.
    2. Current consumption: Deep sleep current is 10 μA, low-voltage sleep current is 75 μA.



1. GPIO as wake-up source
=======================================================


1.1. Call instructions for entering deep sleep
--------------------------------------------------------------------

- Set GPIO as wake-up source

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_GPIO;


- Set GPIO20 as wake-up pin
 
.. code-block:: c++

    deep_sleep_param.gpio_index_map          = 0x1 << GPIO20;


- Set wake up on rising or falling edge for GPIO20

.. code-block:: c++

    deep_sleep_param.gpio_edge_map           = 0x1 << GPIO20;  //Wake up on falling edge

    deep_sleep_param.gpio_edge_map           = 0x0 << GPIO20;  //wake up on rising edge


- Set to maintain GPIO21 voltage during deep sleep

.. code-block:: c++

     deep_sleep_param.gpio_stay_lo_map       =  0x1 << GPIO21;



- For other parameters that are not used, the default value is 0.

.. code-block:: c++

    //These parameters are used for the configuration of GPIO32-GPIO39. The BK7238 does not have these pins, so there is no need to set them.
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;

    //These parameters are used for RTC as wake-up source and do not need to be set.
    deep_sleep_param.sleep_time             = 0;
    deep_sleep_param.lpo_32k_src            = 0;

- Call the deep sleep API and set the system to enter sleep.

.. code-block:: c++
    
    bk_enter_deep_sleep_mode(&deep_sleep_param);





1.2 Call instructions for entering low-voltage sleep
--------------------------------------------------------------------

- Set GPIO as wake-up source, and set the sleep mode to low-voltage sleep

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = 1;  //PS_DEEP_WAKEUP_GPIO;
    deep_sleep_param.sleep_mode             = 1;  //MCU_LOW_VOLTAGE_SLEEP


- Set GPIO20 as wake-up pin

.. code-block:: c++

    deep_sleep_param.gpio_index_map          = 0x1 << GPIO20;   //0x1 << 20 = 0x100000


- Set wake up on rising or falling edge for GPIO20

.. code-block:: c++

    deep_sleep_param.gpio_edge_map           = 0x1 << GPIO20;  //0x1 << 20 = 0x100000 wake up on falling edge

    deep_sleep_param.gpio_edge_map           = 0x0 << GPIO20;  //wake up on rising edge

     
- For other parameters that are not used, the default value is 0.

.. code-block:: c++

    //These parameters are used for the configuration of GPIO32-GPIO39. BK7238 does not have these pins, so there is no need to set them.
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;

    deep_sleep_param.gpio_stay_lo_map       = 0;

    //These parameters are used for RTC as the wake-up source and do not need to be set.
    deep_sleep_param.sleep_time             = 0;
    deep_sleep_param.lpo_32k_src            = 0;

- Call the low-voltage sleep API and set the system to enter sleep.

.. code-block:: c++
    
    bk_wlan_instant_lowvol_sleep(&deep_sleep_param);
    
    
    
    
    
    
2. RTC as wake-up source
=======================================================

2.1. Call instructions for entering deep sleep 
--------------------------------------------------------------------

- Set RTC as wake-up source

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_RTC;


- Set RTC sleep time

.. code-block:: c++

    deep_sleep_param.sleep_time             = 5;     //5s
    deep_sleep_param.lpo_32k_src            = 0;     //LPO_SELECT_ROSC



- For other parameters that are not used, the default value is 0.

.. code-block:: c++

    //These parameters are used for GPIO as wake-up source and do not need to be set.
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_stay_lo_map       = 0;
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;


- Call the deep sleep API and set the syetem to enter sleep.

.. code-block:: c++
    
    bk_enter_deep_sleep_mode(&deep_sleep_param);




2.2. Call instructions for entering low-voltage sleep
--------------------------------------------------------------------

- Set RTC as wake-up source and set the sleep mode to low-voltage sleep mode

.. code-block:: c++
    
    PS_DEEP_CTRL_PARAM deep_sleep_param;
    deep_sleep_param.wake_up_way            = PS_DEEP_WAKEUP_RTC;
    deep_sleep_param.sleep_mode             = 1;  //MCU_LOW_VOLTAGE_SLEEP


- Set RTC sleep time

.. code-block:: c++

    deep_sleep_param.sleep_time             = 5;     //5s
    deep_sleep_param.lpo_32k_src            = 0;     //LPO_SELECT_ROSC



- For other parameters that are not used, the default value is 0.

.. code-block:: c++

    //These parameters are used for GPIO as wake-up source and do not need to be set.
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_index_map         = 0;
    deep_sleep_param.gpio_stay_lo_map       = 0;
    deep_sleep_param.gpio_last_index_map    = 0;
    deep_sleep_param.gpio_last_edge_map     = 0;
    deep_sleep_param.gpio_stay_hi_map       = 0;




- Call the deep sleep API and set the system to enter sleep.

.. code-block:: c++
    
    bk_wlan_instant_lowvol_sleep(&deep_sleep_param);
