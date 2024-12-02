1, enable the macro CFG_ENABLE_DEMO_TEST at sys_config.h;
2, config demo item at demos_config.h;
3, set the macro CFG_BLE_ADV_NUM = 2 at sys_config.h;
4, replace cfg_flag to NULL at bk_ble_service_init();
5, compile and run;

thx

