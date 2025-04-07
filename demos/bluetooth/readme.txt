eddystone demo:
1, enable the macro CFG_ENABLE_DEMO_TEST at sys_config.h;
2, config demo item at demos_config.h;
3, set the macro CFG_BLE_ADV_NUM = 2 at sys_config.h;
4, replace cfg_flag to NULL at bk_ble_service_init();
5, compile and run;

throughput demo:
1. sys_config_bk7238.h: #define CFG_ENABLE_DEMO_TEST 1;
2. app.c: remove line if(application_start);
3. demos_config.h: #define THROUGHPUT_DEMO 1;
4. ble_main.c: ble_sleep_enable = 0;
5. sdp_common.c: remove print in sdp_write_cmp_cb();
6. app_ble_init.h: #define APP_INIT_SET_STOP_CONN_TIMER 0;
7. compile bk7238 freertos
