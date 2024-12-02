#ifndef __ATSVR_AIRKISS_CMD_H_
#define __ATSVR_AIRKISS_CMD_H_
#if CFG_USE_DISTRIBUTION_NETWORK
int stop_ble_config(void);
int start_ble_config(void);
void bk_store_ssid_toflash(char *ssid,char *pwd);
#endif
#endif