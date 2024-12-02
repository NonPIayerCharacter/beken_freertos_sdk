#ifndef APP_BASS_H_
#define APP_BASS_H_

#include "rwprf_config.h"     // SW configuration

#if (BLE_BATT_SERVER)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct app_subtask_handlers app_bass_table_handler;
extern ble_err_t bk_bass_ntf_send(uint8_t level);
extern ble_err_t bk_bass_enable(uint8_t conidx);
extern void bk_bass_init(void);
#endif

#endif

