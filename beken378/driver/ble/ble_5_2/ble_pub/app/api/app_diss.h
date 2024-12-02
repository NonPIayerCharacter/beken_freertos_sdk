#ifndef APP_DISS_H_
#define APP_DISS_H_

#include "rwprf_config.h"     // SW configuration

#if (BLE_DIS_SERVER)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct app_subtask_handlers app_diss_table_handler;
extern ble_err_t bk_diss_set(uint8_t val_id,uint8_t len,uint8_t *data);
extern void bk_diss_init(void);
#endif

#endif

