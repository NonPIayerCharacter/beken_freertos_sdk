#ifndef APP_FINDT_H_
#define APP_FINDT_H_

#include "rwprf_config.h"     // SW configuration

#if (BLE_FINDME_TARGET)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct app_subtask_handlers app_findt_table_handler;
extern void bk_findt_init(void);
#endif

#endif
