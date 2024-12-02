#ifndef APP_HOGPD_H_
#define APP_HOGPD_H_

#include "rwprf_config.h"     // SW configuration

#if (BLE_HID_DEVICE)
#include <stdint.h>          // Standard Integer Definition
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "kernel_task.h"         // Kernel Task Definition

extern const struct app_subtask_handlers app_hogpd_table_handler;
extern void bk_hogpd_enable(void);
extern void bk_hogpd_init(void);

#endif

#endif



