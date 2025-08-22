// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef USER_CONFIG_H_
#define USER_CONFIG_H_

#include "uart_pub.h"

#define VIRTUAL_UART_H4TL          1
#define UART_PRINTF_ENABLE         1
#define DEBUG_HW                   0
#define GPIO_DBG_MSG               0
#define DEBUG_RF_REG               0
#define LDO_MODE                   0

//DRIVER CONFIG
#define UART0_DRIVER				1
#define UART2_DRIVER				1

#define GPIO_DRIVER					0
#define ADC_DRIVER					0
#define I2C_DRIVER					0
#define PWM_DRIVER					0
#define USB_DRIVER                  0
#define SPI_DRIVER                  0
#define AON_RTC_DRIVER              1

#define uart_printf              bk_printf
#define UART_PRINTF              bk_printf

/// Default Device Name
#define APP_DFLT_DEVICE_NAME            ("BK7231N-BLE")
#define APP_DFLT_DEVICE_NAME_LEN        (sizeof(APP_DFLT_DEVICE_NAME))

/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MIN         (160 )
/// Advertising maximum interval - 40ms (64*0.625ms)
#define APP_ADV_INT_MAX         (160)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)


#define BLE_UAPDATA_MIN_INTVALUE		20

#define BLE_UAPDATA_MAX_INTVALUE		40

#define BLE_UAPDATA_LATENCY				0

#define BLE_UAPDATA_TIMEOUT				600

#endif // USER_CONFIG_H_


