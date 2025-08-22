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

#ifndef _ATE_APP_H_
#define _ATE_APP_H_

#include "sys_config.h"

#define ATE_APP_FUN (CFG_TX_EVM_TEST || CFG_RX_SENSITIVITY_TEST)
#define ATE_ENABLE_GIPO_LEVEL  0

#if ATE_APP_FUN
#include "gpio_pub.h"
#include "uart_pub.h"

#define ATE_DEBUG
#ifdef ATE_DEBUG
#define ATE_PRT      os_printf
#define ATE_WARN     warning_prf
#define ATE_FATAL    fatal_prf
#else
#define ATE_PRT      null_prf
#define ATE_WARN     null_prf
#define ATE_FATAL    null_prf
#endif

extern int ate_gpio_port;
extern void ate_gpio_init(void);
extern uint32_t ate_mode_check(void);
extern void ate_app_init(void);
extern void ate_start(void);
#endif /*ATE_APP_FUN */
extern uint32_t get_ate_mode_state(void);
#endif // _ATE_APP_H_
// eof

