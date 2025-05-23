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

