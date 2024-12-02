#ifndef __FORCE_PS_PUB_H__
#define __FORCE_PS_PUB_H__

#include "uart_pub.h"

#define FS_PRT                   os_null_printf
#define FS_WARN                  os_null_printf
#define FS_FATAL                 bk_printf

typedef struct mac_sleep_ctrl {
	uint8_t off_mac;
	uint8_t off_modem;
	uint16_t wakeup_mcu;
} MAC_S_CFG_ST, *MAC_S_CFG_PTR;

typedef struct mcu_sleep_ctrl {
	uint8_t sleep_mode;
	uint8_t off_26M;
	uint16_t off_ble;
	uint32_t wakeup_sig;
	uint64_t sleep_us;
} MCU_S_CFG_ST, *MCU_S_CFG_PTR;

typedef enum mcu_sleep_mode {
	MCU_NORMAL_SLEEP            =  0,
	MCU_LOW_VOLTAGE_SLEEP       =  1,
} MCU_S_MODE;

#endif // #ifndef __FORCE_PS_PUB_H__

