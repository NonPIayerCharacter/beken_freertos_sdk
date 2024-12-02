#include "include.h"
#include "ble_api.h"

uint8_t ble_switch_mac_sleeped = 0, tx_pwr_idx = 0;

bool ble_coex_pta_is_on(void)
{
	return false;
}

void ble_switch_rf_to_wifi(void)
{
	return;
}

UINT32 ble_ctrl(UINT32 cmd, void* param)
{
	return ERR_CMD_NOT_SUPPORT;
}

UINT32 uart_debug_init(void)
{
	return 0;
}

UINT32 ble_in_dut_mode(void)
{
	return 0;
}