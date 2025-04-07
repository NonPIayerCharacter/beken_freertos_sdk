#include "include.h"

uint8_t ble_switch_mac_sleeped = 0, tx_pwr_idx = 0;
uint8_t __attribute__((weak)) ble_active = 0;

bool __attribute__((weak)) ble_coex_pta_is_on(void)
{
	return false;
}

void __attribute__((weak)) ble_switch_rf_to_wifi(void)
{
	return;
}

UINT32 __attribute__((weak)) ble_ctrl(UINT32 cmd, void* param)
{
	return 18;
}

UINT32 __attribute__((weak)) uart_debug_init(void)
{
	return 0;
}

UINT32 __attribute__((weak)) ble_in_dut_mode(void)
{
	return 0;
}

void __attribute__((weak)) turnon_PA_in_temp_dect(void)
{

}

uint8_t __attribute__((weak)) if_ble_sleep(void)
{
	return 0;
}

void __attribute__((weak)) temp_detect_change_configuration(UINT32 intval, UINT32 thre, UINT32 dist)
{

}
