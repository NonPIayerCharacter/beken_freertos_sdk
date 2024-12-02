#ifndef __FORCE_MCU_PS_H__
#define __FORCE_MCU_PS_H__

#include "typedef.h"

UINT32 force_mcu_ps(UINT64 sleep_ms, UINT32 wake_up_bit);
UINT32 bk_force_instant_lowvol_sleep(PS_DEEP_CTRL_PARAM *lowvol_param);
#endif // __FORCE_MCU_PS_H__
