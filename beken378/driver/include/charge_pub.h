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

#ifndef _CHARGE_PUB_H_
#define _CHARGE_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#include "generic.h"
#include "drv_model_pub.h"

#define CHARGE_DEV_NAME     "charge"

#define CHARGE_CMD_MAGIC              (0x0)
enum
{
    CMD_CHG_FULL_OFFSET_SET = CHARGE_CMD_MAGIC + 1,
    CMD_CHG_CMP_SET,
    CMD_CHG_CC2CV_DELAY_SET,
    CMD_VUSB_DETECTOR_ENABLE,
    CMD_CHG_CURRENT_CONTROL,
    CMD_CHG_ICVEND_CONTROL,
    CMD_CHG_ITRICK_CONTROL,
    CMD_CHG_LDO_ENABLE,
    CMD_CHG_ENABLE,
    CMD_CHG_INT_SET,
    CMD_INT_STATE_GET
};


typedef enum {
    CHARGE_0_RECHARGE = 0,  //re_charge
    CHARGE_1_VCAL_IND,  //chg_vcal_ind
    CHARGE_2_TRICK,  //chg_trick
    CHARGE_3_TERMINAL,  //chg_terminal
    CHARGE_4_ICAL_IND,  //chg_ical_ind
    CHARGE_5_CV,  //chg_cv
    CHARGE_6_CC,  //chg_cc
    CHARGE_7_USB_READY = 7  //usb_ready
} CHARGE_INDEX;

typedef enum {
    CHARGE_INT_TYPE_L_LV = 0,
    CHARGE_INT_TYPE_H_LV,
    CHARGE_INT_TYPE_P_EDGE,
    CHARGE_INT_TYPE_N_EDGE = 3
} CHARGE_INT_TYPE;

typedef enum {
    CHARGE_INT_DIS,
    CHARGE_INT_EN
} CHARGE_INT_CTRL;

typedef enum {
    CHARGE_ITRICK_10PERCENT_CC = 0,
    CHARGE_ITRICK_20PERCENT_CC = 1
} CHARGE_TRICK_CUR;

typedef enum {
    CHARGE_ICVEND_5PERCENT_CC = 0,
    CHARGE_ICVEND_10PERCENT_CC = 1,
    CHARGE_ICVEND_15PERCENT_CC = 2,
    CHARGE_ICVEND_20PERCENT_CC = 3
} CHARGE_CVEND_CUR;

typedef struct charge_int_st
{
    CHARGE_INDEX int_index;
    CHARGE_INT_TYPE int_type;
    UINT8 enable;
} CHARGE_INT_ST, *CHARGE_INT_PTR;

extern void charge_init(void);
extern void charge_exit(void);
extern UINT32 charge_ctrl(UINT32 cmd, void *param);

#endif
#endif // _CHARGE_PUB_H_
// eof

