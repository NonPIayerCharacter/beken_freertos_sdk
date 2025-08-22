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

#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "charge_pub.h"
#include "charge.h"
#include "target_util_pub.h"
#include "sys_ctrl.h"
#include "sys_ctrl_pub.h"
#include "intc_pub.h"

#if ((CFG_USE_CHARGE_DEV == 1) && (CFG_SOC_NAME == SOC_BK7252N))

static SDD_OPERATIONS charge_op = {
    charge_ctrl
};

static void charge_cfg_clear_value(void)
{
    REG_WRITE(CHARGE_REG0X0, 0);
    REG_WRITE(SCTRL_ANALOG_CTRL7, 0);
}

__maybe_unused static void charge_int_enable(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    REG_WRITE(CHARGE_REG0X0, CHARGE_REG0X0_CHARGE_X_INT_EN(index));
}

__maybe_unused static void charge_int_disable(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    REG_WRITE(CHARGE_REG0X0, CHARGE_REG0X0_CHARGE_X_INT_DIS(index));
}

__maybe_unused static uint32_t charge_int_en_get(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    return CHARGE_REG0X0_CHARGE_X_INT_GET(index);
}

__maybe_unused static void charge_int_type_set(uint32_t index, uint32_t mode)
{
    index=(CHARGE_INDEX)(index);
    mode=(CHARGE_INT_TYPE)(mode);
    REG_WRITE(CHARGE_REG0X1, CHARGE_REG0X1_CHARGE_X_INT_SET(index,mode));
}

__maybe_unused static uint32_t charge_int_type_get(uint32_t index)
{
    CHARGE_INT_TYPE mode;
    index=(CHARGE_INDEX)(index);
    mode=(CHARGE_INT_TYPE)(CHARGE_REG0X1_CHARGE_X_INT_GET(index));
    return mode;
}

__maybe_unused static uint32_t charge_state_get(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    return CHARGE_REG0X2_CHARGE_X_STA_GET(index);
}

__maybe_unused static uint32_t charge_int_state_get(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    return CHARGE_REG0X2_CHARGE_X_INT_STA_GET(index);
}

static void charge_int_state_clear(uint32_t index)
{
    index=(CHARGE_INDEX)(index);
    REG_WRITE(CHARGE_REG0X2, CHARGE_REG0X2_CHARGE_X_INT_STA_CLR(index));
}

static void charge_int_set(void *param)
{
    CHARGE_INT_PTR chg_int;
    chg_int = (CHARGE_INT_PTR)param;
    if(chg_int)
    {
        charge_int_type_set(chg_int->int_index, chg_int->int_type);
        if(!!chg_int->enable)
        {
            charge_int_enable(chg_int->int_index);
        }
        else
        {
            charge_int_disable(chg_int->int_index);
        }
    }
}

/* To increase charge_full volt upto 4.2V~4.4V, for different types of batteries */
static void charge_full_offset_set(uint32_t vsel)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value &= ~(CHG_VOUT_SELECT_MASK << CHG_VOUT_SELECT_POS);
    value |= ((vsel & CHG_VOUT_SELECT_MASK) << CHG_VOUT_SELECT_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

/* To extend CC range, in order to reduce charging time, for those batteries with high internal resistance */
static void charge_cc_compensation_set(uint32_t vcomp)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value &= ~(CHG_VOUT_COMPENSATION_MASK << CHG_VOUT_COMPENSATION_POS);
    value |= ((vcomp & CHG_VOUT_COMPENSATION_MASK) << CHG_VOUT_COMPENSATION_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

/* To avoid false charge_full interrupt, default value: 2 */
static void charge_cc2cv_delay_set(uint32_t delay)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value &= ~(1 << CHG_CC_TOCV_DELAY_ENABLE);
    value |= (!!delay << CHG_CC_TOCV_DELAY_ENABLE);
    value &= ~(CHG_CC_TO_CV_DELAY_TIME_MASK << CHG_CC_TO_CV_DELAY_TIME_POS);
    value |= ((delay & CHG_CC_TO_CV_DELAY_TIME_MASK) << CHG_CC_TO_CV_DELAY_TIME_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

/* Enable VUSB plugin detect */
static void vusb_detector_enable(uint32_t enable)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value &= ~(1 << CHG_VUSB_DETECTOR_ENABLE_POS);
    value |= (!!enable << CHG_VUSB_DETECTOR_ENABLE_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

/* Charge Constant Current = icp * 1.5 */
static void charge_current_control(uint32_t icp)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~(CHG_CURRENT_CONTROL_MASK << CHG_CURRENT_CONTROL_POS);
    value |= ((icp & CHG_CURRENT_CONTROL_MASK) << CHG_CURRENT_CONTROL_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

/*
To select Charge End Current
Icvend 0 = 5% Constant Current
Icvend 1 = 10% Constant Current
Icvend 2 = 15% Constant Current
Icvend 3 = 20% Constant Current
*/
static void charge_cvend_current_control(CHARGE_CVEND_CUR Icvend)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~(CHG_CV_END_VOLTAGE_CONTROL_MASK << CHG_CV_END_VOLTAGE_CONTROL_POS);
    value |= ((Icvend & CHG_CV_END_VOLTAGE_CONTROL_MASK) << CHG_CV_END_VOLTAGE_CONTROL_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

/*
To select charge trick current
Itrick 0 = 10% Constant Current
Itrick 1 = 20% Constant Current
*/
static void charge_trick_current_control(CHARGE_TRICK_CUR Itrick)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value &= ~(TRICK_CUR_MASK << TRICK_CUR_POS);
    value |= ((Itrick & TRICK_CUR_MASK) << TRICK_CUR_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

#define CHARGE_VCAL_ADDR    3
#define CHARGE_VCAL_MASK    0xff
#define CHARGE_ICAL_ADDR    0
#define CHARGE_ICAL_MASK    0x3f
#define EFUSE_BYTE_LENGTH   4

static int charge_get_efuse_calib(UINT8 *vcal, UINT8 *ical)
{
    EFUSE_OPER_ST efuse;
    int ret = -1;
    UINT8 i;
    UINT8 efuse_valid_check = 0;

    for(i = 0; i < EFUSE_BYTE_LENGTH; i ++)
    {
        efuse.addr = i;
        efuse.data = 0;
        ret = sddev_control(SCTRL_DEV_NAME, CMD_EFUSE_READ_BYTE, &efuse);
        if(ret == 0)
        {
            // bk_printf("Efuse[%d]: 0x%x\r\n", i, efuse.data);
            efuse_valid_check |= efuse.data;
        }
        else
        {
            return ret;
        }
    }

    if(efuse_valid_check == 0)
    {
        bk_printf("Efuse has no cali data\r\n");
        return 0;
    }

    efuse.addr = CHARGE_ICAL_ADDR;
    efuse.data = 0;

    ret = sddev_control(SCTRL_DEV_NAME, CMD_EFUSE_READ_BYTE, &efuse);
    if(ret == 0) {
        *ical = (efuse.data & CHARGE_ICAL_MASK);
    } else {
        // os_printf("efuse get MAC -1\r\n");
        return ret;
    }

    efuse.addr = CHARGE_VCAL_ADDR;
    efuse.data = 0;

    ret = sddev_control(SCTRL_DEV_NAME, CMD_EFUSE_READ_BYTE, &efuse);
    if(ret == 0) {
        *vcal = (efuse.data & CHARGE_VCAL_MASK);
    } else {
        // os_printf("efuse get MAC -1\r\n");
        return ret;
    }

    return 1;
}

static void charge_calib_init(void)
{
    UINT32 value;
    UINT8 VCAL = 0x80, ICAL = 0x20; //default
    int ret = -1;

    ret = charge_get_efuse_calib(&VCAL, &ICAL);

    if(ret != 1)
    {
        bk_printf("Use charge default cali data\r\n");
    }
    bk_printf("Charge vcal:%x, ical:%x\r\n", VCAL, ICAL);

    value = REG_READ(SCTRL_ANALOG_CTRL7);

    value &= ~(CHG_VCAL_MANUAL_CONTROL_MASK << CHG_VCAL_MANUAL_CONTROL_POS);
    value |= (VCAL << CHG_VCAL_MANUAL_CONTROL_POS);
    value |= CHG_VCAL_SELECT;

    value &= ~(CHG_ICAL_MANUAL_CONTROL_MASK << CHG_ICAL_MANUAL_CONTROL_POS);
    value |= (ICAL << CHG_ICAL_MANUAL_CONTROL_POS);
    value |= CHG_ICAL_SELECT;

    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static void charge_ldo_enable(uint32_t enable)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~(CHG_LDO_ENABLE);
    value |= (!!enable << CHG_LDO_ENABLE_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static void charge_enable(uint32_t enable)
{
    UINT32 value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~(CHG_ENABLE);
    value |= (!!enable << CHG_ENABLE_POS);
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static void charge_vusb_plugin_handler(void)
{
    CHARGE_INT_ST charge_int;

    /* charge full int enable*/
    charge_int.int_index = CHARGE_3_TERMINAL;
    charge_int.int_type = CHARGE_INT_TYPE_P_EDGE;
    charge_int.enable = true;
    charge_int_set(&charge_int);

    /* VUSB pull out int enable*/
    charge_int.int_index = CHARGE_7_USB_READY;
    charge_int.int_type = CHARGE_INT_TYPE_N_EDGE;
    charge_int.enable = true;
    charge_int_set(&charge_int);

    charge_enable(true);
    bk_printf("charge start\r\n");
}

static void charge_vusb_pullout_handler(void)
{
    CHARGE_INT_ST charge_int;

    /* charge full int disable*/
    charge_int.int_index = CHARGE_3_TERMINAL;
    charge_int.int_type = CHARGE_INT_TYPE_P_EDGE;
    charge_int.enable = false;
    charge_int_set(&charge_int);

    /* VUSB plug in int enable*/
    charge_int.int_index = CHARGE_7_USB_READY;
    charge_int.int_type = CHARGE_INT_TYPE_P_EDGE;
    charge_int.enable = true;
    charge_int_set(&charge_int);

    charge_enable(0);
    bk_printf("charge stop\r\n");
}

static void charge_full_handler(void)
{
    //TODO
    charge_enable(0);
    bk_printf("charge full\r\n");
}

void charge_isr(void)
{
    UINT32 status;
    status = REG_READ(CHARGE_REG0X2);
    // bk_printf("charge isr int status: %x\r\n", status);
    bk_printf("chg isr usb_rd%d, chg_term%d\r\n", !!(status&(1<<7)), !!(status&(1<<3)));
    bk_printf("chg sta usb_rd%d, chg_term%d\r\n", !!(status&(1<<15)), !!(status&(1<<11)));

    if (status & (1 << CHARGE_7_USB_READY))
    {
        if(status & CHARGE_REG0X2_CHARGE_X_STA_POS(CHARGE_7_USB_READY))
        {
            charge_vusb_plugin_handler();
        }
        else
        {
            charge_vusb_pullout_handler();
        }
    }
    else if (status & (1 << CHARGE_3_TERMINAL))
    {
        charge_full_handler();
    }

    REG_WRITE(CHARGE_REG0X2, status);
}

void charge_init(void)
{
    CHARGE_INT_ST charge_int;

    charge_cfg_clear_value();

    /* Set charge calib value */
    charge_calib_init();

    charge_ldo_enable(1);

    /* Set charge current */
    charge_current_control(0x7f);
    charge_trick_current_control(CHARGE_ITRICK_10PERCENT_CC);
    charge_cvend_current_control(CHARGE_ICVEND_20PERCENT_CC);

    vusb_detector_enable(1);
    charge_cc2cv_delay_set(2);

    intc_service_register(FIQ_CHARGE, PRI_FIQ_CHARGE, charge_isr);

    charge_int.int_index = CHARGE_7_USB_READY;
    charge_int.int_type = CHARGE_INT_TYPE_P_EDGE;
    charge_int.enable = true;
    charge_int_set(&charge_int);

    intc_enable(FIQ_CHARGE);

    sddev_register_dev(CHARGE_DEV_NAME, &charge_op);

    bk_printf("---- charge init ---- CHG0: 0x%x, CHG1: 0x%x\r\n", CHARGE_REG0X0_VAL, CHARGE_REG0X1_VAL);
}

void charge_exit(void)
{
    CHARGE_INDEX index=CHARGE_0_RECHARGE;

    // charge_cfg_clear_value();
    charge_enable(0);

    while(index <= CHARGE_7_USB_READY)
    {
        charge_int_state_clear(index);
        charge_int_disable(index);
        index++;
    }
}

UINT32 charge_ctrl(UINT32 cmd, void *param)
{
    UINT32 index;

    switch (cmd) {
    case CMD_CHG_FULL_OFFSET_SET:
        index = (*(UINT32*)param);
        charge_full_offset_set(index);
        break;

    case CMD_CHG_CMP_SET:
        index = (*(UINT32*)param);
        charge_cc_compensation_set(index);
        break;

    case CMD_CHG_CC2CV_DELAY_SET:
        index = (*(UINT32*)param);
        charge_cc2cv_delay_set(index);
        break;

    case CMD_VUSB_DETECTOR_ENABLE:
        index = (*(UINT32*)param);
        vusb_detector_enable(index);
        break;

    case CMD_CHG_CURRENT_CONTROL:
        index = (*(UINT32*)param);
        charge_current_control(index);
        break;

    case CMD_CHG_ICVEND_CONTROL:
        index = (*(UINT32*)param);
        charge_cvend_current_control(index);
        break;

    case CMD_CHG_ITRICK_CONTROL:
        index = (*(UINT32*)param);
        charge_trick_current_control(index);
        break;

    case CMD_CHG_LDO_ENABLE:
        index = (*(UINT32*)param);
        charge_ldo_enable(index);
        break;

    case CMD_CHG_ENABLE:
        index = (*(UINT32*)param);
        charge_enable(index);
        break;

    case CMD_CHG_INT_SET:
        charge_int_set(param);
        break;

    case CMD_INT_STATE_GET:
        index = (*(UINT32*)param);
        (*(UINT32*)param) = CHARGE_REG0X2_CHARGE_X_INT_STA_GET(index);
        break;

    default:
        break;
    }

    return true;
}

#endif // ((CFG_USE_CHARGE_DEV == 1) && (CFG_SOC_NAME == SOC_BK7252N))

// eof