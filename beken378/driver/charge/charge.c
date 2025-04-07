#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "charge_pub.h"
#include "charge.h"
#include "target_util_pub.h"
#include "sys_ctrl.h"
#include "intc_pub.h"

#if ((CFG_USE_CHARGE_DEV == 1) && (CFG_SOC_NAME == SOC_BK7252N))

static SDD_OPERATIONS charge_op = {
    charge_ctrl
};

static void charge_cfg_init_value(uint32_t val)
{
    REG_WRITE(CHARGE_REG0X0, val);
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
    REG_WRITE(CHARGE_REG0X0, CHARGE_REG0X1_CHARGE_X_INT_SET(index,mode));
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

static uint32_t charge_calibrate_Vcal(void)
{
    uint32_t value;
    volatile uint32_t delay;
    uint32_t restore = REG_READ(SCTRL_ANALOG_CTRL7);
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value |= CHG_ENABLE;
    value &= ~(CHG_CURRENT_CONTROL_MASK << CHG_CURRENT_CONTROL_POS);
    value &= ~(CHG_VCAL_SELECT);
    value |= CHG_ICAL_SELECT;
    value &= ~(CHG_VCAL_TRIG);
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    delay = 10000;
    while(delay--);
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value |= CHG_VCAL_TRIG;
    delay = 100;
    while(delay--);
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~(CHG_VCAL_TRIG);
    delay = 10000;
    while(delay--);
    value = (REG_READ(SCTRL_ANALOG_STATE) >> CHG_VCAL_POS) & CHG_VCAL_MASK;
    REG_WRITE(SCTRL_ANALOG_CTRL7,restore);
    REG_WRITE(SCTRL_ANALOG_CTRL7,(value << CHG_VCAL_MANUAL_CONTROL_POS));
    return value;
}

static void charge_calibrate_Ical_prepare(uint32_t Vcal_value)
{
    uint32_t value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value |= CHG_ENABLE;
    value |= Vcal_value << CHG_VCAL_MANUAL_CONTROL_POS;
    value |= 0x3C << CHG_CURRENT_CONTROL_POS;
    value |= CHG_VCAL_SELECT;
    value &= ~ CHG_ICAL_SELECT;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static void charge_calibrate_Ical_trigger(void)
{
    uint32_t value;
    uint32_t delay;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~CHG_ENABLE;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    delay = 20000;
    while(delay--);
    value |= CHG_ENABLE;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static uint32_t charge_calibrate_Ical(void)
{
    uint32_t value;
    uint32_t delay;
    uint32_t cali_value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value &= ~CHG_ICAL_TRIG;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    delay = 3000;
    while(delay--);
    value |= CHG_ICAL_TRIG;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    delay = 100;
    while(delay--);
    value &= ~CHG_ICAL_TRIG;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    delay = 10000;
    while(delay--);
    cali_value = (REG_READ(SCTRL_ANALOG_STATE) >> CHG_ICAL_POS) & CHG_ICAL_MASK;
    return cali_value;
}

static void charge_verify_Ical(uint32_t cali_value)
{
    uint32_t value;
    REG_WRITE(SCTRL_ANALOG_CTRL7,(cali_value << CHG_ICAL_MANUAL_CONTROL_POS));
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value |= CHG_ICAL_SELECT;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
}

static void charge_verify_Vcal(uint8_t Vcal_value, uint8_t Ical_value)
{
    uint32_t value;
    value = REG_READ(SCTRL_ANALOG_CTRL7);
    value |= (Vcal_value << CHG_VCAL_MANUAL_CONTROL_POS | Ical_value << CHG_ICAL_MANUAL_CONTROL_POS
                | CHG_VCAL_SELECT | CHG_ICAL_SELECT | CHG_ENABLE);
    value &= ~CHG_MODE_SELECT;
    REG_WRITE(SCTRL_ANALOG_CTRL7,value);
    value = REG_READ(SCTRL_ANALOG_CTRL6);
    value |= CHG_CC_MANUAL_ENABLE;
    value = REG_WRITE(SCTRL_ANALOG_CTRL6,value);
}

void charge_isr(void)
{
#if 1 // test code for yingjia
    UINT32 status, value;
    status = REG_READ(CHARGE_REG0X2);
    bk_printf("charge isr int status: %x\r\n", status);
    REG_WRITE(CHARGE_REG0X2, status | 0xFF);

    if (status & (1 << 3))
    {
        value = REG_READ(SCTRL_ANALOG_CTRL7);
        value &= ~(CHG_ENABLE);
        REG_WRITE(SCTRL_ANALOG_CTRL7,value);
        bk_printf("sys 0x22: %x\r\n", value);
    }
#else
    CHARGE_INDEX index=CHARGE_0;

    while(index <= CHARGE_7)
    {
        if (charge_int_state_get(index))
        {
            // todo
            charge_int_state_clear(index);
        }
    }
#endif
}

void charge_init(void)
{
    CHARGE_INDEX index=CHARGE_0;

    charge_cfg_init_value(0);

    intc_service_register(FIQ_CHARGE, PRI_FIQ_CHARGE, charge_isr);
    while(index <= CHARGE_7)
    {
        // charge_int_state_clear(index);
        charge_int_type_set(index,CHARGE_INT_TYPE_H_LV);
        // charge_int_enable(index);
        index++;
    }
    intc_enable(FIQ_CHARGE);

    sddev_register_dev(CHARGE_DEV_NAME, &charge_op);
}

void charge_exit(void)
{
    CHARGE_INDEX index=CHARGE_0;

    charge_cfg_init_value(0);

    while(index <= CHARGE_7)
    {
        charge_int_state_clear(index);
        charge_int_disable(index);
        index++;
    }
}

UINT32 charge_ctrl(UINT32 cmd, void *param)
{
    UINT32 vcal_value;
    UINT32 ical_value;
    UINT32 index;

    switch (cmd) {
    case CMD_VCAL_CALIBRATE:
        (*(UINT32*)param) = charge_calibrate_Vcal();
        break;

    case CMD_ICAL_CALIBRATE_PREPARE:
        vcal_value = (*(UINT32*)param);
        charge_calibrate_Ical_prepare(vcal_value);
        break;

    case CMD_ICAL_CALIBRATE_TRIGGER:
        charge_calibrate_Ical_trigger();
        break;

    case CMD_ICAL_CALIBRATE:
        (*(UINT32*)param) = charge_calibrate_Ical();
        break;

    case CMD_ICAL_VERIFY:
        ical_value = (*(UINT32*)param);
        charge_verify_Ical(ical_value);
        break;

    case CMD_VCAL_VERIFY:
        vcal_value = (*(UINT32*)param) & 0xFF;
        ical_value = (*(UINT32*)param >> 8) & 0x1F;
        charge_verify_Vcal(vcal_value,ical_value);
        break;

    case CMD_INT_STATE_GET:
        index = (*(UINT32*)param);
        (*(UINT32*)param) = CHARGE_REG0X2_CHARGE_X_INT_STA_GET(index);

    default:
        break;
    }

    return 0;
}

#endif // ((CFG_USE_CHARGE_DEV == 1) && (CFG_SOC_NAME == SOC_BK7252N))

// eof