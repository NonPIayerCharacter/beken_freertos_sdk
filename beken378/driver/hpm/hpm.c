#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "hpm_pub.h"
#include "hpm.h"
#include "target_util_pub.h"

#if (CFG_SOC_NAME == SOC_BK7252N)

static SDD_OPERATIONS hpm_op = {
    hpm_ctrl
};

__maybe_unused static void hpm_cfg_init_value(UINT32 val)
{
    UINT32 reg;

    reg = REG_READ(HPM_REG0X0_CFG);
    reg = val;
    REG_WRITE(HPM_REG0X0_CFG, reg);
}

__maybe_unused static void hpm_soft_rstn(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(HPM_REG0X0_CFG);

    if(enable)
    {
        value |= HPM_REG0X0_CFG_SOFT_RSTN;
    }
    else
    {
        value &= ~HPM_REG0X0_CFG_SOFT_RSTN;
    }

    REG_WRITE(HPM_REG0X0_CFG, value);
}

__maybe_unused static void hpm_mode_sel(UINT8 mode)
{
    UINT32 value;

    value = REG_READ(HPM_REG0X0_CFG);

    if(mode == HPM_MODE_MONITOR_EN)
    {
        value |= HPM_REG0X0_CFG_MONITOR_EN;
    }
    else if(mode == HPM_MODE_SINGLE_EN)
    {
        value |= HPM_REG0X0_CFG_SINGLE_EN;
    }
    else if(mode == HPM_MODE_MONITOR_DIS)
    {
        value &= ~HPM_REG0X0_CFG_SINGLE_EN;
    }
    else if(mode == HPM_MODE_SINGLE_DIS)
    {
        value &= ~HPM_REG0X0_CFG_SINGLE_EN;
    }

    REG_WRITE(HPM_REG0X0_CFG, value);
}

__maybe_unused static void hpm_cfg(UINT16 offset, UINT8 shift, UINT8 period)
{
    UINT32 value;

    value = REG_READ(HPM_REG0X0_CFG);

    value = value & ~(HPM_REG0X0_CFG_HPM_OFFSET_MASK << HPM_REG0X0_CFG_HPM_OFFSET_POSI)
                  & ~(HPM_REG0X0_CFG_HPM_SHIFT_MASK << HPM_REG0X0_CFG_HPM_SHIFT_POSI)
                  & ~(HPM_REG0X0_CFG_HPM_PERIOD_MASK << HPM_REG0X0_CFG_HPM_PERIOD_POSI);
    value = value | ((period & HPM_REG0X0_CFG_HPM_OFFSET_MASK) << HPM_REG0X0_CFG_HPM_OFFSET_POSI)
                  | ((period & HPM_REG0X0_CFG_HPM_SHIFT_MASK) << HPM_REG0X0_CFG_HPM_SHIFT_POSI)
                  | ((period & HPM_REG0X0_CFG_HPM_PERIOD_MASK) << HPM_REG0X0_CFG_HPM_PERIOD_POSI);

    REG_WRITE(HPM_REG0X0_CFG, value);
}

__maybe_unused static void hpm_set_limit(UINT16 dn_limit, UINT16 up_limit)
{
    UINT32 value;

    value = REG_READ(HPM_REG0X1_THD);

    value = value & ~(HPM_REG0X1_THD_HPM_DN_LIMIT_MASK << HPM_REG0X1_THD_HPM_DN_LIMIT_POSI)
                  & ~(HPM_REG0X1_THD_HPM_UP_LIMIT_MASK << HPM_REG0X1_THD_HPM_UP_LIMIT_POSI);
    value = value | ((dn_limit & HPM_REG0X1_THD_HPM_DN_LIMIT_MASK) << HPM_REG0X1_THD_HPM_DN_LIMIT_POSI)
                  | ((up_limit & HPM_REG0X1_THD_HPM_UP_LIMIT_MASK) << HPM_REG0X1_THD_HPM_UP_LIMIT_POSI);

    REG_WRITE(HPM_REG0X1_THD, value);
}

__maybe_unused static void hpm_set_wraning_enable(UINT16 dn_wraning_enable, UINT16 up_wraning_enable)
{
    UINT32 value;

    value = REG_READ(HPM_REG0X2_RCD1);

    if(dn_wraning_enable)
    {
        value |= HPM_REG0X2_RCD1_HPM_DN_WRANING;
    }
    else
    {
        value &= ~HPM_REG0X2_RCD1_HPM_DN_WRANING;
    }

    if(up_wraning_enable)
    {
        value |= HPM_REG0X2_RCD1_HPM_UP_WRANING;
    }
    else
    {
        value &= ~HPM_REG0X2_RCD1_HPM_UP_WRANING;
    }

    REG_WRITE(HPM_REG0X2_RCD1, value);
}

__maybe_unused static void hpm_set_record_x_valid(UINT8 record_index, UINT8 record_valid, UINT16 record_value)
{
    UINT32 value = 0;

    if(record_valid)
    {
        value |= HPM_REG0X2_RCD1_HPM_UP_WRANING;
    }
    else
    {
        value &= ~HPM_REG0X2_RCD1_HPM_UP_WRANING;
    }

    if(record_index == HPM_RECORD_0)
    {
        value = REG_READ(HPM_REG0X2_RCD1);
        value = value & ~(HPM_REG0X2_RCD1_HPM_RECORD0_VALID)
                      & ~(HPM_REG0X2_RCD1_HPM_RECORD0_MASK << HPM_REG0X2_RCD1_HPM_RECORD0_POSI);
        value = value | ((record_valid & HPM_REG0X2_RCD1_HPM_RECORD0_VALID_MASK)
                        << HPM_REG0X2_RCD1_HPM_RECORD0_VALID_POSI)
                      | ((record_value & HPM_REG0X2_RCD1_HPM_RECORD0_MASK)
                        << HPM_REG0X2_RCD1_HPM_RECORD0_POSI);
        REG_WRITE(HPM_REG0X2_RCD1, value);
    }
    else if(record_index == HPM_RECORD_1)
    {
        value = REG_READ(HPM_REG0X2_RCD1);
        value = value & ~(HPM_REG0X2_RCD1_HPM_RECORD1_VALID)
                      & ~(HPM_REG0X2_RCD1_HPM_RECORD1_MASK << HPM_REG0X2_RCD1_HPM_RECORD1_POSI);
        value = value | ((record_valid & HPM_REG0X2_RCD1_HPM_RECORD1_VALID_MASK)
                        << HPM_REG0X2_RCD1_HPM_RECORD1_VALID_POSI)
                      | ((record_value & HPM_REG0X2_RCD1_HPM_RECORD1_MASK)
                        << HPM_REG0X2_RCD1_HPM_RECORD1_POSI);
        REG_WRITE(HPM_REG0X2_RCD1, value);
    }
    else if(record_index == HPM_RECORD_2)
    {
        value = REG_READ(HPM_REG0X3_RCD2);
        value = value & ~(HPM_REG0X3_RCD2_HPM_RECORD2_VALID)
                      & ~(HPM_REG0X3_RCD2_HPM_RECORD2_MASK << HPM_REG0X3_RCD2_HPM_RECORD2_POSI);
        value = value | ((record_valid & HPM_REG0X3_RCD2_HPM_RECORD2_VALID_MASK)
                        << HPM_REG0X3_RCD2_HPM_RECORD2_VALID_POSI)
                      | ((record_value & HPM_REG0X3_RCD2_HPM_RECORD2_MASK)
                        << HPM_REG0X3_RCD2_HPM_RECORD2_POSI);
        REG_WRITE(HPM_REG0X3_RCD2, value);
    }
    else if(record_index == HPM_RECORD_3)
    {
        value = REG_READ(HPM_REG0X3_RCD2);
        value = value & ~(HPM_REG0X3_RCD2_HPM_RECORD3_VALID)
                      & ~(HPM_REG0X3_RCD2_HPM_RECORD3_MASK << HPM_REG0X3_RCD2_HPM_RECORD3_POSI);
        value = value | ((record_valid & HPM_REG0X3_RCD2_HPM_RECORD3_VALID_MASK)
                        << HPM_REG0X3_RCD2_HPM_RECORD3_VALID_POSI)
                      | ((record_value & HPM_REG0X3_RCD2_HPM_RECORD3_MASK)
                        << HPM_REG0X3_RCD2_HPM_RECORD3_POSI);
        REG_WRITE(HPM_REG0X3_RCD2, value);
    }
}

void hpm_init(void)
{
    sddev_register_dev(HPM_DEV_NAME, &hpm_op);
}

void hpm_exit(void)
{
    ddev_unregister_dev(HPM_DEV_NAME);
}

UINT32 hpm_ctrl(UINT32 cmd, void *param)
{
	switch (cmd) {
	default:
		break;
	}

	return 0;
}

#endif

// eof

