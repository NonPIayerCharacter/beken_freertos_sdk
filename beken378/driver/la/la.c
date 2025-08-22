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
#include "la_pub.h"
#include "la.h"
#include "target_util_pub.h"
#include "common_reg_rw.h"

#if (CFG_SOC_NAME == SOC_BK7252N)

static SDD_OPERATIONS la_op = {
    la_ctrl
};

__maybe_unused static void la_soft_rstn(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(LA_REG0X2_ADDR);

    if(enable)
    {
        value |= LA_REG0X2_SOFT_RST;
    }
    else
    {
        value &= ~LA_REG0X2_SOFT_RST;
    }

    REG_WRITE_QSPI_RST(LA_REG0X2_ADDR, value);
}

__maybe_unused static void la_cfg_init_value(UINT32 val)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X3_ADDR);
    reg = val;
    REG_WRITE_PROTECT(LA_REG0X3_ADDR, reg);
}

__maybe_unused static void la_set_smp_enable(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(LA_REG0X3_ADDR);

    if(enable)
    {
        value |= LA_REG0X3_LA_SMP_EN;
    }
    else
    {
        value &= ~LA_REG0X3_LA_SMP_EN;
    }

    REG_WRITE_PROTECT(LA_REG0X3_ADDR, value);
}

__maybe_unused static void la_set_smp_clk_inv(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(LA_REG0X3_ADDR);

    if(enable)
    {
        value |= LA_REG0X3_LA_SMP_CLK_INV;
    }
    else
    {
        value &= ~LA_REG0X3_LA_SMP_CLK_INV;
    }

    REG_WRITE_PROTECT(LA_REG0X3_ADDR, value);
}

__maybe_unused static void la_set_trig_mode(UINT8 mode)
{
    UINT32 value;

    value = REG_READ(LA_REG0X3_ADDR);

    if(mode == (LA_TRIG_MODE_0_EQUAL_TO_LASMPVALUE || LA_TRIG_MODE_1_CHANGE))
    {
        value |= ((mode & LA_REG0X3_LA_SMP_MOD_MASK) << LA_REG0X3_LA_SMP_MOD_POSI);
    }
    else
    {
        value &= ~(LA_REG0X3_LA_SMP_MOD_MASK << LA_REG0X3_LA_SMP_MOD_POSI);;
    }

    REG_WRITE_PROTECT(LA_REG0X3_ADDR, value);
}

__maybe_unused static void la_set_smp_int_en(UINT8 mode)
{
    UINT32 value;

    value = REG_READ(LA_REG0X3_ADDR);

    mode = mode & LA_REG0X3_LA_SMP_INT_EN_MASK;
    value &= (LA_REG0X3_LA_SMP_INT_EN_MASK << LA_REG0X3_LA_SMP_INT_EN_POSI);

    if(mode == LA_SMP_INT_EN_MODE_TRANSFER_FINISH_INT)
    {
        value |= (LA_SMP_INT_EN_MODE_TRANSFER_FINISH_INT << LA_REG0X3_LA_SMP_INT_EN_POSI);
    }
    else if(mode == LA_SMP_INT_EN_MODE_BUS_ERR_INT)
    {
        value |= (LA_SMP_INT_EN_MODE_BUS_ERR_INT << LA_REG0X3_LA_SMP_INT_EN_POSI);
    }
    else if(mode == LA_SMP_INT_EN_MODE_BOTH)
    {
        value |= (LA_SMP_INT_EN_MODE_BOTH << LA_REG0X3_LA_SMP_INT_EN_POSI);
    }

    REG_WRITE_PROTECT(LA_REG0X3_ADDR, value);
}

__maybe_unused static void la_set_mem_secu_attr(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(LA_REG0X3_ADDR);

    if(enable)
    {
        value |= LA_REG0X3_MEM_SECU_ATTR;
    }
    else
    {
        value &= ~LA_REG0X3_MEM_SECU_ATTR;
    }

    REG_WRITE_PROTECT(LA_REG0X3_ADDR, value);
}

__maybe_unused static void la_set_smp_source(UINT8 source)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X3_ADDR);
    reg &= ~(LA_REG0X3_LA_SMP_SOURCE_MASK << LA_REG0X3_LA_SMP_SOURCE_POSI);
    reg |= ((source & LA_REG0X3_LA_SMP_SOURCE_MASK) << LA_REG0X3_LA_SMP_SOURCE_POSI);
    REG_WRITE_PROTECT(LA_REG0X3_ADDR, reg);
}

__maybe_unused static void la_set_smp_len(UINT32 len)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X3_ADDR);
    reg &= ~(LA_REG0X3_LA_SMP_LEN_MASK << LA_REG0X3_LA_SMP_LEN_POSI);
    reg |= ((len & LA_REG0X3_LA_SMP_LEN_MASK) << LA_REG0X3_LA_SMP_LEN_POSI);
    REG_WRITE_PROTECT(LA_REG0X3_ADDR, reg);
}

__maybe_unused static void la_set_smp_trigger_value(UINT32 val)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X4_LA_SMP_VALUE);
    reg = val;
    REG_WRITE_PROTECT(LA_REG0X4_LA_SMP_VALUE, reg);
}

__maybe_unused static void la_set_smp_data_mask(UINT32 val)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X5_LA_SMP_MASK);
    reg = val;
    REG_WRITE_PROTECT(LA_REG0X5_LA_SMP_MASK, reg);
}

__maybe_unused static UINT32 la_get_smp_data(void)
{
    return (REG_READ(LA_REG0X6_LA_SMP_DATA));
}

__maybe_unused static void la_set_smp_start_addr(UINT32 val)
{
    UINT32 reg;

    reg = REG_READ(LA_REG0X7_LA_SMP_START_ADDR);
    reg = val;
    REG_WRITE_PROTECT(LA_REG0X7_LA_SMP_START_ADDR, reg);
}

__maybe_unused static LA_BUS_ERR_FLAG la_get_bus_err_flag(void)
{
    return ((REG_READ(LA_REG0X8_ADDR) & LA_REG0X8_BUS_ERR_FLAG)
            >> LA_REG0X8_BUS_ERR_FLAG_POSI);
}

__maybe_unused static void la_clr_smp_int_status(UINT8 mode)
{
    UINT32 value;

    value = REG_READ(LA_REG0X8_ADDR);

    mode = mode & LA_REG0X8_LA_SMP_INT_STATUS_MASK;
    value &= (LA_REG0X8_LA_SMP_INT_STATUS_MASK << LA_REG0X8_LA_SMP_INT_STATUS_POSI);

    if(mode == LA_SMP_INT_EN_MODE_TRANSFER_FINISH_INT)
    {
        value |= (LA_SMP_INT_EN_MODE_TRANSFER_FINISH_INT << LA_REG0X8_LA_SMP_INT_STATUS_POSI);
    }
    else if(mode == LA_SMP_INT_EN_MODE_BUS_ERR_INT)
    {
        value |= (LA_SMP_INT_EN_MODE_BUS_ERR_INT << LA_REG0X8_LA_SMP_INT_STATUS_POSI);
    }
    else if(mode == LA_SMP_INT_EN_MODE_BOTH)
    {
        value |= (LA_SMP_INT_EN_MODE_BOTH << LA_REG0X8_LA_SMP_INT_STATUS_POSI);
    }

    REG_WRITE_PROTECT(LA_REG0X8_ADDR, value);
}

void la_init(void)
{
    sddev_register_dev(LA_DEV_NAME, &la_op);
}

void la_exit(void)
{
    ddev_unregister_dev(LA_DEV_NAME);
}

UINT32 la_ctrl(UINT32 cmd, void *param)
{
    switch (cmd) {
    default:
        break;
    }

    return 0;
}

#endif

// eof

