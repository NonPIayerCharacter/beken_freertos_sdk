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
#include "rtos_pub.h"
#include "BkDriverQspi.h"
#include "drv_model_pub.h"
#include "error.h"
#include "qspi_pub.h"

#if (CFG_SOC_NAME != SOC_BK7252N)
OSStatus bk_qspi_dcache_initialize(qspi_dcache_drv_desc *qspi_config)
{
    UINT32 param, ret;

    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_DCACHE_CONFIG, qspi_config);
    ASSERT(QSPI_SUCCESS == ret);

    param = qspi_config->voltage_level;
    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_SET_VOLTAGE, &param);
    ASSERT(QSPI_SUCCESS == ret);

    param = qspi_config->clk_set;
    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_DIV_CLK_SET,  &param);
    ASSERT(QSPI_SUCCESS == ret);

    param &= 0x30;						//set psram clk source
    if(param == 0x00)
    {
        ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_CLK_SET_DCO, NULL);
        ASSERT(QSPI_SUCCESS == ret);
    }
    else if (param == 0x10)
    {
        ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_CLK_SET_26M, NULL);
        ASSERT(QSPI_SUCCESS == ret);
    }
    else
    {
        ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_CLK_SET_120M, NULL);
        ASSERT(QSPI_SUCCESS == ret);
    }

    if(qspi_config->mode == 0)
    {
        param = 1;
    }
    else if(qspi_config->mode == 3)
    {
        param = 4;
    }
    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_GPIO_CONFIG,  &param);
    ASSERT(QSPI_SUCCESS == ret);


    return kNoErr;
}

OSStatus bk_qspi_start(void)
{
    return sddev_control(QSPI_DEV_NAME, QSPI_DCACHE_CMD_OPEN,  NULL);
}

OSStatus bk_qspi_stop(void)
{
    return sddev_control(QSPI_DEV_NAME, QSPI_DCACHE_CMD_CLOSE,  NULL);
}
#else
OSStatus bk_qspi_psram_initialize(uint8_t line_mode)
{
    UINT32 param, ret;

    ret = sddev_control(QSPI_DEV_NAME, QSPI_DCACHE_CMD_OPEN, NULL);
    if(ret != 0)
        return kGeneralErr;

    //fixed to dpll120M with div=1
    sddev_control(QSPI_DEV_NAME, QSPI_CMD_CLK_SET_120M, NULL);
    param = 1;
    sddev_control(QSPI_DEV_NAME, QSPI_CMD_DIV_CLK_SET, &param);

    if(line_mode == QSPI_1WIRE)
        param = 1;
    else if(line_mode == QSPI_4WIRE)
        param = 4;
    else
    {
        return kParamErr;
    }
    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_GPIO_CONFIG, &param);
    if(ret != 0)
        return kGeneralErr;

    return kNoErr;
}

OSStatus bk_qspi_psram_enter_quad_mode(void)
{
    qspi_cmd_t enter_quad_cmd = {0};
    UINT32 ret;

    enter_quad_cmd.wire_mode = QSPI_1WIRE;
    enter_quad_cmd.work_mode = INDIRECT_MODE;
    enter_quad_cmd.op = QSPI_WRITE;
    enter_quad_cmd.cmd = APS6404_CMD_ENTER_QUAD_MODE;

    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_DCACHE_CONFIG, &enter_quad_cmd);
    if(ret != 0)
        return kGeneralErr;

    return kNoErr;
}

OSStatus bk_qspi_psram_quad_write(void)
{
    qspi_cmd_t quad_wr_cmd = {0};
    UINT32 ret;

    quad_wr_cmd.wire_mode = QSPI_4WIRE;
    quad_wr_cmd.work_mode = MEMORY_MAPPED_MODE;
    quad_wr_cmd.op = QSPI_WRITE;
    quad_wr_cmd.cmd = APS6404_CMD_QUAD_WRITE;
    quad_wr_cmd.addr = 0;
    quad_wr_cmd.dummy_cycle = 0;

    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_DCACHE_CONFIG, &quad_wr_cmd);
    if(ret != 0)
        return kGeneralErr;

    return kNoErr;
}

OSStatus bk_qspi_psaram_quad_read(void)
{
    qspi_cmd_t qspi_rd_cmd = {0};
    UINT32 ret;

    qspi_rd_cmd.wire_mode = QSPI_4WIRE;
    qspi_rd_cmd.work_mode = MEMORY_MAPPED_MODE;
    qspi_rd_cmd.op = QSPI_READ;
    qspi_rd_cmd.cmd = APS6404_CMD_FAST_READ_QUAD;
    qspi_rd_cmd.addr = 0;
    qspi_rd_cmd.dummy_cycle = 6;

    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_DCACHE_CONFIG, &qspi_rd_cmd);
    if(ret != 0)
        return kGeneralErr;

    return kNoErr;
}

OSStatus bk_qspi_psram_switch_mcu_mode(void)
{
    UINT32 param, ret;

    param = QSPI_MEM_FOR_CPU;
    ret = sddev_control(QSPI_DEV_NAME, QSPI_CMD_SWITCH_MODE, &param);
    if(ret != 0)
        return kGeneralErr;

    return kNoErr;
}
#endif
// eof

