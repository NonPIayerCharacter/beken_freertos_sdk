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

#include "audio.h"
#include "audio_pub.h"

#include "intc_pub.h"
#include "icu_pub.h"
#include "sys_ctrl_pub.h"

#include "drv_model_pub.h"
#include "mem_pub.h"
#include "ring_buffer.h"
#include "common_reg_rw.h"

#if ((CFG_USE_AUDIO) && ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N)))
void audio_power_up(void)
{
    UINT32 param;
    param = PWD_AUDIO_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    sddev_control(ICU_DEV_NAME, CMD_ICU_CLKGATING_DISABLE, &param);
    #endif
}

void audio_power_down(void)
{
    UINT32 param;
    param = PWD_AUDIO_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    sddev_control(ICU_DEV_NAME, CMD_ICU_CLKGATING_ENABLE, &param);
    #endif
}

void audio_enable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_AUDIO_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
}

void audio_disable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_AUDIO_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

static void audio_isr(void)
{
    UINT32 status = REG_READ(AUD_AD_FIFO_STATUS);
    //AUD_PRT("%x\r\n", status);
    if (status & (DAC_R_INT_FLAG | DAC_L_INT_FLAG))
    {
        #if CFG_USE_AUD_DAC
        audio_dac_isr();
        #endif

        REG_WRITE_PROTECT(AUD_AD_FIFO_STATUS,
                  DAC_R_NEAR_FULL | DAC_L_NEAR_FULL
                  | DAC_R_NEAR_EMPTY | DAC_L_NEAR_EMPTY
                  | DAC_R_FIFO_FULL | DAC_L_FIFO_FULL
                  | DAC_R_FIFO_EMPTY | DAC_L_FIFO_EMPTY
                  | DAC_R_INT_FLAG | DAC_L_INT_FLAG);

    }
    if (status & ADC_INT_FLAG)
    {
        #if CFG_USE_AUD_ADC
        audio_adc_isr();
        #endif

        REG_WRITE_PROTECT(AUD_AD_FIFO_STATUS,
                  ADC_NEAR_FULL | ADC_NEAR_EMPTY | ADC_FIFO_FULL
                  | ADC_FIFO_EMPTY | ADC_INT_FLAG);
    }
    if (status & DTMF_INT_FLAG)
    {
        //audio_dtmf_isr(status);
        REG_WRITE_PROTECT(AUD_AD_FIFO_STATUS,
                  DTMF_NEAR_FULL | DTMF_NEAR_EMPTY | DTMF_FIFO_FULL
                  | DTMF_FIFO_EMPTY | DTMF_INT_FLAG);
    }
}

void audio_hardware_init(void)
{
    UINT32 val;

    /* register interrupt */
    intc_service_register(IRQ_AUDIO, PRI_IRQ_AUDIO, audio_isr);

    REG_WRITE_PROTECT(AUDIO_CONFIG, 0);

    REG_WRITE_PROTECT(AUD_DTMF_CONFIG_0, 0);
    REG_WRITE_QSPI_RST(AUD_DTMF_CONFIG_1, 0);
    REG_WRITE_PROTECT(AUD_DTMF_CONFIG_2, 0);

    REG_WRITE_PROTECT(AUD_ADC_CONFIG_0, 0x00e93A22);
    REG_WRITE_PROTECT(AUD_ADC_CONFIG_1, 0x8BBF3A22);
    REG_WRITE_PROTECT(AUD_ADC_CONFIG_2, 0xC9E6751C);
    REG_WRITE_PROTECT(AUD_AGC_CONFIG_0, 0x4A019465);
    REG_WRITE_PROTECT(AUD_AGC_CONFIG_1, 0x02016C01);
    REG_WRITE_PROTECT(AUD_AGC_CONFIG_2, 0x0F020940);

    REG_WRITE_PROTECT(AUD_DAC_CONFIG_0, 0);
    REG_WRITE_PROTECT(AUD_DAC_CONFIG_1, 0);
    REG_WRITE_PROTECT(AUD_DAC_CONFIG_2, 0);

    // it's very import to config dac interrupt thred(not all zero)
    REG_WRITE_PROTECT(AUD_FIFO_CONFIG, 0x210);

    /* reset int status */
    val = REG_READ(AUD_AD_FIFO_STATUS);
    REG_WRITE_PROTECT(AUD_AD_FIFO_STATUS, val);

    #if(CFG_AUD_DAC_USE_PORT_SET == CFG_AUD_DAC_SINGLE_PORT)
    audio_dac_volume_use_single_port();
    #else
    audio_dac_volume_diff_port();
    #endif
}


void audio_init(void)
{
    #if CFG_USE_AUD_DAC
    audio_dac_software_init();
    #endif

    #if CFG_USE_AUD_ADC
    audio_adc_software_init();
    #endif

    audio_hardware_init();
}

void audio_exit(void)
{
    UINT32 val;

    REG_WRITE_PROTECT(AUDIO_CONFIG, 0);

    REG_WRITE_PROTECT(AUD_DTMF_CONFIG_0, 0);
    REG_WRITE_QSPI_RST(AUD_DTMF_CONFIG_1, 0);
    REG_WRITE_PROTECT(AUD_DTMF_CONFIG_2, 0);

    REG_WRITE_PROTECT(AUD_ADC_CONFIG_0, 0);
    REG_WRITE_PROTECT(AUD_ADC_CONFIG_1, 0);
    REG_WRITE_PROTECT(AUD_ADC_CONFIG_2, 0);

    REG_WRITE_PROTECT(AUD_DAC_CONFIG_0, 0);
    REG_WRITE_PROTECT(AUD_DAC_CONFIG_1, 0);
    REG_WRITE_PROTECT(AUD_DAC_CONFIG_2, 0);

    REG_WRITE_PROTECT(AUD_FIFO_CONFIG, 0);

    /* reset int status */
    val = REG_READ(AUD_AD_FIFO_STATUS);
    REG_WRITE_PROTECT(AUD_AD_FIFO_STATUS, val);

    #if CFG_USE_AUD_DAC
    ddev_unregister_dev(AUD_DAC_DEV_NAME);
    #endif
}
#endif // CFG_USE_AUDIO
