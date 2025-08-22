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

#include "dd_pub.h"

#include "sdio_pub.h"
#include "sys_ctrl_pub.h"
#include "uart_pub.h"
#include "gpio_pub.h"
#include "icu_pub.h"
#include "wdt_pub.h"
#include "usb_pub.h"
#include "pwm_pub.h"
#include "flash_pub.h"
#include "spi_pub.h"
#include "fft_pub.h"
#include "i2s_pub.h"
#include "saradc_pub.h"
#if !(SOC_BK7252N == CFG_SOC_NAME)
#include "irda_pub.h"
#else
#include "irda_pub_bk7252n.h"
#endif
#include "mac_phy_bypass_pub.h"
#include "bk_timer_pub.h"
#include "i2c_pub.h"

#if CFG_USE_CAMERA_INTF
#include "jpeg_encoder_pub.h"
#endif

#if CFG_USE_AUDIO
#include "audio_pub.h"
#endif

#if CFG_USE_SDCARD_HOST
#include "sdcard_pub.h"
#endif

#if CFG_USE_HSLAVE_SPI
#include "spidma_pub.h"
#endif

#if CFG_GENERAL_DMA
#include "general_dma_pub.h"
#endif

#if CFG_USE_STA_PS
#include "power_save_pub.h"
#endif

#if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_SUPPORT_BLE == 1))
#include "ble_pub.h"
#endif

#ifdef CFG_USE_QSPI
#include "qspi_pub.h"
#endif

#if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_USE_SECURITY == 1))
#include "security_pub.h"
#endif

#if (SOC_BK7231N == CFG_SOC_NAME) || (CFG_SOC_NAME == SOC_BK7238)
#include "calendar_pub.h"
#endif

#if (SOC_BK7252N == CFG_SOC_NAME)
#include "rtc_reg_pub.h"
#include "charge_pub.h"
#include "hpm_pub.h"
#include "la_pub.h"
#include "yuv_buf_pub.h"
#include "ipchksum_pub.h"
#endif

#if (CFG_USE_SOFT_RTC)
#include "rtc.h"
#endif

static DD_INIT_S dd_init_tbl[] =
{
    /* name*/              /* init function*/          /* exit function*/
    {SCTRL_DEV_NAME,        sctrl_init,                 sctrl_exit},
    {ICU_DEV_NAME,          icu_init,                   icu_exit},
    {WDT_DEV_NAME,          wdt_init,                   wdt_exit},
    {GPIO_DEV_NAME,         gpio_init,                  gpio_exit},

    #if (SOC_BK7231N == CFG_SOC_NAME) || (CFG_SOC_NAME == SOC_BK7238)
    {CAL_DEV_NAME,			cal_init,					cal_exit},
    #endif

    #if (CFG_SOC_NAME == SOC_BK7252N)
    #if(CFG_USE_CHARGE_DEV == 1)
    {CHARGE_DEV_NAME,       charge_init,                charge_exit},
    #endif // (CFG_USE_CHARGE_DEV == 1)
    {RTC_REG_DEV_NAME,      rtc_reg_init,               rtc_reg_exit},
    {UART3_DEV_NAME,        uart3_init,                 uart3_exit},
    {HPM_DEV_NAME,          hpm_init,                   hpm_exit},
    {LA_DEV_NAME,           la_init,                    la_exit},
    {YUV_BUF_DEV_NAME,      yuv_buf_init,               yuv_buf_exit},
    {IPCHKSUM_DEV_NAME,     ipchksum_init,              ipchksum_exit},
    #endif
    {UART2_DEV_NAME,        uart2_init,                 uart2_exit},
    {UART1_DEV_NAME,        uart1_init,                 uart1_exit},

    {FLASH_DEV_NAME,        flash_init,                 flash_exit},

    #if CFG_GENERAL_DMA
    {GDMA_DEV_NAME,         gdma_init,                  gdma_exit},
    #endif

    #if CFG_USE_HSLAVE_SPI
    {SPIDMA_DEV_NAME,       spidma_init,                spidma_uninit},
    #endif

    #if CFG_USE_QSPI
    {QSPI_DEV_NAME,       qspi_init,                	qspi_exit},
    #endif

    #if CFG_USE_I2C1
    {I2C1_DEV_NAME,         i2c1_init,                  i2c1_exit},
    #endif

    #if CFG_USE_I2C2
    {I2C2_DEV_NAME,         i2c2_init,                  i2c2_exit},
    #endif

    #if CFG_USE_CAMERA_INTF
    {EJPEG_DEV_NAME,        ejpeg_init,                 ejpeg_exit},
    #endif

    #if CFG_USE_AUDIO
    {AUD_DAC_DEV_NAME,      audio_init,                 audio_exit},
    #endif

    #if CFG_SDIO || CFG_SDIO_TRANS
    {SDIO_DEV_NAME,         sdio_init,                  sdio_exit},
    #endif

    #if CFG_USB
    {USB_DEV_NAME,          usb_init,                   usb_exit},
    #endif

    #if (CFG_SOC_NAME == SOC_BK7221U)
    #if CFG_USB
    {USB_PLUG_DEV_NAME,     usb_plug_inout_init,        usb_plug_inout_exit},
    #endif
    #endif
    {PWM_DEV_NAME,          pwm_init,                   pwm_exit},
    #if (CFG_SOC_NAME != SOC_BK7231)
    {TIMER_DEV_NAME,        bk_timer_init,              bk_timer_exit},
    #endif

    #if (CFG_USE_SPI)
    {SPI_DEV_NAME,          spi_init,                   spi_exit},
    #endif

    #if (CFG_SOC_NAME == SOC_BK7271)
    {SPI2_DEV_NAME,         spi2_init,                  spi2_exit},
    {SPI3_DEV_NAME,         spi3_init,                  spi3_exit},
    #endif

    #if CFG_USE_FFT
    {FFT_DEV_NAME,          fft_init,                   fft_exit},
    #endif
    #if CFG_USE_I2S
    //{I2S_DEV_NAME,          i2s_init,                   i2s_exit},
    #endif

    #if CFG_SUPPORT_SARADC
    {SARADC_DEV_NAME,       saradc_init,                saradc_exit},
    #endif

    #if (CFG_SOC_NAME != SOC_BK7231)
    {IRDA_DEV_NAME,         irda_init,                  irda_exit},
    #endif

    #if CFG_MAC_PHY_BAPASS
    {MPB_DEV_NAME,          mpb_init,                   mpb_exit},
    #endif

    #if CFG_USE_SDCARD_HOST
    {SDCARD_DEV_NAME,       sdcard_init,                sdcard_exit},
    #endif

    #if CFG_USE_STA_PS
    {"power_save",       sctrl_sta_ps_init,             NULLPTR},
    #endif

    #if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_SUPPORT_BLE == 1))
    {BLE_DEV_NAME,       ble_init,                      ble_exit}, //sean
    #endif

    #if ((CFG_SOC_NAME != SOC_BK7231) && (CFG_USE_SECURITY == 1))
    {SEC_DEV_NAME,       bk_secrity_init,               bk_secrity_exit},
    #endif

    #if (CFG_USE_SOFT_RTC)
    {SOFT_RTC_DEVICE_NAME,  rt_soft_rtc_init,           NULLPTR},
    #endif
    {NULL,                  NULLPTR,                    NULLPTR}
};

void g_dd_init(void)
{
    UINT32 i;
    UINT32 tbl_count;
    DD_INIT_S *dd_element;

    tbl_count = sizeof(dd_init_tbl) / sizeof(DD_INIT_S);
    for(i = 0; i < tbl_count; i ++)
    {
        dd_element = &dd_init_tbl[i];
        if(dd_element->dev_name && dd_element->init)
        {
            (dd_element->init)();
        }
        else
        {
            return;
        }
    }
}

void g_dd_exit(void)
{
    UINT32 i;
    UINT32 tbl_count;
    DD_INIT_S *dd_element;

    tbl_count = sizeof(dd_init_tbl) / sizeof(DD_INIT_S);
    for(i = 0; i < tbl_count; i ++)
    {
        dd_element = &dd_init_tbl[i];
        if(dd_element->dev_name && dd_element->exit)
        {
            (dd_element->exit)();
        }
        else
        {
            return;
        }
    }
}

// EOF
