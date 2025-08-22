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
#include "typedef.h"
#include "sys_config.h"
#include "spi_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "mem_pub.h"
#include "rtos_pub.h"

#if CFG_USE_SPI_MST_FLASH
#if (!CFG_USE_SPI) || ((!CFG_USE_SPI_MASTER) && (!CFG_USE_SPI_DMA_MASTER))
#error "SPI FLASH NEED CFG_USE_SPI and (CFG_USE_SPI_MASTER or CFG_USE_SPI_DMA_MASTER) ENABLE!!!"
#endif

#define FLASH_PHY_PAGE_SIZE       256
#define FLASH_PHY_SECTOR_SIZE     4096
#define FLASH_PHY_BLK_32K         (32*1024)
#define FLASH_PHY_BLK_64K         (64*1024)

#define CMD_READ_ID               0x9F
#define READ_ID_RESPONE_LEN       3

#define CMD_READ_STATUS_S7_0      0x05
#define CMD_READ_STATUS_S15_8     0x35
#define READ_STATUS_LEN           1
#define FLASH_STATUS_WIP_BIT      (1 << 0)
#define FLASH_STATUS_WEL_BIT      (1 << 1)
#define FLASH_STATUS_PRETECT_MASK (0x3F << 2)

#define CMD_WRITE_STATUS          0x01

#define ERASE_MODE_ALL            0x01
#define ERASE_MODE_BLOCK_64K      0x02
#define ERASE_MODE_BLOCK_32K      0x03
#define ERASE_MODE_SECTOR         0x04

#define CMD_ERASE_ALL             0xc7  // 0x60
#define CMD_ERASE_BLK_64K         0xD8
#define CMD_ERASE_BLK_32K         0x52
#define CMD_ERASE_SECTOR          0x20

#define DELAY_WHEN_BUSY_MS        (2)
#define DELAY_WHEN_WEL_EN         (20)

#define CMD_READ_DATA             0x03
#define CMD_FAST_READ_DATA        0x0B
#define CMD_PAGE_PROG             0x02

#define CMD_WRITE_ENABLE          0x06
#define CMD_WRITE_DISABLE         0x04

#define FAST_READ_EN              (1)

#define USE_SPI_DMA               (1)
#if ((CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U))
#if USE_SPI_DMA
#undef USE_SPI_DMA
#define USE_SPI_DMA               (0)
#endif
#endif

static void spi_flash_send_command(UINT8 cmd)
{
    UINT8 ucmd = cmd;
    struct spi_message msg;

    os_memset(&msg, 0, sizeof(struct spi_message));

    msg.send_buf = &ucmd;
    msg.send_len = sizeof(ucmd);
    msg.recv_buf = NULL;
    msg.recv_len = 0;
    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif
}

UINT16 spi_flash_read_status(void)
{
    UINT16 ustatus = 0;
    UINT8 ustatus_buf[READ_STATUS_LEN] = {0};
    UINT8 ustatus_cmd[] = {CMD_READ_STATUS_S7_0};
    struct spi_message msg;

    os_memset(&msg, 0, sizeof(struct spi_message));
    msg.send_buf = ustatus_cmd;
    msg.send_len = sizeof(ustatus_cmd);
    msg.recv_buf = ustatus_buf;
    msg.recv_len = READ_STATUS_LEN;
    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif
    ustatus = ustatus_buf[0];

    ustatus_buf[0] = 0;
    ustatus_cmd[0] = CMD_READ_STATUS_S15_8;
    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif
    ustatus |= (ustatus_buf[0]) << 8;

    return ustatus;
}

static UINT32 spi_flash_is_busy(void)
{
    UINT8 ustatus_buf[READ_STATUS_LEN] = {0};
    UINT8 ustatus_cmd[] = {CMD_READ_STATUS_S7_0};
    struct spi_message msg;

    os_memset(&msg, 0, sizeof(struct spi_message));
    msg.send_buf = ustatus_cmd;
    msg.send_len = sizeof(ustatus_cmd);
    msg.recv_buf = ustatus_buf;
    msg.recv_len = READ_STATUS_LEN;
    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif

    return (ustatus_buf[0] & FLASH_STATUS_WIP_BIT);
}

static UINT32 spi_flash_wait_busy_to_idle(void)
{
    while(spi_flash_is_busy())
    {
        rtos_delay_milliseconds(DELAY_WHEN_BUSY_MS);
    }

    return 0;
}

static UINT32 spi_flash_wait_wel_enable(UINT32 timeout)
{
    UINT32 start_time = rtos_get_time();
    UINT32 cur_time = 0;

    while(spi_flash_is_busy())
    {
        cur_time = rtos_get_time();
        if((UINT32)(cur_time - start_time) > timeout)
        {
            BK_SPI_FATAL("spi_flash_wait_wel_enable timeout!\r\n");
            return 1;
        }
    }

    return 0;
}

static void spi_flash_write_status(UINT16 ustatus)
{
    UINT8 ustatus_cmd[] = {CMD_WRITE_STATUS, 0x00, 0x00};
    struct spi_message msg;

    os_memset(&msg, 0, sizeof(struct spi_message));
    ustatus_cmd[1] = ustatus & 0xff;
    ustatus_cmd[2] = (ustatus >> 8) & 0xff;

    msg.send_buf = ustatus_cmd;
    msg.send_len = sizeof(ustatus_cmd);
    msg.recv_buf = NULL;
    msg.recv_len = 0;

    spi_flash_wait_busy_to_idle();

    spi_flash_send_command(CMD_WRITE_ENABLE);

    if(spi_flash_wait_wel_enable(DELAY_WHEN_WEL_EN) == 0)
    {
        #if(USE_SPI_DMA)
        bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
        #else
        bk_spi_master_xfer(&msg);
        #endif
    }
}

static void spi_flash_earse(UINT32 addr, UINT32 mode)
{
    struct spi_message msg;
    UINT8 ucmd[] = {0x00, 0x00, 0x00, 0x00};
    UINT32 send_len;

    os_memset(&msg, 0, sizeof(struct spi_message));

    if(mode == ERASE_MODE_ALL)
    {
        ucmd[0] = CMD_ERASE_ALL;
        send_len = 1;
    }
    else
    {
        if(mode == ERASE_MODE_BLOCK_64K)
        {
            ucmd[0] = CMD_ERASE_BLK_64K;
        }
        else if(mode == ERASE_MODE_BLOCK_32K)
        {
            ucmd[0] = CMD_ERASE_BLK_32K;
        }
        else if(mode == ERASE_MODE_SECTOR)
        {
            ucmd[0] = CMD_ERASE_SECTOR;
        }
        else
        {
            BK_SPI_FATAL("earse wrong mode:%d\r\n", mode);
            return;
        }

        ucmd[1] = ((addr >> 16) & 0xff);
        ucmd[2] = ((addr >> 8) & 0xff);
        ucmd[3] = (addr & 0xff);
        send_len = 4;
    }

    msg.send_buf = ucmd;
    msg.send_len = send_len;
    msg.recv_buf = NULL;
    msg.recv_len = 0;

    spi_flash_wait_busy_to_idle();

    spi_flash_send_command(CMD_WRITE_ENABLE);

    if(spi_flash_wait_wel_enable(DELAY_WHEN_WEL_EN) == 0)
    {
        #if(USE_SPI_DMA)
        bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
        #else
        bk_spi_master_xfer(&msg);
        #endif
    }
}

#if !FAST_READ_EN
static int spi_flash_read_page(UINT32 addr, UINT32 size, UINT8 *dst)
{
    struct spi_message msg;
    UINT8 ucmd[] = {CMD_READ_DATA, 0x00, 0x00, 0x00};

    if(dst == NULL)
        return 1;

    if(size > FLASH_PHY_PAGE_SIZE)
        return 1;

    if(size == 0)
        return 0;

    os_memset(&msg, 0, sizeof(struct spi_message));
    ucmd[1] = ((addr >> 16) & 0xff);
    ucmd[2] = ((addr >> 8) & 0xff);
    ucmd[3] = (addr & 0xff);

    msg.send_buf = ucmd;
    msg.send_len = sizeof(ucmd);

    msg.recv_buf = dst;
    msg.recv_len = size;

    spi_flash_wait_busy_to_idle();

    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif

    return 0;
}
#else
static int spi_flash_fast_read_page(UINT32 addr, UINT32 size, UINT8 *dst)
{
    struct spi_message msg;
    UINT8 ucmd[] = {CMD_FAST_READ_DATA, 0x00, 0x00, 0x00, 0xFF};

    if(dst == NULL)
        return 1;

    if(size > FLASH_PHY_PAGE_SIZE)
        return 1;

    if(size == 0)
        return 0;

    os_memset(&msg, 0, sizeof(struct spi_message));
    ucmd[1] = ((addr >> 16) & 0xff);
    ucmd[2] = ((addr >> 8) & 0xff);
    ucmd[3] = (addr & 0xff);

    msg.send_buf = ucmd;
    msg.send_len = sizeof(ucmd);

    msg.recv_buf = dst;
    msg.recv_len = size;

    spi_flash_wait_busy_to_idle();

    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif

    return 0;
}
#endif

static int spi_flash_program_page(UINT32 addr, UINT32 size, UINT8 *src)
{
    struct spi_message msg;
    UINT8 *ucmd;

    BK_SPI_PRT("spi_flash_program_page: addr=0x%08X, size=%d, src=0x%08X\r\n", addr, size, src);
    if(src == NULL)
        return 1;

    if(size > FLASH_PHY_PAGE_SIZE)
        return 1;

    if(size == 0)
        return 0;

    ucmd = os_malloc(size + 4);
    if(!ucmd)
        return 1;

    os_memset(&msg, 0, sizeof(struct spi_message));
    os_memset(ucmd, 0, size + 4);

    ucmd[0] = CMD_PAGE_PROG;
    ucmd[1] = ((addr >> 16) & 0xff);
    ucmd[2] = ((addr >> 8) & 0xff);
    ucmd[3] = (addr & 0xff);
    os_memcpy(&ucmd[4], src, size);

    msg.send_buf = ucmd;
    msg.send_len = size + 4;
    msg.recv_buf = NULL;
    msg.recv_len = 0;

    spi_flash_wait_busy_to_idle();

    spi_flash_send_command(CMD_WRITE_ENABLE);

    if(spi_flash_wait_wel_enable(DELAY_WHEN_WEL_EN) == 0)
    {
        #if(USE_SPI_DMA)
        bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
        #else
        bk_spi_master_xfer(&msg);
        #endif
        os_free(ucmd);
        return 0;
    }
    else
    {
        os_free(ucmd);
        return 1;
    }
}

static void spi_flash_init_extral_gpio(void)
{
    #if ((CFG_SOC_NAME != SOC_BK7238) && (CFG_SOC_NAME != SOC_BK7231N))
    bk_gpio_config_output(SPI_FLASH_WP_GPIO_NUM);
    bk_gpio_output(SPI_FLASH_WP_GPIO_NUM, GPIO_INT_LEVEL_HIGH);

    bk_gpio_config_output(SPI_FLASH_HOLD_GPIO_NUM);
    bk_gpio_output(SPI_FLASH_HOLD_GPIO_NUM, GPIO_INT_LEVEL_HIGH);
    #endif
}

static void spi_flash_enable_voltage(void)
{
    #if (CFG_SOC_NAME == SOC_BK7252N)
    saradc_config_vddram_voltage(PSRAM_VDD_3_5V);
    #elif ((CFG_SOC_NAME != SOC_BK7238) && (CFG_SOC_NAME != SOC_BK7231N))
    UINT32 param;
    param = BLK_BIT_MIC_QSPI_RAM_OR_FLASH;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_ENABLE, &param);

    param = QSPI_IO_3_3V;
    sddev_control(SCTRL_DEV_NAME, CMD_QSPI_IO_VOLTAGE, &param);

    param = PSRAM_VDD_3_3V_DEF;
    sddev_control(SCTRL_DEV_NAME, CMD_QSPI_VDDRAM_VOLTAGE, &param);
    #endif
}

static void spi_flash_disable_voltage(void)
{
    #if (CFG_SOC_NAME == SOC_BK7252N)
    saradc_disable_vddram_voltage();
    #elif ((CFG_SOC_NAME != SOC_BK7238) && (CFG_SOC_NAME != SOC_BK7231N))
    UINT32 param;

    param = BLK_BIT_MIC_QSPI_RAM_OR_FLASH;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_DISABLE, &param);
    #endif
}

/////////////////////////////////////////////////////
// spi flash extern interface
int spi_flash_init(UINT32 rate)
{
    spi_flash_enable_voltage();
    spi_flash_init_extral_gpio();

    if(rate == 0)
        rate = SPI_DEF_CLK_HZ;

    SPI_CFG_ST cfg;
    cfg.u.value = 0;
    cfg.u.slave = 0;
    /* can not set to 0 for spi flash, atleast set 1, otherwise cmd_0x6 can not send corrently */
    cfg.u.interval = 0x1;
    #if(USE_SPI_DMA)
    cfg.u.dma = 1;
    return bk_spi_master_dma_init(rate, cfg.u.value);
    #else
    return bk_spi_master_init(rate, cfg.u.value);
    #endif
}

void spi_flash_deinit(void)
{
    spi_flash_disable_voltage();
    #if(USE_SPI_DMA)
    bk_spi_master_dma_deinit(1);
    #else
    bk_spi_master_deinit();
    #endif
}

UINT32 spi_flash_read_id(void)
{
    UINT32 uid = 0;
    UINT8 uid_buf[READ_ID_RESPONE_LEN] = {0};
    UINT8 uid_cmd[] = {CMD_READ_ID};
    struct spi_message msg;

    os_memset(&msg, 0, sizeof(struct spi_message));
    msg.send_buf = uid_cmd;
    msg.send_len = sizeof(uid_cmd);
    msg.recv_buf = uid_buf;
    msg.recv_len = READ_ID_RESPONE_LEN;

    #if(USE_SPI_DMA)
    bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
    #else
    bk_spi_master_xfer(&msg);
    #endif

    uid = (uid_buf[0] << 16) | (uid_buf[1] << 8) | (uid_buf[2]);

    BK_SPI_PRT("uid:%06x\r\n", uid);

    return uid;
}

int spi_flash_read(UINT32 addr, UINT32 size, UINT8 *dst)
{
    if(dst == NULL)
        return 1;

    if(size == 0)
    {
        return 0;
    }

    for(int i=0; i<size; )
    {
        int ret;
        UINT32 dsize;

        if((size - i) >= FLASH_PHY_PAGE_SIZE)
            dsize = FLASH_PHY_PAGE_SIZE;
        else
            dsize = size - i;

        #if !FAST_READ_EN
        ret = spi_flash_read_page(addr, dsize, dst);
        #else
        ret = spi_flash_fast_read_page(addr, dsize, dst);
        #endif
        if(ret)
        {
            BK_SPI_PRT("spiff read page err:%d\r\n", ret);
            return 1;
        }

        addr = addr + dsize;
        dst = dst + dsize;
        i = i + dsize;
    }

    return 0;
}

int spi_flash_write(UINT32 addr, UINT32 size, UINT8 *src)
{
    UINT32 head_addr = 0, head_size = 0;
    UINT32 mid_addr = 0, mid_size = 0;
    UINT32 tail_addr = 0, tail_size = 0;
    int ret;

    BK_SPI_PRT("spi_flash_write: addr=0x%08X, size=%d, src=0x%08X\r\n", addr, size, src);
    if(src == NULL)
        return 1;

    if(size == 0)
    {
        return 0;
    }

    if (addr%FLASH_PHY_PAGE_SIZE)
    {
        head_addr = addr;
        head_size = ( size > (FLASH_PHY_PAGE_SIZE - (addr%FLASH_PHY_PAGE_SIZE)) )?  (FLASH_PHY_PAGE_SIZE - (addr%FLASH_PHY_PAGE_SIZE)) : size;
        size -= head_size;
    }

    if (size >= FLASH_PHY_PAGE_SIZE)
    {
        mid_addr = addr + head_size;
        mid_size = (size / FLASH_PHY_PAGE_SIZE) * FLASH_PHY_PAGE_SIZE;
        size -= mid_size;
    }

    if (size > 0)
    {
        tail_addr = addr + head_size + mid_size;
        tail_size = size;
    }

    BK_SPI_PRT("[%s] h_addr=0x%08X, h_size=%u, m_addr=0x%08X, m_size=%u, t_addr=0x%08X, t_size=%u.\r\n", __func__, head_addr, head_size, mid_addr, mid_size, tail_addr, tail_size);

    if (head_size)
    {
        ret = spi_flash_program_page(head_addr, head_size, src);
        if(ret)
        {
            BK_SPI_PRT("head: spi flash write page err[%d]!\r\n", ret);
            return 1;
        }
    }

    if (mid_size)
    {
        UINT32 src_addr = mid_addr;
        UINT8 *src_data = src+head_size;

        for(UINT32 index=0; index<mid_size; index+=FLASH_PHY_PAGE_SIZE)
        {
            ret = spi_flash_program_page(src_addr+index, FLASH_PHY_PAGE_SIZE, src_data+index);
            if(ret)
            {
                BK_SPI_PRT("middle: spi flash write page err[%d]!\r\n", ret);
                return 1;
            }
        }
    }

    if (tail_size)
    {
        ret = spi_flash_program_page(tail_addr, tail_size, src+head_size+mid_size);
        if(ret)
        {
            BK_SPI_PRT("tail: spi flash write page err[%d]!\r\n", ret);
            return 1;
        }
    }

    return 0;
}

int spi_flash_erase(UINT32 addr, UINT32 size)
{
    int left_size = (int)size;

    while (left_size > 0)
    {
        UINT32 erase_size = 0, erase_mode;

        if(left_size <= 4 * 1024)
        {
            erase_size = 4 * 1024;
            erase_mode = ERASE_MODE_SECTOR;
        }
        else if(size <= 32 * 1024)
        {
            erase_size = 32 * 1024;
            erase_mode = ERASE_MODE_BLOCK_32K;
        }
        else
        {
            erase_size = 64 * 1024;
            erase_mode = ERASE_MODE_BLOCK_64K;
        }

        spi_flash_earse(addr, erase_mode);

        if(addr & (erase_size - 1))
        {
            size = erase_size - (addr & (erase_size - 1));
        }
        else
        {
            size = erase_size;
        }

        left_size -= size;
        addr += size;
    }

    return 0;
}

void spi_flash_protect(void)
{
    spi_flash_write_status(0x003C);
}

void spi_flash_unprotect(void)
{
    spi_flash_write_status(0x0000);
}
#endif  // BEKEN_USING_SPI_FLASH

