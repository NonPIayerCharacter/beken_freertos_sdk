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
#include "arm_arch.h"
#include "icu_pub.h"
#include "sys_config.h"

#if(CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
#include "spi_bk7231n.h"
#include "spi_pub.h"
#include "gpio_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "mem_pub.h"
#include "error.h"
#include "rtos_pub.h"
#include "spi_struct.h"
#include "general_dma_pub.h"
#include "general_dma.h"

#define SPI_RX_DMA_CHANNEL     GDMA_CHANNEL_1
#define SPI_TX_DMA_CHANNEL     GDMA_CHANNEL_3

#if ((CFG_USE_SPI_DMA_SLAVE) && (CFG_USE_SPI))

#define SPI_SLAVE_DMA_BUF_LEN          264
#define USE_ALLOC_BUF                   (1 << 4)

struct spi_mdev *spi_slave_dma_dev;
UINT8 *spi_slave_dma_tx_buf = NULL, *spi_slave_dma_rx_buf = NULL;
void spi_txfifo_clr(void);
void spi_rxfifo_clr(void);
UINT32 spi_ctrl(UINT32 cmd, void *param);

static void spi_slave_dma_tx_finish_callback(int port, void *param)
{
    uint32_t flag_param;
    struct spi_mdev *spi_dev = spi_slave_dma_dev;

    if ((spi_slave_dma_dev->tx_req_len) && (spi_dev->flag & TX_FINISH_FLAG) == 0) {
        spi_dev->flag |= TX_FINISH_FLAG;

        flag_param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&flag_param);

        rtos_set_semaphore(&spi_dev->tx_sem);
    }
}

static void spi_slave_dma_rx_finish_callback(int port, void *param)
{
    uint32_t flag_param;
    struct spi_mdev *spi_dev = spi_slave_dma_dev;

    if ((spi_slave_dma_dev->rx_len) && (spi_dev->flag & RX_FINISH_FLAG) == 0) {
        spi_dev->flag |= RX_FINISH_FLAG;

        flag_param = 0;
        spi_ctrl(CMD_SPI_RX_EN, (void *)&flag_param);
        rtos_set_semaphore(&spi_dev->rx_sem);
    }
}

static void spi_slave_dma_csn_end_callback(int is_rx_end, void *param)
{
    uint32_t flag_param;
    struct spi_mdev *spi_dev = spi_slave_dma_dev;

    if(is_rx_end)
    {
        if ((spi_slave_dma_dev->tx_req_len) && (spi_dev->flag & TX_FINISH_FLAG) == 0) {
            spi_dev->flag |= TX_FINISH_FLAG;

            flag_param = 0;
            spi_ctrl(CMD_SPI_TX_EN, (void *)&flag_param);

            rtos_set_semaphore(&spi_dev->tx_sem);
        }

        if ((spi_slave_dma_dev->rx_len) && (spi_dev->flag & RX_FINISH_FLAG) == 0) {
            spi_dev->flag |= RX_FINISH_FLAG;

            flag_param = 0;
            spi_ctrl(CMD_SPI_RX_EN, (void *)&flag_param);
            rtos_set_semaphore(&spi_dev->rx_sem);
        }
    }
}

static void spi_slave_dma_spi_configure(UINT32 rate)
{
    UINT32 param;

    /* data bit width */
    if(spi_slave_dma_dev->cfg.u.wdth)
        param = 1;
    else
        param = 0;
    spi_ctrl(CMD_SPI_SET_BITWIDTH, (void *)&param);

    /* baudrate */
    BK_SPI_PRT("max_hz = %d \n", rate);
    spi_ctrl(CMD_SPI_SET_CKR, (void *)&rate);

    /* mode */
    if (spi_slave_dma_dev->cfg.u.cpol) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPOL, (void *)&param);

    /* CPHA */
    if (spi_slave_dma_dev->cfg.u.cpha) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPHA, (void *)&param);

    /* slave */
    param = 0;
    spi_ctrl(CMD_SPI_SET_MSTEN, (void *)&param);

    // 4line :7231N nssms is 0
    if (spi_slave_dma_dev->cfg.u.line) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_LINE_MODE, (void *)&param);

    if (spi_slave_dma_dev->cfg.u.lsb) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_LSB_EN, (void *)&param);

    param = 0;
    spi_ctrl(CMD_SPI_INIT_MSTEN, (void *)&param);

    /*spi data port has a default data, hardware issue*/
    spi_txfifo_clr();
    spi_rxfifo_clr();

    // /* set call back func */
    struct spi_callback_des spi_dev_cb;
    spi_dev_cb.callback = spi_slave_dma_rx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_slave_dma_tx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_slave_dma_csn_end_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_CALLBACK, (void *)&spi_dev_cb);

    //disable tx/rx int disable
    param = 0;
    spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);
    spi_ctrl(CMD_SPI_RXINT_EN, (void *)&param);

    //disable rx/tx finish enable bit
    param = 0;
    spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
    spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);

    //disable rx/tx over
    param = 0;
    spi_ctrl(CMD_SPI_RXOVR_EN, (void *)&param);
    spi_ctrl(CMD_SPI_TXOVR_EN, (void *)&param);

    //disable CSN intterrupt
    param = 0;
    spi_ctrl(CMD_SPI_CS_EN, (void *)&param);

    /* spi master as the sender, and configure the tx/rx threshold level in this way*/
    param = 3;/* 3: generate tx isr when tx fifo count is less than 48*/
    spi_ctrl(CMD_SPI_TXINT_MODE, (void *)&param);
    param = 1;/* 1: generate rx isr when rx fifo count is more than 16*/
    spi_ctrl(CMD_SPI_RXINT_MODE, (void *)&param);

    /* enable spi */
    param = 1;
    spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);
}

static int spi_slave_dma_tx_dma_init(struct spi_message *spi_msg)
{
    GDMACFG_TPYES_ST init_cfg;
    GDMA_CFG_ST en_cfg;

    BK_SPI_PRT("spi_slave_dma_tx_dma_init\r\n");
    os_memset(&init_cfg, 0, sizeof(GDMACFG_TPYES_ST));
    os_memset(&en_cfg, 0, sizeof(GDMA_CFG_ST));

    init_cfg.dstdat_width = 8;
    init_cfg.srcdat_width = 32;
    init_cfg.dstptr_incr =  0;
    init_cfg.srcptr_incr =  1;

    init_cfg.src_start_addr = spi_slave_dma_dev->tx_ptr;
    init_cfg.dst_start_addr = (void *)SPI_DAT;

    init_cfg.channel = SPI_TX_DMA_CHANNEL ;
    init_cfg.prio = 0;
    init_cfg.u.type4.src_loop_start_addr = spi_slave_dma_dev->tx_ptr;
    init_cfg.u.type4.src_loop_end_addr = spi_slave_dma_dev->tx_ptr + spi_slave_dma_dev->tx_req_len;

    init_cfg.half_fin_handler = NULL;
    init_cfg.fin_handler = NULL;

    init_cfg.src_module = GDMA_X_SRC_DTCM_RD_REQ;
    init_cfg.dst_module = GDMA_X_DST_GSPI_TX_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE4, (void *)&init_cfg);

    en_cfg.channel = SPI_TX_DMA_CHANNEL;
    en_cfg.param = spi_slave_dma_dev->tx_req_len; // dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);

    en_cfg.channel = SPI_TX_DMA_CHANNEL;
    en_cfg.param = 0; // 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);

    en_cfg.channel = SPI_TX_DMA_CHANNEL;
    en_cfg.param = 0; // src no loop
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_SRCADDR_LOOP, &en_cfg);

    return 0;
}

static void spi_dma_rx_fin_handler(UINT32 param)
{
    //os_printf("spi_dma half handler\r\n");
}

static void spi_slave_dma_rx_init(struct spi_message *spi_msg)
{
    GDMACFG_TPYES_ST init_cfg;
    GDMA_CFG_ST en_cfg;

    os_memset(&init_cfg, 0, sizeof(GDMACFG_TPYES_ST));
    os_memset(&en_cfg, 0, sizeof(GDMA_CFG_ST));

    init_cfg.dstdat_width = 32;
    init_cfg.srcdat_width = 8;
    init_cfg.dstptr_incr =  1;
    init_cfg.srcptr_incr =  0;

    init_cfg.src_start_addr = (void *)SPI_DAT;
    init_cfg.dst_start_addr = spi_slave_dma_dev->rx_ptr;

    init_cfg.channel = SPI_RX_DMA_CHANNEL;
    init_cfg.prio = 0;
    init_cfg.u.type5.dst_loop_start_addr = spi_slave_dma_dev->rx_ptr;
    init_cfg.u.type5.dst_loop_end_addr = spi_slave_dma_dev->rx_ptr + spi_slave_dma_dev->rx_len;

    init_cfg.half_fin_handler = NULL;
    init_cfg.fin_handler = spi_dma_rx_fin_handler;

    init_cfg.src_module = GDMA_X_SRC_GSPI_RX_REQ;
    init_cfg.dst_module = GDMA_X_DST_DTCM_WR_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE5, (void *)&init_cfg);

    en_cfg.channel = SPI_RX_DMA_CHANNEL;
    en_cfg.param   = spi_slave_dma_dev->rx_len; // dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);

    en_cfg.channel = SPI_RX_DMA_CHANNEL;
    en_cfg.param = 0; // 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);

    en_cfg.param = 0; // src no loop:important
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_DSTADDR_LOOP, &en_cfg);
}

static void spi_slave_dma_spi_unconfigure(void)
{
    spi_ctrl(CMD_SPI_DEINIT_MSTEN, NULL);
}

static int spi_slave_dma_xfer_start_setting(struct spi_message *msg)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    spi_slave_dma_dev->tx_ptr = msg->send_buf;
    spi_slave_dma_dev->tx_remain_cnt = msg->send_len;
    spi_slave_dma_dev->tx_req_len = msg->send_len;

    spi_slave_dma_dev->rx_ptr = msg->recv_buf;
    spi_slave_dma_dev->rx_len = msg->recv_len;
    spi_slave_dma_dev->rx_offset = 0;
    spi_slave_dma_dev->rx_overflow_cnt = 0;
    spi_slave_dma_dev->rx_drop = 0;

    spi_slave_dma_dev->total_len = 0;
    spi_slave_dma_dev->flag &= ~(TX_FINISH_FLAG | RX_FINISH_FLAG);
    GLOBAL_INT_RESTORE();

    return 1;
}

//static int spi_slave_dma_xfer_complete_setting(struct spi_message *msg)
int spi_slave_dma_xfer_complete_setting(struct spi_message *msg)
{
    GLOBAL_INT_DECLARATION();

    if((spi_slave_dma_dev->tx_ptr) && (spi_slave_dma_dev->flag & (USE_ALLOC_BUF))) {
        os_free(spi_slave_dma_dev->tx_ptr);
    }

    if((spi_slave_dma_dev->rx_ptr) && (spi_slave_dma_dev->flag & (USE_ALLOC_BUF))) {
        os_free(spi_slave_dma_dev->rx_ptr);
    }

    GLOBAL_INT_DISABLE();
    spi_slave_dma_dev->tx_ptr = NULL;
    spi_slave_dma_dev->tx_remain_cnt = 0;
    spi_slave_dma_dev->tx_req_len = 0;

    spi_slave_dma_dev->rx_ptr = NULL;
    spi_slave_dma_dev->rx_len = 0;

    spi_slave_dma_dev->total_len = 0;
    spi_slave_dma_dev->flag |= TX_FINISH_FLAG;
    GLOBAL_INT_RESTORE();

    return 0;
}

static void bk_spi_slave_dma_set_enable(struct spi_message *msg, UINT8 enable)
{
    int param;
    GDMA_CFG_ST en_cfg;

    if (enable) {
        en_cfg.param = 1;
        param = 1;
    } else {
        en_cfg.param = 0;
        param = 0;
    }

    if(msg->send_len) {
        en_cfg.channel = SPI_TX_DMA_CHANNEL;
        sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
    }

    if(msg->recv_len) {
        en_cfg.channel = SPI_RX_DMA_CHANNEL;
        sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
    }

    //enable rx
    if((msg->recv_len) && (msg->recv_len)) {
        spi_ctrl(CMD_SPI_TRX_EN, (void *)&param);
    } else if(msg->send_len) {
        spi_ctrl(CMD_SPI_TX_EN, (void *)&param);
    } else if(msg->send_len) {
        spi_ctrl(CMD_SPI_RX_EN, (void *)&param);
    }
}

int bk_spi_slave_dma_xfer(struct spi_message *msg)
{
    OSStatus err;
    UINT32 param, total_size;

    ASSERT(spi_slave_dma_dev != NULL);
    ASSERT(msg != NULL);

    rtos_lock_mutex(&spi_slave_dma_dev->mutex);

    total_size = msg->recv_len + msg->send_len;
    spi_txfifo_clr();
    spi_rxfifo_clr();

    if (total_size) {
        spi_slave_dma_xfer_start_setting(msg);

        param = 1;
        spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);

        /* disable rx/tx over*/
        param = 0;
        spi_ctrl(CMD_SPI_RXOVR_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXOVR_EN, (void *)&param);

        /*set trans len rx equ to tx to prevnt unexpected clock*/
        if(msg->send_len) {
            spi_slave_dma_tx_dma_init(msg);
            spi_ctrl(CMD_SPI_TXTRANS_EN, (void *)&msg->send_len);
            param = 1;
            spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
        }

        if(msg->recv_len) {
            spi_slave_dma_rx_init(msg);
            spi_ctrl(CMD_SPI_RXTRANS_EN, (void *)&msg->recv_len);
            param = 1;
            spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);
        }

        //enable CSN intterrupt
        param = 1;
        spi_ctrl(CMD_SPI_CS_EN, (void *)&param);

        bk_spi_slave_dma_set_enable(msg, 1);

        os_printf("spi_slave [CTRL]:0x%08x \n", REG_READ(SPI_CTRL));
        os_printf("spi_slave [CONFIG]:0x%08x \n", REG_READ(SPI_CONFIG));

        /* wait tx finish */
        if(msg->send_len) {
            err = rtos_get_semaphore(&spi_slave_dma_dev->tx_sem, BEKEN_NEVER_TIMEOUT);
        }
        if(msg->recv_len) {
            err = rtos_get_semaphore(&spi_slave_dma_dev->rx_sem, BEKEN_NEVER_TIMEOUT);
        }

        //disable tx-rx
        bk_spi_slave_dma_set_enable(msg, 0);

        /* disable tx & rx interrupt again */
        param = 0;
        spi_ctrl(CMD_SPI_RXINT_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
        spi_ctrl(CMD_SPI_CS_EN, (void *)&param);
        spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);

        /* initial spi_slave_dma_dev with zero*/
        do {
            err = rtos_get_semaphore(&spi_slave_dma_dev->tx_sem, 0);
        } while (err == kNoErr);
        do {
            err = rtos_get_semaphore(&spi_slave_dma_dev->rx_sem, 0);
        } while (err == kNoErr);

        spi_slave_dma_xfer_complete_setting(msg);
    }

    rtos_unlock_mutex(&spi_slave_dma_dev->mutex);

    return msg->recv_len;
}

int bk_spi_slave_dma_init(UINT32 rate, UINT32 mode)
{
    OSStatus result = 0;
    SPI_CFG_ST cfg;
    cfg.u.value = mode;

    if (spi_slave_dma_dev)
        bk_spi_slave_dma_deinit();

    spi_slave_dma_tx_buf = os_zalloc(SPI_SLAVE_DMA_BUF_LEN);
    if(!spi_slave_dma_tx_buf) {
        BK_SPI_PRT("[spi]:malloc memory for spi_slave_dma_tx_buf\n");
        result = -1;
        goto _exit;
    }

    spi_slave_dma_rx_buf = os_zalloc(SPI_SLAVE_DMA_BUF_LEN);
    if(!spi_slave_dma_rx_buf) {
        BK_SPI_PRT("[spi]:malloc memory for spi_slave_dma_rx_buf\n");
        result = -1;
        goto _exit;
    }

    spi_slave_dma_dev = os_zalloc(sizeof(struct spi_mdev));
    if (!spi_slave_dma_dev) {
        BK_SPI_PRT("[spi]:malloc memory for spi_slave_dma_dev failed\n");
        result = -1;
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_slave_dma_dev->tx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi tx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_slave_dma_dev->rx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi rx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_mutex(&spi_slave_dma_dev->mutex);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi mutex init failed\n");
        goto _exit;
    }

    cfg.u.slave = 1;
    cfg.u.wdth = 0;
    cfg.u.line = 0;
    spi_slave_dma_dev->cfg = cfg;

    spi_slave_dma_dev->tx_ptr = NULL;
    spi_slave_dma_dev->tx_remain_cnt = 0;
    spi_slave_dma_dev->flag |= TX_FINISH_FLAG;

    spi_slave_dma_spi_configure(rate);

    return 0;

_exit:
    if(spi_slave_dma_tx_buf) {
        os_free(spi_slave_dma_tx_buf);
        spi_slave_dma_tx_buf = NULL;
    }

    if(spi_slave_dma_rx_buf) {
        os_free(spi_slave_dma_rx_buf);
        spi_slave_dma_rx_buf = NULL;
    }

    if (spi_slave_dma_dev->mutex)
        rtos_deinit_mutex(&spi_slave_dma_dev->mutex);

    if (spi_slave_dma_dev->tx_sem)
        rtos_deinit_semaphore(&spi_slave_dma_dev->tx_sem);

    if (spi_slave_dma_dev) {
        os_free(spi_slave_dma_dev);
        spi_slave_dma_dev = NULL;
    }

    return 1;
}

int bk_spi_slave_dma_deinit(void)
{
    if (spi_slave_dma_dev == NULL)
        return 0;

    if (spi_slave_dma_dev->mutex)
        rtos_lock_mutex(&spi_slave_dma_dev->mutex);

    if (spi_slave_dma_dev->tx_sem)
        rtos_deinit_semaphore(&spi_slave_dma_dev->tx_sem);

    if (spi_slave_dma_dev->mutex) {
        rtos_unlock_mutex(&spi_slave_dma_dev->mutex);
        rtos_deinit_mutex(&spi_slave_dma_dev->mutex);
    }

    if (spi_slave_dma_dev) {
        os_free(spi_slave_dma_dev);
        spi_slave_dma_dev = NULL;
    }

    if(spi_slave_dma_tx_buf) {
        os_free(spi_slave_dma_tx_buf);
        spi_slave_dma_tx_buf = NULL;
    }

    if(spi_slave_dma_rx_buf) {
        os_free(spi_slave_dma_rx_buf);
        spi_slave_dma_rx_buf = NULL;
    }

    spi_slave_dma_spi_unconfigure();

    return 0;
}
#endif	 // ((CFG_USE_SPI_DMA_SLAVE) && (CFG_USE_SPI))
#endif   // (CFG_SOC_NAME == SOC_BK7231N)
// eof

