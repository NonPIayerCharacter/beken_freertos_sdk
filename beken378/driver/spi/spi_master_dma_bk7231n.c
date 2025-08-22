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
#include "sys_config.h"

#if(CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
#if ((CFG_USE_SPI_DMA_MASTER) && (CFG_USE_SPI))
#include "icu_pub.h"
#include "spi_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "mem_pub.h"
#include "error.h"
#include "rtos_pub.h"
#include "general_dma_pub.h"
#include "general_dma.h"
#include "spi_struct.h"
#include "spi_bk7231n.h"
#include "co_list.h"
#include "bk_timer_pub.h"

enum
{
    SPI_DMA_STATUS_UINITED = 0,
    SPI_DMA_STATUS_INITED,
    SPI_DMA_STATUS_PUASED,
    SPI_DMA_STATUS_UINITING,
};

#define SPI_DMA_CHANNEL                 (GDMA_CHANNEL_3)
#define SPI_MASTER_DELAY_HTIMER_CHNAL   (2)   // need a hardware timer count by us
#define SPI_DMA_ELEM_CNT                (10)  // max data elem in free list
#define SPI_DMA_DELAY_BYTE_CNT          (72)  // max spi fifo 64+8

void spi_txfifo_clr(void);
void spi_rxfifo_clr(void);
UINT32 spi_ctrl(UINT32 cmd, void *param);

typedef struct spi_data_elem_st
{
    struct co_list_hdr hdr;
    UINT8 *tx_buf;
    UINT32 tx_len;
    UINT8 *rx_buf;
    UINT32 rx_len;
    volatile UINT32 flags;
} SPI_D_ELEM_ST, *SPI_D_ELEM_PTR;

typedef struct spi_master_dev_st
{
    SPI_CFG_ST cfg;
    UINT32 status;
    SPI_D_ELEM_ST elem[SPI_DMA_ELEM_CNT];
    struct co_list ready_list;
    struct co_list free_list;
    volatile SPI_D_ELEM_PTR cur_elem;
    UINT32 hw_delay_us;
    beken_semaphore_t finish_sem;
    beken_mutex_t mutex;
    spi_master_dma_fin_cb finish_callback;
} SPI_M_DEV_ST, *SPI_M_DEV_PTR;

static SPI_M_DEV_PTR spi_mdev = NULL;
static void bk_spi_master_dma_trigger_tx_node(void);
static void bk_spi_master_dma_trigger_rx_node(void);
static void bk_spi_master_dma_free_elem(void);

static void spi_master_dma_enable_dma(UINT32 enable)
{
    GDMA_CFG_ST en_cfg;
    en_cfg.channel = SPI_DMA_CHANNEL;
    if (enable)
        en_cfg.param = 1;
    else
        en_cfg.param = 0;
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
}

void spi_master_dma_hw_timer_hdl(UINT8 param)
{
    uint32_t channel = SPI_MASTER_DELAY_HTIMER_CHNAL;
    sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_DISABLE, &channel);

    if(spi_mdev->cur_elem) {
        if((spi_mdev->cur_elem->flags & TX_FINISH_FLAG) == 0) {
            // can't reset tx en here, othersie csn to high
            spi_master_dma_enable_dma(0);
            spi_mdev->cur_elem->flags |= TX_FINISH_FLAG;
            spi_mdev->cur_elem->flags &= ~TX_FINISH_DELAY_FLAG;
        }

        bk_spi_master_dma_trigger_rx_node();
        // reset tx en after check rx node.
        UINT32 flag_param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&flag_param);

        if(spi_mdev->cur_elem->flags & RX_FINISH_FLAG) {
            bk_spi_master_dma_free_elem();
        } else {
            return;
        }
    }
    bk_spi_master_dma_trigger_tx_node();
}

static void spi_master_dma_tx_dma_finish_callback(UINT32 val)
{
    timer_param_t param;
    if(spi_mdev->cur_elem) {
        spi_mdev->cur_elem->flags |= TX_FINISH_DELAY_FLAG;
    }
    param.channel = SPI_MASTER_DELAY_HTIMER_CHNAL;
    param.div = 1;
    param.period = spi_mdev->hw_delay_us;
    param.t_Int_Handler = spi_master_dma_hw_timer_hdl;
    sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM_US, &param);
}

static void spi_master_dma_rx_dma_finish_callback(UINT32 val)
{
    if(spi_mdev->cur_elem) {
        if((spi_mdev->cur_elem->flags & RX_FINISH_FLAG) == 0) {
            UINT32 flag_param = 0;
            spi_ctrl(CMD_SPI_RX_EN, (void *)&flag_param);
            spi_master_dma_enable_dma(0);
            spi_mdev->cur_elem->flags |= RX_FINISH_FLAG;
        }
        bk_spi_master_dma_free_elem();
    }
    bk_spi_master_dma_trigger_tx_node();
}

static void spi_master_dma_tx_finish_callback(int port, void *param)
{
    if(spi_mdev->cur_elem) {
        if((spi_mdev->cur_elem->flags & TX_FINISH_FLAG) == 0) {
            spi_master_dma_enable_dma(0);
            spi_mdev->cur_elem->flags |= TX_FINISH_FLAG;
        }
        bk_spi_master_dma_trigger_rx_node();
        // reset tx en after check rx node.
        UINT32 flag_param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&flag_param);
        if(spi_mdev->cur_elem->flags & RX_FINISH_FLAG) {
            bk_spi_master_dma_free_elem();
        } else {
            return;
        }
    }
    bk_spi_master_dma_trigger_tx_node();
}

static void spi_master_dma_rx_finish_callback(int port, void *param)
{
    if(spi_mdev->cur_elem) {
        if((spi_mdev->cur_elem->flags & RX_FINISH_FLAG) == 0) {
            UINT32 flag_param = 0;
            spi_ctrl(CMD_SPI_RX_EN, (void *)&flag_param);
            spi_master_dma_enable_dma(0);
            spi_mdev->cur_elem->flags |= RX_FINISH_FLAG;
        }
        bk_spi_master_dma_free_elem();
    }
    bk_spi_master_dma_trigger_tx_node();
}

static void spi_master_dma_spi_configure(UINT32 rate)
{
    UINT32 param;

    /* data bit width */
    if(spi_mdev->cfg.u.wdth)
        param = 1;
    else
        param = 0;
    spi_ctrl(CMD_SPI_SET_BITWIDTH, (void *)&param);

    if(spi_mdev->cfg.u.interval)
        param = spi_mdev->cfg.u.interval;
    else
        param = 0x0;
    spi_ctrl(CMD_SPI_SET_BYTE_INTVAL, (void *)&param);

    /* baudrate */
    param = spi_ctrl(CMD_SPI_SET_CKR, (void *)&rate);
    /* calc max delay time for hw timer */
    spi_mdev->hw_delay_us = ((SPI_DMA_DELAY_BYTE_CNT *
                              ((8 * (1 + spi_mdev->cfg.u.wdth) + 1) + spi_mdev->cfg.u.interval))
                             * 1000000) / (param);

    /* mode */
    if (spi_mdev->cfg.u.cpol) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPOL, (void *)&param);

    /* CPHA */
    if (spi_mdev->cfg.u.cpha) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPHA, (void *)&param);

    /* Master */
    param = 1;
    spi_ctrl(CMD_SPI_SET_MSTEN, (void *)&param);

    // 4line :7231N nssms is 0
    if (spi_mdev->cfg.u.line) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_LINE_MODE, (void *)&param);

    if (spi_mdev->cfg.u.lsb) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_LSB_EN, (void *)&param);

    param = 0;
    spi_ctrl(CMD_SPI_INIT_MSTEN, (void *)&param);

    // /* set call back func */
    struct spi_callback_des spi_dev_cb;
    spi_dev_cb.callback = spi_master_dma_rx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_master_dma_tx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

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

    BK_SPI_PRT("spi_master [CTRL]:0x%08x \n", REG_READ(SPI_CTRL));
    BK_SPI_PRT("spi_master [CONFIG]:0x%08x \n", REG_READ(SPI_CONFIG));
    BK_SPI_PRT("max_hz = %d, delay_time= %d us\n", rate, spi_mdev->hw_delay_us);
}

static void spi_master_dma_config_tx_dma(void)
{
    GDMACFG_TPYES_ST init_cfg;
    GDMA_CFG_ST en_cfg;

    os_memset(&init_cfg, 0, sizeof(GDMACFG_TPYES_ST));
    os_memset(&en_cfg, 0, sizeof(GDMA_CFG_ST));

    init_cfg.dstdat_width = 8;
    init_cfg.srcdat_width = 32;
    init_cfg.dstptr_incr =  0;
    init_cfg.srcptr_incr =  1;
    init_cfg.src_start_addr = spi_mdev->cur_elem->tx_buf;
    init_cfg.dst_start_addr = (void *)SPI_DAT;
    init_cfg.channel = SPI_DMA_CHANNEL;
    init_cfg.prio = 0;
    init_cfg.u.type4.src_loop_start_addr = spi_mdev->cur_elem->tx_buf;
    init_cfg.u.type4.src_loop_end_addr = spi_mdev->cur_elem->tx_buf + spi_mdev->cur_elem->tx_len;
    init_cfg.half_fin_handler = NULL;
    init_cfg.fin_handler = spi_master_dma_tx_dma_finish_callback;
    init_cfg.src_module = GDMA_X_SRC_DTCM_RD_REQ;
    init_cfg.dst_module = GDMA_X_DST_GSPI_TX_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE4, (void *)&init_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = spi_mdev->cur_elem->tx_len; // dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = 0; // 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = 0; // src no loop
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_SRCADDR_LOOP, &en_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = 0; // close fin en
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_FIN_INT_ENABLE, &en_cfg);
}

static void spi_master_dma_config_rx_dma(void)
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
    init_cfg.dst_start_addr = spi_mdev->cur_elem->rx_buf;
    init_cfg.channel = SPI_DMA_CHANNEL;
    init_cfg.prio = 0;
    init_cfg.u.type5.dst_loop_start_addr = spi_mdev->cur_elem->rx_buf;
    init_cfg.u.type5.dst_loop_end_addr = spi_mdev->cur_elem->rx_buf + spi_mdev->cur_elem->rx_len;
    init_cfg.half_fin_handler = NULL;
    init_cfg.fin_handler = spi_master_dma_rx_dma_finish_callback;
    init_cfg.src_module = GDMA_X_SRC_GSPI_RX_REQ;
    init_cfg.dst_module = GDMA_X_DST_DTCM_WR_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE5, (void *)&init_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param   = spi_mdev->cur_elem->rx_len; // dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = 0; // 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);
    en_cfg.param = 0; // src no loop:important
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_DSTADDR_LOOP, &en_cfg);
    en_cfg.channel = SPI_DMA_CHANNEL;
    en_cfg.param = 0; // close fin en
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_FIN_INT_ENABLE, &en_cfg);
}

static void spi_master_dma_flush_xfering_elem(void)
{
    SPI_D_ELEM_PTR elem;
    do {
        elem = (SPI_D_ELEM_PTR)co_list_pop_front(&spi_mdev->ready_list);
        if(elem) {
            if(elem->flags & WAIT_DONE_FLAG) {
                elem->flags |= TRX_ABORT_FLAG;
                rtos_set_semaphore(&spi_mdev->finish_sem);
            }
            if((spi_mdev->finish_callback) && (spi_mdev->cfg.u.status)) {
                struct spi_message msg;
                msg.send_buf = elem->tx_buf;
                msg.send_len = elem->tx_len;
                msg.recv_buf = elem->rx_buf;
                msg.recv_len = elem->rx_len;
                UINT32 event = SPI_EVEN_TRX_LOSE;
                spi_mdev->finish_callback(event, &msg);
            }
            co_list_push_back(&spi_mdev->free_list, (struct co_list_hdr *)&elem->hdr);
        }
    } while(elem);
}

static void spi_master_dma_spi_unconfigure(void)
{
    spi_ctrl(CMD_SPI_DEINIT_MSTEN, NULL);
}

static void bk_spi_master_dma_free_elem(void)
{
    SPI_D_ELEM_PTR elem;
    if(spi_mdev->cur_elem)
    {
        struct spi_message msg;
        elem = spi_mdev->cur_elem;
        msg.send_buf = elem->tx_buf;
        msg.send_len = elem->tx_len;
        msg.recv_buf = elem->rx_buf;
        msg.recv_len = elem->rx_len;
        if(elem->flags & WAIT_DONE_FLAG) {
            rtos_set_semaphore(&spi_mdev->finish_sem);
        }
        co_list_extract(&spi_mdev->ready_list, (struct co_list_hdr *)&elem->hdr);
        co_list_push_back(&spi_mdev->free_list, (struct co_list_hdr *)&elem->hdr);

        spi_mdev->cur_elem = NULL;
        if((spi_mdev->finish_callback) && (spi_mdev->cfg.u.status)) {
            UINT32 event = SPI_EVEN_TRX_DONE;
            spi_mdev->finish_callback(event, &msg);
        }
    }
}

static void bk_spi_master_dma_trigger_tx_node(void)
{
    SPI_D_ELEM_PTR elem;

    if(spi_mdev->cur_elem)
        return;

    if(spi_mdev->status != SPI_DMA_STATUS_INITED) {
        return;
    }

    // clear fifo for new node, akthought there is no new node
    spi_txfifo_clr();
    spi_rxfifo_clr();

    elem = (SPI_D_ELEM_PTR)co_list_pick(&spi_mdev->ready_list);
    if(elem) {
        GLOBAL_INT_DECLARATION();
        GLOBAL_INT_DISABLE();
        spi_mdev->cur_elem = elem;
        GLOBAL_INT_RESTORE();
        if(((elem->flags & TX_FINISH_FLAG) == 0) && (elem->tx_buf) && (elem->tx_len)) {
            UINT32 param;

            spi_master_dma_config_tx_dma();
            spi_master_dma_enable_dma(1);

            if((elem->tx_len < 4096)
                    && ((elem->rx_buf == NULL) || (elem->rx_buf == 0))) {
                // use spi tx fin isr
                param = elem->tx_len;
                spi_ctrl(CMD_SPI_TXTRANS_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_TX_EN, (void *)&param);

            } else {
                // two case use dma tx fin + hw_timer isr
                // 1. elem->tx_len >= 4096, 2. has rx data to receive
                GDMA_CFG_ST en_cfg;
                en_cfg.channel = SPI_DMA_CHANNEL;
                en_cfg.param = 1; // close fin en
                sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_FIN_INT_ENABLE, &en_cfg);
                param = 0; // transfor for ever
                spi_ctrl(CMD_SPI_TXTRANS_EN, (void *)&param);
                param = 0;
                spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_TX_EN, (void *)&param);
            }
        } else {
            elem->flags |= TX_FINISH_FLAG;
        }
    }
}

static void bk_spi_master_dma_trigger_rx_node(void)
{
    SPI_D_ELEM_PTR elem;

    if(spi_mdev->status != SPI_DMA_STATUS_INITED) {
        return;
    }

    if(spi_mdev->cur_elem == NULL) {
        GLOBAL_INT_DECLARATION();
        elem = (SPI_D_ELEM_PTR)co_list_pick(&spi_mdev->ready_list);
        GLOBAL_INT_DISABLE();
        spi_mdev->cur_elem = elem;
        GLOBAL_INT_RESTORE();
    } else {
        elem = spi_mdev->cur_elem;
    }

    if(elem) {
        if ((elem->flags & TX_FINISH_FLAG) == 0) {
            return;
        }

        if(((elem->flags & RX_FINISH_FLAG) == 0) && (elem->rx_buf) && (elem->rx_len)) {
            UINT32 param;

            spi_master_dma_enable_dma(0);
            if(elem->rx_len < 4096) {
                // use spi tx fin isr
                param = elem->rx_len;
                spi_ctrl(CMD_SPI_RXTRANS_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_RX_EN, (void *)&param);
                spi_master_dma_config_rx_dma();
                spi_master_dma_enable_dma(1);
            } else {
                param = 0; // transfor for ever
                spi_ctrl(CMD_SPI_RXTRANS_EN, (void *)&param);
                param = 0;
                spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);
                param = 1;
                spi_ctrl(CMD_SPI_RX_EN, (void *)&param);
                spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);

                spi_master_dma_config_rx_dma();
                spi_master_dma_enable_dma(1);
                // use dma tx fin + hw_timer isr
                GDMA_CFG_ST en_cfg;
                en_cfg.channel = SPI_DMA_CHANNEL;
                en_cfg.param = 1; // close fin en
                sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_FIN_INT_ENABLE, &en_cfg);
            }
        } else {
            elem->flags |= RX_FINISH_FLAG;
        }
    }
}

int bk_spi_master_dma_xfer(struct spi_message *msg, UINT32 wait_time_ms)
{
    if((msg->send_buf == NULL) && (msg->recv_buf == NULL)) {
        return -1;
    }
    if((msg->send_len == 0) && (msg->recv_len == 0)) {
        return -2;
    }
    if((msg->send_buf) && (msg->send_len >= 64*1024)) {
        // length less than 64K, for dma limited
        return -3;
    }
    if((msg->recv_buf) && (msg->recv_len >= 64*1024)) {
        // length less than 64K, for dma limited
        return -4;
    }
    if(spi_mdev == NULL) {
        return -5;
    }
    if(spi_mdev->status != SPI_DMA_STATUS_INITED) {
        return -6;
    }

    int ret = 0;
    SPI_D_ELEM_PTR elem = (SPI_D_ELEM_PTR)co_list_pop_front(&spi_mdev->free_list);
    if (elem == NULL) {
        ret = -6;
        goto xfer_end;
    }
    elem->tx_buf = msg->send_buf;
    elem->tx_len = msg->send_len;
    elem->rx_buf = msg->recv_buf;
    elem->rx_len = msg->recv_len;
    elem->flags = 0;
    if(wait_time_ms) {
        elem->flags |= WAIT_DONE_FLAG;
    }
    elem->flags &= ~(TX_FINISH_FLAG | RX_FINISH_FLAG);
    co_list_push_back(&spi_mdev->ready_list, (struct co_list_hdr *)&elem->hdr);
    if(wait_time_ms) {
        OSStatus err;
        // only one finish_sem, if need wait finish, use mutex to protect this finish_sem
        rtos_lock_mutex(&spi_mdev->mutex);
        do {
            err = rtos_get_semaphore(&spi_mdev->finish_sem, 0);
        } while (err == kNoErr);
        bk_spi_master_dma_trigger_tx_node();
        bk_spi_master_dma_trigger_rx_node();
        err = rtos_get_semaphore(&spi_mdev->finish_sem, wait_time_ms);
        if(err != kNoErr) {
            rtos_unlock_mutex(&spi_mdev->mutex);
            ret = -7;
            goto xfer_end;
        } else {
            if(elem->flags & TRX_ABORT_FLAG) {
                rtos_unlock_mutex(&spi_mdev->mutex);
                ret = -8;
                goto xfer_end;
            }
        }
        rtos_unlock_mutex(&spi_mdev->mutex);
    } else {
        bk_spi_master_dma_trigger_tx_node();
        bk_spi_master_dma_trigger_rx_node();
    }
    ret = (msg->send_len + msg->recv_len);
xfer_end:
    return ret;
}

int bk_spi_master_dma_init(UINT32 rate, UINT32 mode)
{
    OSStatus result = 0;
    int ret = 0;
    SPI_CFG_ST cfg;
    cfg.u.value = mode;

    if (spi_mdev != NULL) {
        BK_SPI_PRT("[spi]:ready inited\n");
        return -1;
    }
    spi_mdev = (SPI_M_DEV_PTR)os_zalloc(sizeof(SPI_M_DEV_ST));
    if (!spi_mdev) {
        BK_SPI_PRT("[spi]:malloc memory for SPI_M_DEV_ST failed\n");
        return -2;
    }
    result = rtos_init_semaphore(&spi_mdev->finish_sem, 1);
    if (result != kNoErr) {
        ret = -3;
        BK_SPI_PRT("[spi]: spi fin semp init failed\n");
        goto _exit;
    }
    result = rtos_init_mutex(&spi_mdev->mutex);
    if (result != kNoErr) {
        ret = -4;
        BK_SPI_PRT("[spi]: spi mutex init failed\n");
        goto _exit;
    }

    cfg.u.slave = 0;
    cfg.u.wdth = 0;
    cfg.u.line = 0;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    spi_mdev->cfg = cfg;
    co_list_init(&spi_mdev->ready_list);
    co_list_init(&spi_mdev->free_list);

    for (int i = 0; i < SPI_DMA_ELEM_CNT; i++)
    {
        co_list_push_back(&spi_mdev->free_list,
                          (struct co_list_hdr *)&spi_mdev->elem[i].hdr);
    }
    spi_mdev->cur_elem = NULL;
    spi_mdev->cfg.u.status = 0;
    spi_mdev->finish_callback = NULL;
    spi_mdev->status = SPI_DMA_STATUS_INITED;
    GLOBAL_INT_RESTORE();

    spi_master_dma_spi_configure(rate);

    return 0;

_exit:
    if(spi_mdev) {
        if (spi_mdev->mutex)
            rtos_deinit_mutex(&spi_mdev->mutex);
        if (spi_mdev->finish_sem)
            rtos_deinit_semaphore(&spi_mdev->finish_sem);
        os_free(spi_mdev);
        spi_mdev = NULL;
    }

    return ret;
}

int bk_spi_master_dma_abort_xfer(void)
{
    UINT32 status;
    if (spi_mdev == NULL)
        return -1;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    status = spi_mdev->status;
    spi_mdev->status = SPI_DMA_STATUS_UINITING;
    GLOBAL_INT_RESTORE();

    if(spi_mdev->cur_elem) {
        GLOBAL_INT_DISABLE();
        spi_master_dma_enable_dma(0);
        UINT32 param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&param);
        spi_ctrl(CMD_SPI_RX_EN, (void *)&param);
        spi_mdev->cur_elem->flags &= ~(TX_FINISH_FLAG | RX_FINISH_FLAG | TX_FINISH_DELAY_FLAG);
        spi_mdev->cur_elem = NULL;
        GLOBAL_INT_RESTORE();
    }
    spi_master_dma_flush_xfering_elem();

    GLOBAL_INT_DISABLE();
    spi_mdev->status = status;
    GLOBAL_INT_RESTORE();

    return 0;
}

int bk_spi_master_dma_deinit(UINT32 force_deinit)
{
    if (spi_mdev == NULL)
        return 0;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    spi_mdev->status = SPI_DMA_STATUS_UINITING;
    GLOBAL_INT_RESTORE();

    if(force_deinit == 0) {
        if(spi_mdev->cur_elem) {
            return -1;
        }
        if(!co_list_is_empty(&spi_mdev->ready_list)) {
            return -2;
        }
    }

    spi_master_dma_enable_dma(0);
    spi_master_dma_spi_unconfigure();
    if(force_deinit) {
        // flush node, call use callback, notify SPI_EVEN_TRX_LOSE
        spi_master_dma_flush_xfering_elem();
    }

    GLOBAL_INT_DISABLE();
    spi_mdev->status = SPI_DMA_STATUS_UINITED;
    GLOBAL_INT_RESTORE();

    if (spi_mdev->mutex)
        rtos_deinit_mutex(&spi_mdev->mutex);
    if (spi_mdev->finish_sem)
        rtos_deinit_semaphore(&spi_mdev->finish_sem);
    os_free(spi_mdev);
    spi_mdev = NULL;

    return 0;
}

int bk_spi_master_dma_pause_xfer(void)
{
    if (spi_mdev == NULL)
        return -1;

    if(spi_mdev->status == SPI_DMA_STATUS_PUASED)
        return 0;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    spi_mdev->status = SPI_DMA_STATUS_PUASED;
    GLOBAL_INT_RESTORE();

    if(spi_mdev->cur_elem) {
        GLOBAL_INT_DISABLE();
        SPI_D_ELEM_PTR elem = spi_mdev->cur_elem;
        spi_master_dma_enable_dma(0);
        UINT32 param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&param);
        spi_ctrl(CMD_SPI_RX_EN, (void *)&param);
        elem->flags &= ~(TX_FINISH_FLAG | RX_FINISH_FLAG | TX_FINISH_DELAY_FLAG);
        spi_mdev->cur_elem = NULL;
        GLOBAL_INT_RESTORE();
    }
    return 1;
}

int bk_spi_master_dma_continue_xfer(void)
{
    if (spi_mdev == NULL)
        return -1;

    if(spi_mdev->status != SPI_DMA_STATUS_PUASED)
        return 0;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    spi_mdev->status = SPI_DMA_STATUS_INITED;
    GLOBAL_INT_RESTORE();

    bk_spi_master_dma_trigger_tx_node();
    bk_spi_master_dma_trigger_rx_node();
    return 1;
}

int bk_spi_master_dma_check_busy(void)
{
    if (spi_mdev == NULL)
        return 0;

    if(spi_mdev->status != SPI_DMA_STATUS_INITED)
        return 0;

    if(spi_mdev->cur_elem) {
        return 1;
    } else {
        return 0;
    }
}

int bk_spi_master_dma_set_finish_callback(spi_master_dma_fin_cb cb)
{
    if (spi_mdev == NULL)
        return -1;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    spi_mdev->finish_callback = cb;
    GLOBAL_INT_RESTORE();

    return 0;
}

int bk_spi_master_dma_enable_finish_callback(UINT32 enable)
{
    if (spi_mdev == NULL)
        return -1;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if(enable) {
        spi_mdev->cfg.u.status = 1;
    } else {
        spi_mdev->cfg.u.status = 0;
    }
    GLOBAL_INT_RESTORE();

    return 0;
}

int bk_spi_master_dma_get_xfering_data_count(void)
{
    if (spi_mdev == NULL)
        return -1;

    if(spi_mdev->cur_elem) {
        UINT32 tx_left = 0, rx_left = 0, ret = 0;
        SPI_D_ELEM_PTR elem = spi_mdev->cur_elem;
        GDMA_CFG_ST en_cfg;
        if((elem->tx_buf) && (elem->tx_len)) {
            if(elem->flags & TX_FINISH_FLAG) {
                tx_left = 0;
            } else {
                en_cfg.channel = SPI_DMA_CHANNEL;
                ret = sddev_control(GDMA_DEV_NAME, CMD_GDMA_GET_ENABLE, &en_cfg);
                if(ret) {
                    tx_left = sddev_control(GDMA_DEV_NAME, CMD_GDMA_GET_REMAIN_LENGTH, &en_cfg);
                    if(tx_left > elem->tx_len)
                        tx_left = elem->tx_len;
                } else {
                    if(spi_mdev->cur_elem->flags & TX_FINISH_DELAY_FLAG)
                        tx_left = 0;
                    else
                        tx_left = elem->tx_len;
                }
                // tx not finish, not need calc rx, so return
                return tx_left;
            }
        }
        if((elem->rx_buf) && (elem->rx_len)) {
            // calc rx aready received byte
            if(elem->flags & RX_FINISH_FLAG) {
                rx_left = elem->rx_len;
            } else {
                en_cfg.channel = SPI_DMA_CHANNEL;
                ret = sddev_control(GDMA_DEV_NAME, CMD_GDMA_GET_ENABLE, &en_cfg);
                if(ret) {
                    ret = sddev_control(GDMA_DEV_NAME, CMD_GDMA_GET_REMAIN_LENGTH, &en_cfg);
                    if(ret > elem->rx_len)
                        ret = elem->rx_len;
                    rx_left = elem->rx_len - ret;
                } else {
                    rx_left = 0;
                }
            }
        }
        return tx_left + rx_left;
    }
    return 0;
}
#endif // #if ((CFG_USE_SPI_DMA_MASTER) && (CFG_USE_SPI))
#endif // #if(CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)