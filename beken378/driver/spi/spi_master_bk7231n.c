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
#include "icu_pub.h"
#include "spi_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "mem_pub.h"
#include "error.h"
#include "rtos_pub.h"

#if(CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
#include "spi_struct.h"
#include "spi_bk7231n.h"

#if ((CFG_USE_SPI_MASTER) && (CFG_USE_SPI))
struct spi_mdev *spi_master_dev;
void spi_txfifo_clr(void);
void spi_rxfifo_clr(void);
UINT32 spi_ctrl(UINT32 cmd, void *param);

void spi_master_tx_finish_callback(int port, void *param)
{
    uint32_t flag_param;
    struct spi_mdev *spi_dev = spi_master_dev;

    if ((spi_dev->tx_req_len)
            && (spi_dev->total_len == 0)
            && ((spi_dev->flag & TX_FINISH_FLAG) == 0)) {
        spi_dev->flag |= TX_FINISH_FLAG;

        flag_param = 0;
        spi_ctrl(CMD_SPI_TX_EN, (void *)&flag_param);

        rtos_set_semaphore(&spi_dev->tx_sem);
    }
}

static void bk_spi_rx_finish_callback(int port, void *param)
{
    uint32_t flag_param;
    struct spi_mdev *spi_dev = spi_master_dev;

    if ((spi_dev->flag & RX_FINISH_FLAG) == 0) {
        spi_dev->flag |= RX_FINISH_FLAG;

        flag_param = 0;
        spi_ctrl(CMD_SPI_RX_EN, (void *)&flag_param);

        rtos_set_semaphore(&spi_dev->rx_sem);
    }
}

void spi_master_rx_callback(int is_rx_end, void *param)
{
    UINT8 ch, *rxbuf;
    UINT32 offset, drop, overflow_cnt = 0;

    GLOBAL_INT_DECLARATION();

    rxbuf = spi_master_dev->rx_ptr;
    drop = spi_master_dev->rx_drop;
    offset = spi_master_dev->rx_offset;

    while (1) {
        if (spi_read_rxfifo(&ch) == 0)
            break;

        //rec_func(0x22000002);
        if (rxbuf) {
            if (drop != 0) {
                drop--;
            } else {
                if (offset < spi_master_dev->rx_len) {
                    rxbuf[offset] = ch;
                    offset++;
                } else {
                    overflow_cnt ++;
                    BK_SPI_WPRT("rx over flow:%02x, %d\r\n", ch, spi_master_dev->rx_len);
                }
            }
        }
    }

    GLOBAL_INT_DISABLE();
    spi_master_dev->rx_drop = drop;
    spi_master_dev->rx_offset = offset;
    spi_master_dev->rx_overflow_cnt = overflow_cnt;
    GLOBAL_INT_RESTORE();

    if(spi_master_dev->rx_len == spi_master_dev->rx_offset) {
        bk_spi_rx_finish_callback(0, NULL);
    }
}

void spi_master_tx_need_write_callback(int port, void *param)
{
    UINT8 *tx_ptr = spi_master_dev->tx_ptr, data;
    UINT32 tx_len = spi_master_dev->tx_remain_cnt, total_len = spi_master_dev->total_len, tx_ok = 0;
    UINT8 *rxbuf;
    UINT32 offset, drop;

    rxbuf = spi_master_dev->rx_ptr;
    drop = spi_master_dev->rx_drop;
    offset = spi_master_dev->rx_offset;

    GLOBAL_INT_DECLARATION();

    while (total_len) {
        tx_ok = 0;

        if (tx_len) {
            data = *tx_ptr;
            if (spi_write_txfifo(data) == 1) {
                tx_ok = 1;

                tx_len --;
                tx_ptr ++;
            } else {
            }
        } else {
            data = 0xff;
            if (spi_write_txfifo(data) == 1) {
                tx_ok = 1;
            }
        }

        /* check rx data to prevent rx over flow */
        if (spi_read_rxfifo(&data) == 1) {
            if (rxbuf) {
                if (drop != 0) {
                    drop --;
                } else {
                    if (offset < spi_master_dev->rx_len) {
                        rxbuf[offset] = data;
                        offset++;
                    } else {
                        BK_SPI_WPRT("0 rx over flow:%02x, %d\r\n", data, spi_master_dev->rx_len);
                    }
                }
            }
        }

        if (tx_ok == 1) {
            total_len --;
            if (total_len == 0) {
                UINT32 enable = 0;
                spi_ctrl(CMD_SPI_TXINT_EN, (void *)&enable);

                break;
            }
        } else
            break;
    }

    GLOBAL_INT_DISABLE();
    spi_master_dev->tx_ptr = tx_ptr;
    spi_master_dev->tx_remain_cnt = tx_len;
    spi_master_dev->total_len = total_len;

    spi_master_dev->rx_drop = drop;
    spi_master_dev->rx_offset = offset;
    GLOBAL_INT_RESTORE();
}

void bk_spi_configure(UINT32 rate)
{
    UINT32 param;
    struct spi_callback_des spi_dev_cb;

    /* data bit width */
    if(spi_master_dev->cfg.u.wdth)
        param = 1;
    else
        param = 0;
    spi_ctrl(CMD_SPI_SET_BITWIDTH, (void *)&param);

    if(spi_master_dev->cfg.u.interval)
        param = spi_master_dev->cfg.u.interval;
    else
        param = 0x0;
    spi_ctrl(CMD_SPI_SET_BYTE_INTVAL, (void *)&param);

    /* baudrate */
    BK_SPI_PRT("max_hz = %d \n", rate);
    spi_ctrl(CMD_SPI_SET_CKR, (void *)&rate);

    /* mode */
    if (spi_master_dev->cfg.u.cpol) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPOL, (void *)&param);

    /* CPHA */
    if (spi_master_dev->cfg.u.cpha) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPHA, (void *)&param);

    /* Master */
    param = 1;
    spi_ctrl(CMD_SPI_SET_MSTEN, (void *)&param);

    // 4line :7231N nssms is 0
    if (spi_master_dev->cfg.u.line) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_LINE_MODE, (void *)&param);

    if (spi_master_dev->cfg.u.lsb) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_LSB_EN, (void *)&param);

    param = 0;
    spi_ctrl(CMD_SPI_INIT_MSTEN, (void *)&param);

    /* set call back func */
    spi_dev_cb.callback = spi_master_rx_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_master_tx_need_write_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_NEED_WRITE_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_master_tx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    /* enable spi */
    param = 1;
    spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);

    BK_SPI_PRT("spi_master:[CTRL]:0x%08x \n", REG_READ(SPI_CTRL));
}

void bk_spi_unconfigure(void)
{
    spi_ctrl(CMD_SPI_DEINIT_MSTEN, NULL);
}

int spi_master_xfer_start_setting(struct spi_message *msg)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    spi_master_dev->tx_ptr = msg->send_buf;
    spi_master_dev->tx_remain_cnt = msg->send_len;
    spi_master_dev->tx_req_len = msg->send_len;

    spi_master_dev->rx_ptr = msg->recv_buf;
    spi_master_dev->rx_len = msg->recv_len;
    spi_master_dev->rx_offset = 0;
    spi_master_dev->rx_overflow_cnt = 0;
    spi_master_dev->rx_drop = msg->send_len;

    spi_master_dev->total_len = msg->recv_len + msg->send_len;
    spi_master_dev->flag &= ~(TX_FINISH_FLAG | RX_FINISH_FLAG);
    GLOBAL_INT_RESTORE();

    return 0;
}

int spi_master_xfer_complete_setting(struct spi_message *msg)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    spi_master_dev->tx_ptr = NULL;
    spi_master_dev->tx_remain_cnt = 0;
    spi_master_dev->tx_req_len = 0;

    spi_master_dev->rx_ptr = NULL;
    spi_master_dev->rx_len = 0;

    spi_master_dev->total_len = 0;
    spi_master_dev->flag |= TX_FINISH_FLAG;
    GLOBAL_INT_RESTORE();

    return 0;
}

int bk_spi_master_xfer(struct spi_message *msg)
{
    OSStatus err;
    UINT32 param, total_size;

    ASSERT(spi_master_dev != NULL);
    ASSERT(msg != NULL);

    rtos_lock_mutex(&spi_master_dev->mutex);

    total_size = msg->recv_len + msg->send_len;
    spi_txfifo_clr();
    spi_rxfifo_clr();

    if (total_size) {
        spi_master_xfer_start_setting(msg);

        param = 1;
        spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);

        /* 4line :7231N nssms is 0*/
        #if (SPI_LINE_MODE == SPI_USE_4_LINE)
        param = 0;
        #else
        param = 1;
        #endif
        spi_ctrl(CMD_SPI_SET_LINE_MODE, (void *)&param);

        if(msg->send_len) {
            /* spi master as the sender, and configure the tx/rx threshold level in this way*/
            param = 3;/* 3: generate tx isr when tx fifo count is less than 48*/
            spi_ctrl(CMD_SPI_TXINT_MODE, (void *)&param);
            param = 1;/* 1: generate rx isr when rx fifo count is more than 16*/
            spi_ctrl(CMD_SPI_RXINT_MODE, (void *)&param);

            spi_master_tx_need_write_callback(0, NULL);
        } else {
            /* spi master as the reciever, and configure the tx/rx threshold level in this way*/
            param = 2;/* 2: generate tx isr when tx fifo count is less than 32*/
            spi_ctrl(CMD_SPI_TXINT_MODE, (void *)&param);
            param = 0;/* 1: generate rx isr when rx fifo count is more than 1*/
            spi_ctrl(CMD_SPI_RXINT_MODE, (void *)&param);
        }

        /* disable rx/tx over*/
        param = 0;
        spi_ctrl(CMD_SPI_RXOVR_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXOVR_EN, (void *)&param);

        /*set trans len rx equ to tx to prevnt unexpected clock*/
        spi_ctrl(CMD_SPI_TXTRANS_EN, (void *)&total_size);
        spi_ctrl(CMD_SPI_RXTRANS_EN, (void *)&total_size);
        param = 1;
        spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);

        /* enable tx*/
        param = 1;
        spi_ctrl(CMD_SPI_RXINT_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);

        /* if enable rx_en before enabling tx_en, maybe tx_fifo is not filled with data, and then
         * master will transfer one or more unexpected data; if enable tx_en before enabling rx_en,
         * the data transmission maybe has ended, but reception has not yet started. if spi flash read id
         * spi cmd sends successfully, and the flash id can not recieved;
         */
        spi_ctrl(CMD_SPI_TRX_EN, (void *)&param);

        /* wait tx finish */
        if(msg->send_len) {
            err = rtos_get_semaphore(&spi_master_dev->tx_sem, BEKEN_NEVER_TIMEOUT);
        }
        if(msg->recv_len) {
            err = rtos_get_semaphore(&spi_master_dev->rx_sem, BEKEN_NEVER_TIMEOUT);
        }

        //disable tx-rx
        param = 0;
        spi_ctrl(CMD_SPI_TRX_EN, (void *)&param);

        /* disable tx & rx interrupt again */
        param = 0;
        spi_ctrl(CMD_SPI_RXINT_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);
        spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);
        spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);

        /* initial spi_master_dev with zero*/
        do {
            err = rtos_get_semaphore(&spi_master_dev->tx_sem, 0);
        } while (err == kNoErr);
        do {
            err = rtos_get_semaphore(&spi_master_dev->rx_sem, 0);
        } while (err == kNoErr);

        spi_master_xfer_complete_setting(msg);
    }

    rtos_unlock_mutex(&spi_master_dev->mutex);

    return total_size;
}

int bk_spi_master_init(UINT32 rate, UINT32 mode)
{
    OSStatus result = 0;
    SPI_CFG_ST cfg;
    cfg.u.value = mode;

    if (spi_master_dev)
        bk_spi_master_deinit();

    spi_master_dev = os_zalloc(sizeof(struct spi_mdev));
    if (!spi_master_dev) {
        BK_SPI_PRT("[spi]:malloc memory for spi_master_dev failed\n");
        result = -1;
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_master_dev->tx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi tx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_master_dev->rx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi rx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_mutex(&spi_master_dev->mutex);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi mutex init failed\n");
        goto _exit;
    }

    cfg.u.slave = 0;
    cfg.u.wdth = 0;
    cfg.u.line = 0;
    spi_master_dev->cfg = cfg;
    spi_master_dev->tx_ptr = NULL;
    spi_master_dev->tx_remain_cnt = 0;
    spi_master_dev->flag |= TX_FINISH_FLAG;

    bk_spi_configure(rate);

    return 0;

_exit:
    if (spi_master_dev->mutex)
        rtos_deinit_mutex(&spi_master_dev->mutex);

    if (spi_master_dev->tx_sem)
        rtos_deinit_semaphore(&spi_master_dev->tx_sem);

    if (spi_master_dev) {
        os_free(spi_master_dev);
        spi_master_dev = NULL;
    }

    return 1;
}

int bk_spi_master_deinit(void)
{
    if (spi_master_dev == NULL)
        return 0;

    if (spi_master_dev->mutex)
        rtos_lock_mutex(&spi_master_dev->mutex);

    if (spi_master_dev->tx_sem)
        rtos_deinit_semaphore(&spi_master_dev->tx_sem);

    if (spi_master_dev->mutex) {
        rtos_unlock_mutex(&spi_master_dev->mutex);
        rtos_deinit_mutex(&spi_master_dev->mutex);
    }

    if (spi_master_dev) {
        os_free(spi_master_dev);
        spi_master_dev = NULL;
    }

    bk_spi_unconfigure();

    return 0;
}
#endif  // CFG_USE_SPI_MASTER
#endif  //(CFG_SOC_NAME == SOC_BK7231N)
// eof

