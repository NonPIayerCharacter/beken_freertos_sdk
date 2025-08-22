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

#if ((CFG_USE_SPI_SLAVE) && (CFG_USE_SPI))
#define SPI_SLAVE_RX_FIFO_LEN      (512)

struct spi_sdev *spi_slave_dev;
void spi_txfifo_clr(void);
void spi_rxfifo_clr(void);

UINT32 bk_spi_slave_get_rx_fifo(void)
{
    UINT32 rx_length;
    struct spi_rx_fifo *rx_fifo = spi_slave_dev->rx_fifo;
    GLOBAL_INT_DECLARATION();

    /* get rx length */
    GLOBAL_INT_DISABLE();

    rx_length = (rx_fifo->put_index >= rx_fifo->get_index) ?
                (rx_fifo->put_index - rx_fifo->get_index) :
                (SPI_SLAVE_RX_FIFO_LEN - (rx_fifo->get_index - rx_fifo->put_index));

    GLOBAL_INT_RESTORE();

    return rx_length;
}

void spi_slave_rx_callback(int is_rx_end, void *param)
{
    UINT8 ch;
    struct spi_rx_fifo *rx_fifo;

    rx_fifo = (struct spi_rx_fifo *)spi_slave_dev->rx_fifo;
    ASSERT(rx_fifo != NULL);

    if(is_rx_end) {
        os_printf("rx callback:rx_end:%d\r\n ", is_rx_end);
    }

    while (1) {
        if (spi_read_rxfifo(&ch) == 0) {
            break;
        } else {
        }

        rx_fifo->buffer[rx_fifo->put_index] = ch;
        rx_fifo->put_index += 1;
        if (rx_fifo->put_index >= SPI_SLAVE_RX_FIFO_LEN)
            rx_fifo->put_index = 0;

        if (rx_fifo->put_index == rx_fifo->get_index) {
            rx_fifo->get_index += 1;
            rx_fifo->is_full = true;
            if (rx_fifo->get_index >= SPI_SLAVE_RX_FIFO_LEN)
                rx_fifo->get_index = 0;
        }

        if (spi_slave_dev->tx_ptr == NULL)
            spi_write_txfifo(0xFF);
    }

    if (is_rx_end) {
        // only rx end happened, wake up rx_semp
        os_printf("----> rx end\r\n");
        rtos_set_semaphore(&spi_slave_dev->rx_sem);
    }
}

static int bk_spi_slave_get_rx_data(UINT8 *rx_buf, int len)
{
    struct spi_rx_fifo *rx_fifo;
    rx_fifo = (struct spi_rx_fifo *)spi_slave_dev->rx_fifo;
    int size = len;
    os_printf("len:%d\r\n", len);

    ASSERT(rx_fifo != NULL);

    if (rx_buf == NULL)
        return 0;

    while (size) {
        uint8_t ch;
        GLOBAL_INT_DECLARATION();

        GLOBAL_INT_DISABLE();

        if ((rx_fifo->get_index == rx_fifo->put_index)
                && (rx_fifo->is_full == false)) {
            GLOBAL_INT_RESTORE();
            os_printf("break:get rx data \r\n");
            break;
        }

        ch = rx_fifo->buffer[rx_fifo->get_index];
        rx_fifo->get_index += 1;
        if (rx_fifo->get_index >= SPI_SLAVE_RX_FIFO_LEN)
            rx_fifo->get_index = 0;

        if (rx_fifo->is_full == true)
            rx_fifo->is_full = false;

        GLOBAL_INT_RESTORE();

        *rx_buf = ch & 0xff;
        rx_buf ++;
        size --;

    }

    return (len - size);
}

void spi_slave_tx_need_write_callback(int port, void *param)
{
    UINT8 *tx_ptr = spi_slave_dev->tx_ptr;
    UINT32 tx_len = spi_slave_dev->tx_len;
    GLOBAL_INT_DECLARATION();

    if (tx_ptr && tx_len) {
        UINT8 data = *tx_ptr;

        while (spi_write_txfifo(data) == 1) {
            spi_read_rxfifo(&data);
            tx_len --;
            tx_ptr ++;
            if (tx_len == 0) {
                UINT32 enable = 0;
                spi_ctrl(CMD_SPI_TXINT_EN, (void *)&enable);
                break;
            }
            data = *tx_ptr;
        }
    } else {
        //rt_kprintf("nw:%p,%d\r\n", tx_ptr, tx_len);
        while (spi_write_txfifo(0xff)) {
            if (tx_len)
                tx_len--;

            if (tx_len == 0) {
                os_printf("close tx\r\n");
                UINT32 enable = 0;
                spi_ctrl(CMD_SPI_TXINT_EN, (void *)&enable);
                break;
            }
        }
    }

    GLOBAL_INT_DISABLE();
    spi_slave_dev->tx_ptr = tx_ptr;
    spi_slave_dev->tx_len = tx_len;
    GLOBAL_INT_RESTORE();

}

static void spi_slave_tx_finish_callback(int port, void *param)
{
    if ((spi_slave_dev->tx_len == 0) && (spi_slave_dev->tx_ptr)) {
        if ((spi_slave_dev->flag & TX_FINISH_FLAG) == 0) {
            spi_slave_dev->flag |= TX_FINISH_FLAG;
            rtos_set_semaphore(&spi_slave_dev->tx_sem);
        }
    }
}

static void bk_spi_slave_rx_finish_callback(int port, void *param)
{
    rtos_set_semaphore(&spi_slave_dev->rx_sem);
}

void bk_spi_slave_configure(UINT32 rate)
{
    UINT32 param;
    struct spi_callback_des spi_dev_cb;

    /* data bit width */
    if(spi_slave_dev->cfg.u.wdth)
        param = 1;
    else
        param = 0;
    spi_ctrl(CMD_SPI_SET_BITWIDTH, (void *)&param);

    /* baudrate */
    BK_SPI_PRT("max_hz = %d \n", rate);
    spi_ctrl(CMD_SPI_SET_CKR, (void *)&rate);

    /* mode */
    if (spi_slave_dev->cfg.u.cpol) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPOL, (void *)&param);

    /* CPHA */
    if (spi_slave_dev->cfg.u.cpha) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_CKPHA, (void *)&param);

    /* slave */
    param = 0;
    spi_ctrl(CMD_SPI_SET_MSTEN, (void *)&param);

    // 4line :7231N nssms is 0
    if (spi_slave_dev->cfg.u.line) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_SET_LINE_MODE, (void *)&param);

    if (spi_slave_dev->cfg.u.lsb) {
        param = 1;
    } else {
        param = 0;
    }
    spi_ctrl(CMD_SPI_LSB_EN, (void *)&param);

    param = 0;
    spi_ctrl(CMD_SPI_INIT_MSTEN, (void *)&param);

    /* set call back func */
    spi_dev_cb.callback = spi_slave_rx_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_slave_tx_need_write_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_NEED_WRITE_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = spi_slave_tx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_TX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    spi_dev_cb.callback = bk_spi_slave_rx_finish_callback;
    spi_dev_cb.param = NULL;
    spi_ctrl(CMD_SPI_SET_RX_FINISH_INT_CALLBACK, (void *)&spi_dev_cb);

    //enable rx/tx finish enable bit
    param = 1;
    spi_ctrl(CMD_SPI_TXFINISH_EN, (void *)&param);

    param = 1;
    spi_ctrl(CMD_SPI_RXFINISH_EN, (void *)&param);

    param = 1;
    spi_ctrl(CMD_SPI_RXINT_EN, (void *)&param);

    /* enable spi */
    param = 1;
    spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&param);
}

static void bk_spi_slave_unconfigure(void)
{
    spi_ctrl(CMD_SPI_DEINIT_MSTEN, NULL);
}

int bk_spi_slave_xfer(struct spi_message *msg)
{
    UINT8 *recv_ptr = NULL;
    const UINT8 *send_ptr = NULL;
    UINT32 param, unit_en_flag;
    UINT32 send_len, recv_len;
    GLOBAL_INT_DECLARATION();

    ASSERT(spi_slave_dev != NULL);
    ASSERT(msg != NULL);

    rtos_lock_mutex(&spi_slave_dev->mutex);

    recv_ptr = msg->recv_buf;
    recv_len = msg->recv_len;
    send_ptr = msg->send_buf;
    send_len = msg->send_len;

    /* if spi unit bit(reg0x0 bit23) is 0, the other register cannot set, for instance:reg0x01
     */
    unit_en_flag = 1;
    spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&unit_en_flag);

    //new spi hardware bug
    param = send_len - 1;
    spi_ctrl(CMD_SPI_TXTRANS_EN, (void *)&param);

    param = recv_len;
    spi_ctrl(CMD_SPI_RXTRANS_EN, (void *)&param);

    /*spi data port has a default data, hardware issue*/
    spi_txfifo_clr();
    spi_rxfifo_clr();

    //enbale rx/tx enable bit
    param = 1;
    spi_ctrl(CMD_SPI_TX_EN, (void *)&param);

    param = 1;
    spi_ctrl(CMD_SPI_RX_EN, (void *)&param);

    BK_SPI_PRT("spi_slave [CTRL]:0x%08x \n", REG_READ(SPI_CTRL));
    BK_SPI_PRT("spi_slave [CONFIG]:0x%08x \n", REG_READ(SPI_CONFIG));

    if ((send_ptr) && send_len) {
        GLOBAL_INT_DISABLE();
        spi_slave_dev->tx_ptr = (UINT8 *)send_ptr;
        spi_slave_dev->tx_len = send_len;
        spi_slave_dev->flag &= ~(TX_FINISH_FLAG);
        GLOBAL_INT_RESTORE();

        param = 1;
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);

        rtos_get_semaphore(&spi_slave_dev->tx_sem, BEKEN_NEVER_TIMEOUT);

        param = 0;
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);

        GLOBAL_INT_DISABLE();
        spi_slave_dev->tx_ptr = NULL;
        spi_slave_dev->tx_len = 0;
        spi_slave_dev->flag |= TX_FINISH_FLAG;
        GLOBAL_INT_RESTORE();

        param = send_len;
    } else if ((recv_ptr) && recv_len) {
        OSStatus err;
        int len;

        GLOBAL_INT_DISABLE();
        spi_slave_dev->tx_ptr = NULL;
        spi_slave_dev->tx_len = recv_len;
        GLOBAL_INT_RESTORE();

        param = 1;
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);

        do {
            len = bk_spi_slave_get_rx_data(recv_ptr, recv_len);
            if (len == 0) {
                err = rtos_get_semaphore(&spi_slave_dev->rx_sem, BEKEN_WAIT_FOREVER);
                if (err != kNoErr)
                    break;
            }
        } while (len == 0);

        param = 0;
        spi_ctrl(CMD_SPI_TXINT_EN, (void *)&param);

        // clear all rx semaphore for this time
        do {
            err = rtos_get_semaphore(&spi_slave_dev->rx_sem, 0);
        } while (err == kNoErr);

        param = len;
    }

    /* if spi unit does not be disable, the second rx of spi slave will recieve a unexpected slave-release-interrupt
     * and then spi slave cannot recieve any data.
     */
    unit_en_flag = 0;
    spi_ctrl(CMD_SPI_UNIT_ENABLE, (void *)&unit_en_flag);

    rtos_unlock_mutex(&spi_slave_dev->mutex);

    return param;
}

int bk_spi_slave_init(UINT32 rate, UINT32 mode)
{
    OSStatus result = 0;
    SPI_CFG_ST cfg;
    cfg.u.value = mode;

    if (spi_slave_dev)
        bk_spi_slave_deinit();

    spi_slave_dev = os_zalloc(sizeof(struct spi_sdev));
    if (!spi_slave_dev) {
        BK_SPI_PRT("[spi]:malloc memory for spi_dev failed\n");
        result = -1;
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_slave_dev->tx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi tx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_semaphore(&spi_slave_dev->rx_sem, 1);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi rx semp init failed\n");
        goto _exit;
    }

    result = rtos_init_mutex(&spi_slave_dev->mutex);
    if (result != kNoErr) {
        BK_SPI_PRT("[spi]: spi mutex init failed\n");
        goto _exit;
    }

    struct spi_rx_fifo *rx_fifo;

    rx_fifo = (struct spi_rx_fifo *)os_zalloc(sizeof(struct spi_rx_fifo) +
              SPI_SLAVE_RX_FIFO_LEN);
    if (!rx_fifo) {
        BK_SPI_PRT("[spi]: spi rx fifo malloc failed\n");
        goto _exit;
    }

    rx_fifo->buffer = (uint8_t *)(rx_fifo + 1);
    rx_fifo->put_index = 0;
    rx_fifo->get_index = 0;
    rx_fifo->is_full = 0;

    cfg.u.slave = 1;
    cfg.u.wdth = 0;
    cfg.u.line = 0;
    spi_slave_dev->cfg = cfg;

    spi_slave_dev->rx_fifo = rx_fifo;
    spi_slave_dev->tx_ptr = NULL;
    spi_slave_dev->tx_len = 0;
    spi_slave_dev->flag |= TX_FINISH_FLAG;

    bk_spi_slave_configure(rate);

    return 0;

_exit:
    if (spi_slave_dev->mutex)
        rtos_deinit_mutex(&spi_slave_dev->mutex);

    if (spi_slave_dev->tx_sem)
        rtos_deinit_semaphore(&spi_slave_dev->tx_sem);

    if (spi_slave_dev->rx_sem)
        rtos_deinit_semaphore(&spi_slave_dev->rx_sem);

    if (spi_slave_dev->rx_fifo)
        os_free(spi_slave_dev->rx_fifo);

    if (spi_slave_dev) {
        os_free(spi_slave_dev);
        spi_slave_dev = NULL;
    }

    return 1;
}


int bk_spi_slave_deinit(void)
{
    if (spi_slave_dev == NULL)
        return 0;

    bk_spi_slave_unconfigure();

    if (spi_slave_dev->mutex)
        rtos_lock_mutex(&spi_slave_dev->mutex);

    if (spi_slave_dev->tx_sem)
        rtos_deinit_semaphore(&spi_slave_dev->tx_sem);

    if (spi_slave_dev->rx_sem)
        rtos_deinit_semaphore(&spi_slave_dev->rx_sem);

    if (spi_slave_dev->rx_fifo)
        os_free(spi_slave_dev->rx_fifo);

    if (spi_slave_dev->mutex) {
        rtos_unlock_mutex(&spi_slave_dev->mutex);
        rtos_deinit_mutex(&spi_slave_dev->mutex);
    }

    os_free(spi_slave_dev);
    spi_slave_dev = NULL;

    return 0;
}
#endif	 // CFG_USE_SPI_SLAVE
#endif   //(CFG_SOC_NAME == SOC_BK7231N)
// eof

