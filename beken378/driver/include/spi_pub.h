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

#ifndef _SPI_PUB_H_
#define _SPI_PUB_H_

#include "uart_pub.h"
#include "gpio_pub.h"

#define SPI_FAILURE                (1)
#define SPI_SUCCESS                (0)

#define SPI_DEV_NAME                "spi"
#define SPI_CMD_MAGIC              (0xe250000)
enum
{
    CMD_SPI_UNIT_ENABLE = SPI_CMD_MAGIC + 1,
    CMD_SPI_SET_MSTEN,
    CMD_SPI_SET_CKPHA,
    CMD_SPI_SET_CKPOL,
    CMD_SPI_SET_BITWIDTH,
    CMD_SPI_SET_NSSMD,
    CMD_SPI_SET_LINE_MODE, /* configure spi to 3-wire mode or 4-wire mode*/
    CMD_SPI_SET_CKR,
    CMD_SPI_RXINT_EN,
    CMD_SPI_TXINT_EN,
    CMD_SPI_RXOVR_EN, /* offset 0x0a*/
    CMD_SPI_TXOVR_EN,
    CMD_SPI_RXFIFO_CLR,
    CMD_SPI_RXINT_MODE,
    CMD_SPI_TXINT_MODE,
    CMD_SPI_INIT_MSTEN,
    CMD_SPI_GET_BUSY,
    CMD_SPI_SET_RX_CALLBACK,
    CMD_SPI_SET_TX_NEED_WRITE_CALLBACK,
    CMD_SPI_SET_TX_FINISH_CALLBACK,
    CMD_SPI_DEINIT_MSTEN,
    CMD_SPI_LSB_EN,
    CMD_SPI_TX_EN,
    CMD_SPI_RX_EN,
    CMD_SPI_TRX_EN,
    CMD_SPI_TXFINISH_EN,
    CMD_SPI_RXFINISH_EN,
    CMD_SPI_TXTRANS_EN,
    CMD_SPI_RXTRANS_EN,
    CMD_SPI_CS_EN,
    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
    CMD_SPI_SET_TX_FINISH_INT_CALLBACK,
    CMD_SPI_SET_RX_FINISH_INT_CALLBACK,
    #endif
    CMD_SPI_SET_BYTE_INTVAL,
};

enum
{
    SPI_EVEN_NONE = 0,
    SPI_EVEN_TRX_DONE,
    SPI_EVEN_TRX_LOSE,
    SPI_EVEN_OTHERS,
};

#define BK_SPI_DEBUG                0

#if BK_SPI_DEBUG
#define BK_SPI_PRT               warning_prf
#define BK_SPI_WPRT              warning_prf
#define BK_SPI_FATAL             fatal_prf
#else
#define BK_SPI_PRT		null_prf
#define BK_SPI_WPRT		null_prf
#define BK_SPI_FATAL		warning_prf
#endif

#define USE_SPI_GPIO_14_17          (0)
#define USE_SPI_GPIO_30_33          (1)
#define USE_SPI_GPIO_NUM            USE_SPI_GPIO_14_17
#define SPI_FLASH_WP_GPIO_NUM       (GPIO18)
#define SPI_FLASH_HOLD_GPIO_NUM     (GPIO19)

#define SPI_DEF_CLK_HZ              (10 * 1000 * 1000)
#define TX_FINISH_FLAG              (1 << 0)
#define RX_FINISH_FLAG              (1 << 1)
#define TX_FINISH_DELAY_FLAG        (1 << 2)
#define WAIT_DONE_FLAG              (1 << 3)
#define TRX_ABORT_FLAG              (1 << 4)

#define BK_SPI_CPOL                 0x01
#define BK_SPI_CPHA                 0x02
#define SPI_DEF_MODE                (~((BK_SPI_CPOL)|(BK_SPI_CPHA)))

#define SPI_CPHA     (1<<0)                             /* bit[0]:CPHA, clock phase */
#define SPI_CPOL     (1<<1)                             /* bit[1]:CPOL, clock polarity */

#define SPI_LSB      (0<<2)                             /* bit[2]: 0-LSB */
#define SPI_MSB      (1<<2)                             /* bit[2]: 1-MSB */

#define SPI_MASTER   (0<<3)								/* SPI master device */
#define SPI_SLAVE    (1<<3)								/* SPI slave device */

#define SPI_USE_4_LINE			0					/* 0-4 line mode*/
#define SPI_USE_3_LINE			1					/* 1-3 line mode: no NSS*/
#define SPI_LINE_MODE			SPI_USE_4_LINE		/* default:4 line mode*/

#define SPI_MODE_0       (0 | 0)                        /* CPOL = 0, CPHA = 0 */
#define SPI_MODE_1       (0 | SPI_CPHA)              /* CPOL = 0, CPHA = 1 */
#define SPI_MODE_2       (SPI_CPOL | 0)              /* CPOL = 1, CPHA = 0 */
#define SPI_MODE_3       (SPI_CPOL | SPI_CPHA)    /* CPOL = 1, CPHA = 1 */

#define ENABLE				1
#define DISABLE				0
#define SPI_TX_LENGTH_MAX		4096

struct spi_message
{
    UINT8 *send_buf;
    UINT32 send_len;

    UINT8 *recv_buf;
    UINT32 recv_len;
};

/**
 * SPI configuration structure
 */
typedef struct spi_configuration {
    union {
        struct {
                uint32_t cpha:	1; /**< bit[0] cpha */
                uint32_t cpol:	1; /**< bit[1] cpol */
                uint32_t lsb:	1; /**< bit[2] 0:msb send first; 1:lsb send first */
                uint32_t slave:	1; /**< bit[3] 0:master; 1:slave */
                uint32_t dma:	1; /**< bit[4] 0:not use dma; 1: use dma */
                uint32_t wdth:	1; /**< bit[5] data unit, 0: 8 bits; 1: 16 bits */
                uint32_t line:	1; /**< bit[6] line_mode, 0: 4 lines; 1: 3 lines */
                uint32_t group:	1; /**< bit[7] io group, 0: p16-p20, 1: p30-p33 */
                uint32_t interval:	6;  /**< bit[8:13] the interval between each data unit, count by spi sck clock */
                uint32_t reserved:	17; /**< bit[14:30] reserved */
                uint32_t status:1; /**< bit[31] status, 0: un inited, 1: inited */
            };
        UINT32 value;
    } u;
} SPI_CFG_ST, *SPI_CFG_PTR;

typedef void (*spi_callback)(int port, void *param);
typedef void (*spi_master_dma_fin_cb)(UINT32 event, struct spi_message *msg);
struct spi_callback_des
{
    spi_callback callback;
    void  *param;
};

/*******************************************************************************
* Function Declarations
*******************************************************************************/
UINT32 spi_read_rxfifo(UINT8 *data);
UINT32 spi_write_txfifo(UINT8 data);

void spi_init(void);
void spi_exit(void);
void spi_isr(void);

#if (CFG_SOC_NAME == SOC_BK7271)
void spi_channel_set(UINT8 channel );
void spi2_init(void);
void spi2_exit(void);
void spi2_isr(void);
void spi3_init(void);
void spi3_exit(void);
void spi3_isr(void);
#endif

/*slave api*/
int bk_spi_slave_init(UINT32 rate, UINT32 mode);
int bk_spi_slave_xfer(struct spi_message *msg);
int bk_spi_slave_deinit(void);

/*master api*/
int bk_spi_master_init(UINT32 rate, UINT32 mode);
int bk_spi_master_xfer(struct spi_message *msg);
int bk_spi_master_deinit(void);

#if CFG_USE_SPI_DMA_MASTER
#define BK_SPI_M_DMA_DEF_WAIT_TIME     (5000)
int bk_spi_master_dma_xfer(struct spi_message *msg, UINT32 wait_time_ms);
int bk_spi_master_dma_init(UINT32 rate, UINT32 mode);
int bk_spi_master_dma_deinit(UINT32 force_deinit);
int bk_spi_master_dma_abort_xfer(void);
int bk_spi_master_dma_check_busy(void);
int bk_spi_master_dma_set_finish_callback(spi_master_dma_fin_cb cb);
int bk_spi_master_dma_enable_finish_callback(UINT32 enable);
int bk_spi_master_dma_get_xfering_data_count(void);
int bk_spi_master_dma_pause_xfer(void);
int bk_spi_master_dma_continue_xfer(void);
#endif

#if CFG_USE_SPI_DMA_SLAVE
int bk_spi_slave_dma_xfer(struct spi_message *msg);
int bk_spi_slave_dma_init(UINT32 rate, UINT32 mode);
int bk_spi_slave_dma_deinit(void);
#endif

#endif //_SPI_PUB_H_
