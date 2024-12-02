#ifndef _SPI_PUB_H_
#define _SPI_PUB_H_

#include "uart_pub.h"

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
    CMD_SPI_SET_CKR,
    CMD_SPI_RXINT_EN,
    CMD_SPI_TXINT_EN,
    CMD_SPI_RXOVR_EN,
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
    CMD_SPI_TXFINISH_EN,
    CMD_SPI_RXFINISH_EN,
    CMD_SPI_TXTRANS_EN,
    CMD_SPI_RXTRANS_EN,
    CMD_SPI_CS_EN,
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    CMD_SPI_SET_TX_FINISH_INT_CALLBACK,
    CMD_SPI_SET_RX_FINISH_INT_CALLBACK,
#endif
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
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
    UINT8 *send_buf;
    UINT32 send_len;

    UINT8 *recv_buf;
    UINT32 recv_len;
#else
    UINT8*send_buf;
    UINT32 send_len;

    UINT8*recv_buf;
    UINT32 recv_len;
    UINT32 repeat_cnt;
#endif
};

/**
 * SPI configuration structure
 */
struct spi_configuration
{
    UINT8 mode;
    UINT8 data_width;
    UINT16 reserved;
    UINT32 max_hz;
};

typedef void (*spi_callback)(int port, void *param);
struct spi_callback_des
{
    spi_callback callback;
    void  *param;
};

/*******************************************************************************
* Function Declarations
*******************************************************************************/



/*slave api*/
/**@brief   Initialises spi slave driver
** User example:
* @code
* 
* max_hz=26 * 1000 * 1000;
* mode = SPI_MODE_0 | SPI_MSB | SPI_SLAVE;
* bk_spi_slave_init(max_hz, mode);
* @endcode
* 
* @param
*       - rate: the rate of spi transfer
*       - mode: spi mode
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_slave_init(UINT32 rate, UINT32 mode);


/**@brief  spi slave tX/rx spi data
** User example:
* @code
*    //TX
*    msg.send_buf = buf;
*    msg.send_len = tx_len;
*    msg.recv_buf = NULL;
*    msg.recv_len = 0;
*    bk_spi_slave_xfer(&msg);
* 
*    //RX
*    msg.send_buf = NULL;
*    msg.send_len = 0;
*    msg.recv_buf = buf;
*    msg.recv_len = buf_len;
*    rx_len = bk_spi_slave_xfer(&msg);
*
* 
* @endcode
* 
* @param
*       - msg:  the config of spi driver
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_slave_xfer(struct spi_message *msg);



/**@brief    De-initialise spi slave driver
*
* @param
*       - void 
* @return    
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_slave_deinit(void);



/**@brief   Initialises spi master driver
** User example:
* @code
* 
* max_hz=26 * 1000 * 1000;
* mode = SPI_MODE_0 | SPI_MSB | SPI_MASTER;
* bk_spi_slave_init(max_hz, mode);
* @endcode
* 
* @param
*       - rate: the rate of spi transfer
*       - mode: spi mode
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_init(UINT32 rate,UINT32 mode);



/**@brief  spi master tX/rx spi data
** User example:
* @code
*    //TX
*    msg.send_buf = buf;
*    msg.send_len = tx_len;
*    msg.recv_buf = NULL;
*    msg.recv_len = 0;
*    bk_spi_master_xfer(&msg);
* 
*    //RX
*    msg.send_buf = NULL;
*    msg.send_len = 0;
*    msg.recv_buf = buf;
*    msg.recv_len = buf_len;
*    rx_len = bk_spi_master_xfer(&msg);
*
* 
* @endcode
* 
* @param
*       - msg: the config of spi driver
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_xfer(struct spi_message *msg);



/**@brief    De-initialise spi master driver
*
* @param
*       -   void 
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_deinit(void);




/**@brief   Initialises spi dma driver
** User example:
* @code
* 
*    
*    max_hz=26 * 1000 * 1000;
*    msg.send_buf = buf;
*    msg.send_len = tx_len;
*    msg.recv_buf = NULL;
*    msg.recv_len = 0;
*    mode = SPI_MODE_0 | SPI_MSB | SPI_MASTER;
*    bk_spi_dma_init(mode, max_hz, &msg);
* @endcode
* 
* @param
*       - rate: the rate of spi transfer
*       - mode: spi mode
*       - spi_msg: the config of spi driver
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_dma_init(UINT32 mode, UINT32 rate, struct spi_message *spi_msg);


/**@brief  start spi tX/rx spi data
** User example:
* @code
* 
*    
*    max_hz=26 * 1000 * 1000;
*    msg.send_buf = buf;
*    msg.send_len = tx_len;
*    msg.recv_buf = NULL;
*    msg.recv_len = 0;
*    mode = SPI_MODE_0 | SPI_MSB | SPI_MASTER;
*    bk_spi_dma_transfer(mode, max_hz, &msg);
*    
* @endcode
* 
* @param
*       - mode: spi mode
*       - spi_msg: the config of spi driver
* @return    
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_dma_transfer(UINT32 mode, struct spi_message *spi_msg);



/**@brief   Initialises spi dma driver with loop mode
** User example:
* @code
* 
*    max_hz=26 * 1000 * 1000;
*    msg.send_buf = buf;
*    msg.send_len = tx_len;
*    msg.recv_buf = NULL;
*    msg.recv_len = 0;
*    msg.repeat_cnt =100; //repeat_cnt  0 & 1, means send 1 time.
*    mode = SPI_MODE_0 | SPI_MSB | SPI_MASTER;
*    bk_spi_master_dma_tx_loop_init(mode, max_hz, &msg);
*
* @endcode
* 
* @param
*       - rate: the rate of spi transfer
*       - mode: spi mode
*       - spi_msg: the config of spi driver
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_dma_tx_loop_init(UINT32 mode, UINT32 rate, struct spi_message *spi_msg);

/**@brief  start loop spi tX/rx spi data
** User example:
* @code
* 
* @endcode
* 
* @param
*       - spi_msg: the data of spi transfer
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_dma_send_loop(struct spi_message *spi_msg);

/**@brief    De-initialise spi dma loop mode
*
* @param
*       - void
* @return
*       - kNoErr: On success.
*       - others: other errors.
*/
int bk_spi_master_dma_tx_loop_deinit(void);


#endif //_SPI_PUB_H_
