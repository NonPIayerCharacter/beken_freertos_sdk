#ifndef __SDIO_DRIVER_H__
#define __SDIO_DRIVER_H__

#include "include.h"

#include "uart_pub.h"
#if (CFG_SOC_NAME == SOC_BK7252N)
#include "sdcard_pub.h"
#endif
#define SDCARD_DEBUG

#ifdef SDCARD_DEBUG
#define SDCARD_PRT      os_printf
#define SDCARD_WARN     warning_prf
#define SDCARD_FATAL    fatal_prf
#else
#define SDCARD_PRT      null_prf
#define SDCARD_WARN     null_prf
#define SDCARD_FATAL    null_prf
#endif

// SDCARD REG
#define SDCARD_BASE_ADDR                        (0x00802D00)

#if (CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_DEV_ID                       (SDCARD_BASE_ADDR + 0*4)
#define REG_SDCARD_VER_ID                       (SDCARD_BASE_ADDR + 1*4)
#define REG_SDCARD_CLK_RST                      (SDCARD_BASE_ADDR + 2*4)
#define REG_SDCARD_CLK_RST                      (SDCARD_BASE_ADDR + 2*4)
#define REG_SDCARD_CLK_RST_SOFT_RST             (1 << 0)
#define REG_SDCARD_CLK_RST_BPS_CLKGATE          (1 << 1)
#define REG_SDCARD_GLOBAL_STATUS                (SDCARD_BASE_ADDR + 3*4)
#endif

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_CMD_SEND_CTRL                (SDCARD_BASE_ADDR + 0*4)
#else
#define REG_SDCARD_CMD_SEND_CTRL                (SDCARD_BASE_ADDR + 4*4)
#endif
#define SDCARD_CMD_SEND_CTRL_CMD_START          (1 << 0)
#define SDCARD_CMD_SEND_CTRL_CMD_FLAGS_MASK     (0x7)
#define SDCARD_CMD_SEND_CTRL_CMD_FLAGS_POSI     (1)
#if (CFG_SOC_NAME == SOC_BK7252N)
#define SDCARD_CMD_SEND_CTRL_CMD_RSP            (1 << 1)
#define SDCARD_CMD_SEND_CTRL_CMD_LONG           (1 << 2)
#define SDCARD_CMD_SEND_CTRL_CMD_CRC_CHECK      (1 << 3)
#endif
#define SDCARD_CMD_SEND_CTRL_CMD_INDEX_MASK     (0x3f)
#define SDCARD_CMD_SEND_CTRL_CMD_INDEX_POSI     (4)

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_CMD_SEND_AGUMENT             (SDCARD_BASE_ADDR + 1*4)
#define REG_SDCARD_CMD_RSP_TIMER                (SDCARD_BASE_ADDR + 2*4)

#define REG_SDCARD_DATA_REC_CTRL                (SDCARD_BASE_ADDR + 3*4)
#else
#define REG_SDCARD_CMD_SEND_AGUMENT             (SDCARD_BASE_ADDR + 5*4)
#define REG_SDCARD_CMD_RSP_TIMER                (SDCARD_BASE_ADDR + 6*4)

#define REG_SDCARD_DATA_REC_CTRL                (SDCARD_BASE_ADDR + 7*4)
#endif
#define SDCARD_DATA_REC_CTRL_DATA_EN            (1 << 0)
#define SDCARD_DATA_REC_CTRL_DATA_STOP_EN       (1 << 1)
#define SDCARD_DATA_REC_CTRL_DATA_BUS           (1 << 2)
#define SDCARD_DATA_REC_CTRL_DATA_MUL_BLK       (1 << 3)
#define SDCARD_DATA_REC_CTRL_BLK_SIZE_MASK      (0xfff)
#define SDCARD_DATA_REC_CTRL_BLK_SIZE_POSI      (4)
#define SDCARD_DATA_REC_CTRL_DATA_WR_DATA_EN    (1 << 16)
#define SDCARD_DATA_REC_CTRL_DATA_BYTE_SEL      (1 << 17)

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_DATA_REC_TIMER               (SDCARD_BASE_ADDR + 4*4)
#define REG_SDCARD_CMD_RSP_AGUMENT0             (SDCARD_BASE_ADDR + 5*4)
#define REG_SDCARD_CMD_RSP_AGUMENT1             (SDCARD_BASE_ADDR + 6*4)
#define REG_SDCARD_CMD_RSP_AGUMENT2             (SDCARD_BASE_ADDR + 7*4)
#define REG_SDCARD_CMD_RSP_AGUMENT3             (SDCARD_BASE_ADDR + 8*4)

#define REG_SDCARD_CMD_RSP_INT_SEL              (SDCARD_BASE_ADDR + 9*4)
#else
#define REG_SDCARD_DATA_REC_TIMER               (SDCARD_BASE_ADDR + 8*4)
#define REG_SDCARD_CMD_RSP_AGUMENT0             (SDCARD_BASE_ADDR + 9*4)
#define REG_SDCARD_CMD_RSP_AGUMENT1             (SDCARD_BASE_ADDR + 10*4)
#define REG_SDCARD_CMD_RSP_AGUMENT2             (SDCARD_BASE_ADDR + 11*4)
#define REG_SDCARD_CMD_RSP_AGUMENT3             (SDCARD_BASE_ADDR + 12*4)

#define REG_SDCARD_CMD_RSP_INT_SEL              (SDCARD_BASE_ADDR + 13*4)
#endif
#define SDCARD_CMDRSP_NORSP_END_INT             (1 << 0)
#define SDCARD_CMDRSP_RSP_END_INT               (1 << 1)
#define SDCARD_CMDRSP_TIMEOUT_INT               (1 << 2)
#define SDCARD_CMDRSP_DATA_REC_END_INT          (1 << 3)
#define SDCARD_CMDRSP_DATA_WR_END_INT           (1 << 4)
#define SDCARD_CMDRSP_DATA_TIME_OUT_INT         (1 << 5)
#define SDCARD_CMDRSP_RX_FIFO_NEED_READ         (1 << 6)
#define SDCARD_CMDRSP_TX_FIFO_NEED_WRITE        (1 << 7)
#define SDCARD_CMDRSP_RX_OVERFLOW               (1 << 8)
#define SDCARD_CMDRSP_TX_FIFO_EMPTY             (1 << 9)
#define SDCARD_CMDRSP_CMD_CRC_OK                (1 << 10)
#define SDCARD_CMDRSP_CMD_CRC_FAIL              (1 << 11)
#define SDCARD_CMDRSP_DATA_CRC_OK               (1 << 12)
#define SDCARD_CMDRSP_DATA_CRC_FAIL             (1 << 13)
#define SDCARD_CMDRSP_RSP_INDEX                 (0x3f<<14)
#define SDCARD_CMDRSP_WR_STATU                  (0x7<<20)
#define SDCARD_CMDRSP_DATA_BUSY                 (0x1<<23)
#if (CFG_SOC_NAME == SOC_BK7252N)
#define SDCARD_CMDRSP_RES_END_INT               (1 << 24)
#define SDCARD_CMDRSP_DAT_WR_WAI_INT            (1 << 25)
#define SDCARD_CMDRSP_DAT_RD_BUS_INT            (1 << 26)
#define SDCARD_CMDRSP_DAT_WRSTS_ERR_INT         (1 << 27)
#define SDCARD_CMDRSP_DAT_CRC_FAIL_INT          (1 << 28)
#define SDCARD_CMDRSP_DAT_WR_BLK_INT            (1 << 29)
#endif

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_CMD_RSP_INT_MASK             (SDCARD_BASE_ADDR + 10*4)
#else
#define REG_SDCARD_CMD_RSP_INT_MASK             (SDCARD_BASE_ADDR + 14*4)
#endif
#define SDCARD_CMDRSP_NORSP_END_INT_MASK        (1 << 0)
#define SDCARD_CMDRSP_RSP_END_INT_MASK          (1 << 1)
#define SDCARD_CMDRSP_TIMEOUT_INT_MASK          (1 << 2)
#define SDCARD_CMDRSP_DATA_REC_END_INT_MASK     (1 << 3)
#define SDCARD_CMDRSP_DATA_WR_END_INT_MASK      (1 << 4)
#define SDCARD_CMDRSP_DATA_TIME_OUT_INT_MASK    (1 << 5)
#define SDCARD_CMDRSP_RX_FIFO_NEED_READ_MASK    (1 << 6)
#define SDCARD_CMDRSP_TX_FIFO_NEED_WRITE_MASK   (1 << 7)
#define SDCARD_CMDRSP_RX_OVERFLOW_MASK          (1 << 8)
#define SDCARD_CMDRSP_TX_FIFO_EMPTY_MASK        (1 << 9)
#if (CFG_SOC_NAME == SOC_BK7252N)
#define SDCARD_CMDRSP_RES_END_INT_MASK          (1 << 10)
#define SDCARD_CMDRSP_DAT_WR_WAI_INT_MASK       (1 << 11)
#define SDCARD_CMDRSP_DAT_RD_BUS_INT_MASK       (1 << 12)
#define SDCARD_CMDRSP_TX_FIFO_NEED_WR_MASK_CG   (1 << 13)
#define SDCARD_CMDRSP_WRITE_WAIT_JUMP_SEL       (1 << 14)
#define SDCARD_CMDRSP_IDLE_STOP_JUMP_SEL        (1 << 15)
#define SDCARD_CMDRSP_DAT_WRSTS_ERR_INT_MASK    (1 << 16)
#define SDCARD_CMDRSP_DAT_CRC_FAIL_INT_MASK     (1 << 17)
#define SDCARD_CMDRSP_DAT_WR_BLK_INT_MASK       (1 << 18)
#define SDCARD_CMDRSP_DAT_RD_MUL_SEL            (1 << 19)
#define SDCARD_CMDRSP_DAT_WR_MUL_SEL            (1 << 20)
#endif

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_WR_DATA_ADDR                 (SDCARD_BASE_ADDR + 11*4)
#define REG_SDCARD_RD_DATA_ADDR                 (SDCARD_BASE_ADDR + 12*4)

#define REG_SDCARD_FIFO_THRESHOLD               (SDCARD_BASE_ADDR + 13*4)
#else
#define REG_SDCARD_WR_DATA_ADDR                 (SDCARD_BASE_ADDR + 15*4)
#define REG_SDCARD_RD_DATA_ADDR                 (SDCARD_BASE_ADDR + 16*4)
#define REG_SDCARD_TX_FIFO_DIN_ADDR             (SDCARD_BASE_ADDR + 15*4)
#define REG_SDCARD_RX_FIFO_DOUT_ADDR            (SDCARD_BASE_ADDR + 16*4)

#define REG_SDCARD_FIFO_THRESHOLD               (SDCARD_BASE_ADDR + 17*4)
#endif
#define SDCARD_FIFO_RX_FIFO_THRESHOLD_MASK      (0xff)
#define SDCARD_FIFO_RX_FIFO_THRESHOLD_POSI      (0)
#define SDCARD_FIFO_TX_FIFO_THRESHOLD_MASK      (0xff)
#define SDCARD_FIFO_TX_FIFO_THRESHOLD_POSI      (8)
#define SDCARD_FIFO_RX_FIFO_RST                 (1 << 16)
#define SDCARD_FIFO_TX_FIFO_RST                 (1 << 17)
#define SDCARD_FIFO_RXFIFO_RD_READY             (1 << 18)
#define SDCARD_FIFO_TXFIFO_WR_READY             (1 << 19)
#define SDCARD_FIFO_SD_STA_RST                  (1 << 20)
#define SDCARD_FIFO_SD_RATE_SELECT_POSI         (21)
#define SDCARD_FIFO_SD_RATE_SELECT_MASK         (0x3)
#if (CFG_SOC_NAME == SOC_BK7252N)
#define SDCARD_FIFO_SD_CLK_SEL_POSI             (21)
#define SDCARD_FIFO_SD_CLK_SEL_MASK             (0x3)
#define SDCARD_FIFO_SD_RD_WAIT_SEL              (1 << 23)
#define SDCARD_FIFO_SD_WR_WAIT_SEL              (1 << 24)
#define SDCARD_FIFO_CLK_REC_SEL                 (1 << 25)
#define SDCARD_FIFO_SAMP_SEL                    (1 << 26)
#define SDCARD_FIFO_CLK_GATE_ON                 (1 << 27)
#define SDCARD_FIFO_HOST_WR_BLK_EN              (1 << 28)
#define SDCARD_FIFO_HOST_RD_BLK_EN              (1 << 29)
#endif

#if (CFG_SOC_NAME == SOC_BK7252N)
#define REG_SDCARD_REG0X12                      (SDCARD_BASE_ADDR + 18*4)
#define SDCARD_REG0X12_SD_SLAVE                 (1 << 0)
#define SDCARD_REG0X12_DAT_RD_MUL_BLK           (1 << 1)
#define SDCARD_REG0X12_IO_CUR_STA_POSI          (2)
#define SDCARD_REG0X12_IO_CUR_STA_MASK          (0x3)
#define SDCARD_REG0X12_CMD_52_STOP_CLR          (1 << 4)
#define SDCARD_REG0X12_CMD_KEEP_DET             (1 << 5)
#define SDCARD_REG0X12_FIFO_SEND_CNT_POSI       (8)
#define SDCARD_REG0X12_FIFO_SEND_CNT_MASK       (0xff)

#define REG_SDCARD_SD_SLAVE_RDAT_0              (SDCARD_BASE_ADDR + 19*4)
#define REG_SDCARD_SD_SLAVE_RDAT_1              (SDCARD_BASE_ADDR + 20*4)
#define REG_SDCARD_SD_SLAVE_WDAT_0              (SDCARD_BASE_ADDR + 21*4)
#define REG_SDCARD_SD_SLAVE_WDAT_1              (SDCARD_BASE_ADDR + 22*4)
#define REG_SDCARD_REG0X17                      (SDCARD_BASE_ADDR + 23*4)
#define SDCARD_REG0X12_CMD_RES_DAT_RD           (1 << 0)
#define SDCARD_REG0X12_CMD_RES_DAT_WR           (1 << 1)
#define SDCARD_REG0X12_CMD_REC_BB_CNT_POSI      (2)
#define SDCARD_REG0X12_CMD_REC_BB_CNT_MASK      (0x1ff)
#define SDCARD_REG0X12_CMD_REC_OP_CODE          (1 << 11)
#define SDCARD_REG0X12_CMD_REC_BLK_MOD          (1 << 12)
#define SDCARD_REG0X12_SD_START_WR_EN_R3        (1 << 13)
#define SDCARD_REG0X12_DAT_RD_BUS_4RD           (1 << 14)
#define SDCARD_REG0X12_CMD_RES_END_4RD          (1 << 15)
#define SDCARD_REG0X12_DAT_WR_WAI_4RD           (1 << 16)

#define REG_SDCARD_REG0X18                      (SDCARD_BASE_ADDR + 24*4)
#define SDCARD_REG0X18_RD_RGT_BLKS_POSI         (0)
#define SDCARD_REG0X18_RD_RGT_BLKS_MASK         (0xffff)
#define SDCARD_REG0X18_WR_RGT_BLKS_POSI         (16)
#define SDCARD_REG0X18_WR_RGT_BLKS_MASK         (0xffff)

#define REG_SDCARD_REG0X19                      (SDCARD_BASE_ADDR + 25*4)
#define SDCARD_REG0X19_DAT_WR_BLK_CNT_POSI      (0)
#define SDCARD_REG0X19_DAT_WR_BLK_CNT_MASK      (0x1ff)

#endif
// SDcard defination
/* Exported types ------------------------------------------------------------*/
typedef enum
{
    SD_OK   =   0,
    SD_CMD_CRC_FAIL               = (1), /*!< Command response received (but CRC check failed) */
    SD_DATA_CRC_FAIL              = (2), /*!< Data bock sent/received (CRC check Failed) */
    SD_CMD_RSP_TIMEOUT            = (3), /*!< Command response timeout */
    SD_DATA_TIMEOUT               = (4), /*!< Data time out */

    SD_INVALID_VOLTRANGE,
    SD_R5_ERROR,            /* A general or an unknown error occurred during the operation */
    SD_R5_ERR_FUNC_NUMB,    /* An invalid function number was requested */
    SD_R5_OUT_OF_RANGE,     /*The command's argument was out of the allowed range for this card*/
    SD_ERROR,
    SD_ERR_LONG_TIME_NO_RESPONS,
    SD_ERR_CMD41_CNT = 0xfffe
} SDIO_Error;


#define SD_CMD_NORESP             0
#define SD_CMD_SHORT             (CMD_FLAG_RESPONSE|CMD_FLAG_CRC_CHECK)
#define SD_CMD_LONG              (CMD_FLAG_RESPONSE|CMD_FLAG_LONG_CMD\
                                 |CMD_FLAG_CRC_CHECK)

#define SD_CMD_RSP               (SDCARD_CMDRSP_NORSP_END_INT\
                                 |SDCARD_CMDRSP_RSP_END_INT\
                                 |SDCARD_CMDRSP_TIMEOUT_INT\
                                 |SDCARD_CMDRSP_CMD_CRC_FAIL)

#define SD_DATA_RSP              (SDCARD_CMDRSP_DATA_REC_END_INT\
                                 |SDCARD_CMDRSP_DATA_CRC_FAIL\
                                 |SDCARD_CMDRSP_DATA_WR_END_INT\
                                 |SDCARD_CMDRSP_DATA_TIME_OUT_INT)


#define SD_DATA_DIR_RD           0
#define SD_DATA_DIR_WR           1

#define OCR_MSK_BUSY             0x80000000 // Busy flag
#define OCR_MSK_HC               0x40000000 // High Capacity flag
#define OCR_MSK_VOLTAGE_3_2V_3_3V           0x00100000 // Voltage 3.2V to 3.3V flag
#define OCR_MSK_VOLTAGE_ALL      0x00FF8000 // All Voltage flag

#define SD_DEFAULT_OCR           (OCR_MSK_VOLTAGE_ALL|OCR_MSK_HC)

#define SD_MAX_VOLT_TRIAL        (0xFF)

#define SD_DEFAULT_BLOCK_SIZE    512
#define SDCARD_TX_FIFO_THRD      (0x01) // 16byte
#define SDCARD_RX_FIFO_THRD      (0x01)

#define	CLK_26M                  0
#define	CLK_13M                  1
#define	CLK_6_5M                 2
#define	CLK_200K                 3

#define CMD_FLAG_RESPONSE        0x01
#define CMD_FLAG_LONG_CMD        0x02
#define CMD_FLAG_CRC_CHECK       0x04
#define CMD_FLAG_MASK            0x07

#define CMD_TIMEOUT_200K	5000	//about 5us per cycle (25ms)
#define DATA_TIMEOUT_200K	20000 //100ms

#define CMD_TIMEOUT_6_5_M	300000 //about 150ns per cycle (45ms)
#define DATA_TIMEOUT_6_5_M  3000000 //450ms

#define CMD_TIMEOUT_13M		600000 //about 77ns pr cycle (45ms)
#define DATA_TIMEOUT_13M	6000000 //450ms

#define CMD_TIMEOUT_26M		1200000//about 38ns pr cycle (45ms)
#define DATA_TIMEOUT_26M	12000000 //450ms

#define SDIO_RD_DATA             0
#define SDIO_WR_DATA             1
#define SDIO_RD_AF_WR            2

#define SDIO_DEF_LINE_MODE       4
#define SDIO_DEF_WORK_CLK        13


#define	SD_CLK_PIN_TIMEOUT1				0x1000
#define	SD_CLK_PIN_TIMEOUT2				0x8000
#define SD_CARD_OFFLINE				    0
#define SD_CARD_ONLINE				    1


//#define CONFIG_SDCARD_BUSWIDTH_4LINE

// interface function
void sdio_set_clock(UINT8 clk_index);
void sdio_gpio_config(void);
void sdio_clk_config(UINT8 enable);
void sdio_register_reset(void);
void sdio_sendcmd_function( UINT8 cmd_index, UINT32 flag,
                            UINT32 timeout, VOID *arg );
SDIO_Error sdio_wait_cmd_response(UINT32 cmd);
void sdio_get_cmdresponse_argument(UINT8 num, UINT32 *resp);
void sdio_setup_data(UINT32 data_dir, UINT32 byte_len);
void sdio_set_data_timeout(UINT32 timeout);

SDIO_Error sdcard_wait_receive_data(UINT8 *receive_buf);
//SDIO_Error sdcard_wait_write_end(void);
//SDIO_Error sdcard_write_data(UINT8 *writebuff, UINT32 block);
void driver_sdcard_recv_data_start(int timeout );
//uint8 sd_clk_is_attached(void);
//uint8 sd_is_attached(void);
//void sdio_register_reenable(void);
int wait_Receive_Data(void);

#endif

