#ifndef __GDMA_PUB_H__
#define __GDMA_PUB_H__

#include "sys_config.h"
#include "bk_err.h"

#if CFG_GENERAL_DMA
#define GDMA_FAILURE                (1)
#define GDMA_SUCCESS                (0)

typedef void (*DMA_ISR_FUNC)(UINT32 val);

#define BK_ERR_DMA_ID             (BK_ERR_DMA_BASE - 1) /**< DMA id is invalid */
#define BK_ERR_DMA_NOT_INIT       (BK_ERR_DMA_BASE - 2) /**< DMA driver not init */
#define BK_ERR_DMA_ID_NOT_INIT    (BK_ERR_DMA_BASE - 3) /**< DMA id not init */
#define BK_ERR_DMA_ID_NOT_START   (BK_ERR_DMA_BASE - 4) /**< DMA id not start */
#define BK_ERR_DMA_INVALID_ADDR   (BK_ERR_DMA_BASE - 5) /**< DMA addr is invalid */
#define BK_ERR_DMA_ID_REINIT      (BK_ERR_DMA_BASE - 6) /**< DMA id has inited, if reinit,please de-init firstly */
#define BK_ERR_DMA_TRANS_LEN      (BK_ERR_DMA_BASE - 7) /**< DMA  trans len  is invalid */


#define GDMA_DEV_NAME                "generaldma"
#define GDMA_CMD_MAGIC              (0x0e809000)

enum
{
    CMD_GDMA_SET_DMA_ENABLE = GDMA_CMD_MAGIC + 1,
    CMD_GDMA_CFG_FIN_INT_ENABLE,
    CMD_GDMA_CFG_HFIN_INT_ENABLE,
    CMD_GDMA_CFG_WORK_MODE,
    CMD_GDMA_CFG_SRCDATA_WIDTH,
    CMD_GDMA_CFG_DSTDATA_WIDTH,
    CMD_GDMA_CFG_SRCADDR_INCREASE,
    CMD_GDMA_CFG_DSTADDR_INCREASE,
    CMD_GDMA_CFG_SRCADDR_LOOP,
    CMD_GDMA_CFG_DSTADDR_LOOP,
    CMD_GDMA_SET_CHNL_PRIO,
    CMD_GDMA_SET_TRANS_LENGTH,
    CMD_GDMA_SET_DST_START_ADDR,
    CMD_GDMA_SET_SRC_START_ADDR,
    CMD_GDMA_SET_DST_LOOP_ENDADDR,
    CMD_GDMA_SET_DST_LOOP_STARTADDR,
    CMD_GDMA_SET_SRC_LOOP_ENDADDR,
    CMD_GDMA_SET_SRC_LOOP_STARTADDR,
    CMD_GDMA_SET_DST_REQ_MUX,
    CMD_GDMA_SET_SRC_REQ_MUX,
    CMD_GDMA_SET_DTCM_WRITE_WORD,
    CMD_GDMA_GET_REMAIN_LENGTH,
    CMD_GDMA_SET_FIN_CNT,
    CMD_GDMA_SET_HFIN_CNT,
    CMD_GDMA_SET_PRIO_MODE,
    CMD_GDMA_GET_FIN_INT_STATUS,
    CMD_GDMA_CLR_FIN_INT_STATUS,
    CMD_GDMA_GET_HFIN_INT_STATUS,
    CMD_GDMA_CLR_HFIN_INT_STATUS,
    CMD_GDMA_CFG_TYPE0,
    CMD_GDMA_CFG_TYPE1,
    CMD_GDMA_CFG_TYPE2,
    CMD_GDMA_CFG_TYPE3,
    CMD_GDMA_CFG_TYPE4,
    CMD_GDMA_CFG_TYPE5,
    CMD_GDMA_CFG_TYPE6,
    CMD_GDMA_ENABLE,
    CMD_GDMA_GET_LEFT_LEN,
    CMD_GDMA_SET_SRC_PAUSE_ADDR,
    CMD_GDMA_GET_SRC_PAUSE_ADDR,
    CMD_GDMA_SET_DST_PAUSE_ADDR,
    CMD_GDMA_GET_DST_PAUSE_ADDR,
    CMD_GDMA_GET_SRC_READ_ADDR,
    CMD_GDMA_GET_DST_WRITE_ADDR,
};

#if (CFG_SOC_NAME == SOC_BK7252N)
#define GDMA_X_DST_DTCM_WR_REQ            (0x0)
#define GDMA_X_DST_UART1_TX_REQ           (0x1)
#define GDMA_X_DST_UART2_TX_REQ           (0x2)
#define GDMA_X_DST_UART3_TX_REQ           (0x3)
#define GDMA_X_DST_GSPI_TX_REQ            (0x4)
#define GDMA_X_DST_SDIO_TX_REQ            (0x5)
#define GDMA_X_DST_HSSPI_TX_REQ           (0x6)
#define GDMA_X_DST_JPEG_TX_REQ            (0x7)
#define GDMA_X_DST_I2S_TX_REQ             (0x8)
#define GDMA_X_DST_AUDIO_TX_REQ           (0x9)
#define GDMA_X_DST_DTCM_RD_REQ            (0x10)
#define GDMA_X_DST_UART1_RX_REQ           (0x11)
#define GDMA_X_DST_UART2_RX_REQ           (0x12)
#define GDMA_X_DST_UART3_RX_REQ           (0x13)
#define GDMA_X_DST_GSPI_RX_REQ            (0x14)
#define GDMA_X_DST_SDIO_RX_REQ            (0x15)
#define GDMA_X_DST_HSSPI_RX_REQ           (0x16)
#define GDMA_X_DST_JPEG_RX_REQ            (0x17)
#define GDMA_X_DST_I2S_RX_REQ             (0x18)
#define GDMA_X_DST_AUDIO_RX_REQ           (0x19)

#define GDMA_X_SRC_DTCM_WR_REQ            (0x0)
#define GDMA_X_SRC_UART1_TX_REQ           (0x1)
#define GDMA_X_SRC_UART2_TX_REQ           (0x2)
#define GDMA_X_SRC_UART3_TX_REQ           (0x3)
#define GDMA_X_SRC_GSPI_TX_REQ            (0x4)
#define GDMA_X_SRC_SDIO_TX_REQ            (0x5)
#define GDMA_X_SRC_HSSPI_TX_REQ           (0x6)
#define GDMA_X_SRC_JPEG_TX_REQ            (0x7)
#define GDMA_X_SRC_I2S_TX_REQ             (0x8)
#define GDMA_X_SRC_AUDIO_TX_REQ           (0x9)
#define GDMA_X_SRC_DTCM_RD_REQ            (0x10)
#define GDMA_X_SRC_UART1_RX_REQ           (0x11)
#define GDMA_X_SRC_UART2_RX_REQ           (0x12)
#define GDMA_X_SRC_UART3_RX_REQ           (0x13)
#define GDMA_X_SRC_GSPI_RX_REQ            (0x14)
#define GDMA_X_SRC_SDIO_RX_REQ            (0x15)
#define GDMA_X_SRC_HSSPI_RX_REQ           (0x16)
#define GDMA_X_SRC_JPEG_RX_REQ            (0x17)
#define GDMA_X_SRC_I2S_RX_REQ             (0x18)
#define GDMA_X_SRC_AUDIO_RX_REQ           (0x19)

/* 0xA~0xF and 0x1A~0x1F are all reserved bits */
#define GDMA_SRC_DST_RESERVE              (0xA)

#else // (CFG_SOC_NAME == SOC_BK7252N)
#define GDMA_X_DST_DTCM_WR_REQ            (0x0)
#define GDMA_X_DST_HSSPI_TX_REQ           (0x1)
#define GDMA_X_DST_AUDIO_TX_REQ           (0x2)
#define GDMA_X_DST_SDIO_TX_REQ            (0x3)
#define GDMA_X_DST_UART1_TX_REQ           (0x4)
#define GDMA_X_DST_UART2_TX_REQ           (0x5)
#define GDMA_X_DST_I2S_TX_REQ             (0x6)
#define GDMA_X_DST_GSPI_TX_REQ            (0x7)
#define GDMA_X_DST_PSRAM_V_WR_REQ         (0x9)
#define GDMA_X_DST_PSRAM_A_WR_REQ         (0xA)
#define GDMA_X_SRC_DTCM_RD_REQ            (0x0)
#define GDMA_X_SRC_HSSPI_RX_REQ           (0x1)
#define GDMA_X_SRC_AUDIO_RX_REQ           (0x2)
#define GDMA_X_SRC_SDIO_RX_REQ            (0x3)
#define GDMA_X_SRC_UART1_RX_REQ           (0x4)
#define GDMA_X_SRC_UART2_RX_REQ           (0x5)
#define GDMA_X_SRC_I2S_RX_REQ             (0x6)
#define GDMA_X_SRC_GSPI_RX_REQ            (0x7)
#define GDMA_X_SRC_JPEG_WR_REQ            (0x8)
#define GDMA_X_SRC_PSRAM_V_RD_REQ         (0x9)
#define GDMA_X_SRC_PSRAM_A_RD_REQ         (0xA)
#define GDMA_X_SRC_DST_RESERVE            (0xB)
#endif // (CFG_SOC_NAME == SOC_BK7252N)

typedef enum
{
    GDMA_CHANNEL_0  = 0,
    GDMA_CHANNEL_1,
    GDMA_CHANNEL_2,
    GDMA_CHANNEL_3,

#if (CFG_SOC_NAME != SOC_BK7231)
    GDMA_CHANNEL_4,
    GDMA_CHANNEL_5,
#endif // (CFG_SOC_NAME != SOC_BK7231)

#if (CFG_SOC_NAME == SOC_BK7252N)
    GDMA_CHANNEL_6,
    GDMA_CHANNEL_7,
#endif

    GDMA_CHANNEL_MAX,
    DMA_ID_MAX = GDMA_CHANNEL_MAX
}dma_id_t;

#define GDMA_TYPE_0         (0U)  // no loop src, no loop dst, no register
#define GDMA_TYPE_1         (1U)  // loop src,    no loop dst, no register
#define GDMA_TYPE_2         (2U)  // no loop src, loop dst, no register
#define GDMA_TYPE_3         (3U)  // loop src,    loop dst, no register

#define GDMA_TYPE_4         (4U)  // loop src,    no loop dst, register
#define GDMA_TYPE_5         (5U)  // no loop src, loop dst, register
#define GDMA_TYPE_6         (6U)  // loop src,    loop dst, register

typedef struct gdmacfg_types_st
{
    UINT32 channel;
    UINT32 prio;
    UINT8 dstptr_incr;
    UINT8 srcptr_incr;
    UINT8 dstdat_width;
    UINT8 srcdat_width;

    void *src_start_addr;
    void *dst_start_addr;

    UINT8 src_module;
    UINT8 dst_module;
    DMA_ISR_FUNC fin_handler;
    DMA_ISR_FUNC half_fin_handler;

    union
    {
        struct
        {
            void *src_loop_start_addr;
            void *src_loop_end_addr;
        } type1; // loop src,    no loop dst,    no register

        struct
        {
            void *dst_loop_start_addr;
            void *dst_loop_end_addr;
        } type2; // no loop src, loop dst,    no register

        struct
        {
            void *src_loop_start_addr;
            void *src_loop_end_addr;
            void *dst_loop_start_addr;
            void *dst_loop_end_addr;
        } type3; // loop src,    loop dst,    no register

        struct
        {
            void *src_loop_start_addr;
            void *src_loop_end_addr;
        } type4; // loop src,    no loop dst,    register

        struct
        {
            void *dst_loop_start_addr;
            void *dst_loop_end_addr;
        } type5; // no loop src, loop dst,    register

        struct
        {
            void *src_loop_start_addr;
            void *src_loop_end_addr;
            void *dst_loop_start_addr;
            void *dst_loop_end_addr;
        } type6; // loop src,    loop dst,    register
    } u;
} GDMACFG_TPYES_ST, *GDMACFG_TPYES_PTR;

typedef struct gdma_do_st
{
    UINT32 channel;
    void *src_addr;
    void *dst_addr;
    UINT32 length;
} GDMA_DO_ST, *GDMA_DO_PTR;


typedef struct generdam_cfg_st
{
    UINT32 param;
    UINT32 channel;
} GDMA_CFG_ST, *GDMA_CFG_PTR;

typedef enum{
    GDMA_PRIO_MODE_ROUND_ROBIN,
    GDMA_PRIO_MODE_FIXED_PRIORITY
} GDMA_PRIO_MODE;

typedef enum{
    GDMA_CHANNEL_SECURE_ATTR_NON_SECURE,
    GDMA_CHANNEL_SECURE_ATTR_SECURE
} GDMA_CHANNEL_SECURE_ATTR;

typedef enum{
    GDMA_CHANNEL_PRIVILEGED_ATTR_NON_PRIVILEGED,
    GDMA_CHANNEL_PRIVILEGED_ATTR_PRIVILEGED
} GDMA_CHANNEL_PRIVILEGED_ATTR;

typedef enum{
    GDMA_CHANNEL_INT0_STATUS_DIS,
    GDMA_CHANNEL_INT0_STATUS_EN
} GDMA_CHANNEL_INT0_STATUS;

typedef enum{
    GDMA_CHANNEL_INT_ALLOCATE_INT0,
    GDMA_CHANNEL_INT_ALLOCATE_REVD0,
    GDMA_CHANNEL_INT_ALLOCATE_REVD1,
    GDMA_CHANNEL_INT_ALLOCATE_REVD2
} GDMA_CHANNEL_INT_ALLOCATE;

typedef enum{
    GDMA_CHANNEL_SRC_DATA_WIDTH_8BIT,
    GDMA_CHANNEL_SRC_DATA_WIDTH_16BIT,
    GDMA_CHANNEL_SRC_DATA_WIDTH_32BIT,
    GDMA_CHANNEL_SRC_DATA_WIDTH_REVD
} GDMA_CHANNEL_SRC_DATA_WIDTH;

typedef enum{
    GDMA_CHANNEL_DEST_DATA_WIDTH_8BIT,
    GDMA_CHANNEL_DEST_DATA_WIDTH_16BIT,
    GDMA_CHANNEL_DEST_DATA_WIDTH_32BIT,
    GDMA_CHANNEL_DEST_DATA_WIDTH_REVD
} GDMA_CHANNEL_DEST_DATA_WIDTH;

void gdma_init(void);
void gdma_exit(void);
void *gdma_memcpy(void *out, const void *in, UINT32 n);
UINT32 gdma_ctrl(UINT32 cmd, void *param);

#if (CFG_SOC_NAME == SOC_BK7252N)
uint32_t bk_dma_get_transfer_len_max(dma_id_t id);
bk_err_t bk_dma_set_transfer_len(dma_id_t id, uint32_t tran_len);
bk_err_t bk_dma_set_dest_addr(dma_id_t id, uint32_t start_addr, uint32_t end_addr);
bk_err_t bk_dma_set_src_addr(dma_id_t id, uint32_t start_addr, uint32_t end_addr);
bk_err_t bk_dma_start(dma_id_t id);
bk_err_t bk_dma_enable_finish_interrupt(dma_id_t id);
bk_err_t bk_dma_register_isr(dma_id_t id, void *half_finish_isr, void *finish_isr);
bk_err_t bk_dma_stop(dma_id_t id);
#endif // SOC_BK7252N
#endif // CFG_GENERAL_DMA
#endif
// eof

