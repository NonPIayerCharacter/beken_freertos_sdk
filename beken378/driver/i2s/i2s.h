#ifndef _I2S_H_
#define _I2S_H_

#define NUMBER_ROUND_UP(a,b)        ((a) / (b) + (((a) % (b)) ? 1 : 0))
#define NUMBER_ROUND_DOWN(a,b)      ((a) / (b))

#if (CFG_SOC_NAME == SOC_BK7252N)
#define I2S_BASE                     (0x00A06100)
#define I2S_DEV_ID                   (I2S_BASE + 0 * 4)
#define I2S_VER_ID                   (I2S_BASE + 1 * 4)
#define I2S_CLK_RST                  (I2S_BASE + 2 * 4)
#define I2S_CLK_RST_CLKG_BYPASS_POS  (1 << 1)
#define I2S_CLK_RST_SOFT_RESET_POS   (1 << 0)
#define I2S_STATE                    (I2S_BASE + 3 * 4)
#else
#define I2S_BASE                     (0x00802500)
#endif

#if (CFG_SOC_NAME == SOC_BK7252N)
#define PCM_CTRL                     (I2S_BASE + 4 * 4)
#else
#define PCM_CTRL                     (I2S_BASE + 0 * 4)
#endif
#define BITRATIO_POSI                (0)
#define BITRATIO_MASK                (0xFF)
#define SMPRATIO_POSI                (8)
#define SMPRATIO_MASK                (0x1F)
#define PCM_DLEN_POSI                (13)
#define PCM_DLEN_MASK                (0x07)
#define DATALEN_POSI                 (16)
#define DATALEN_MASK                 (0x1F)
#define SYNCLEN_POSI                 (21)
#define SYNCLEN_MASK                 (0x07UL <<21)
#define LSB_FIRST                    (0x01UL << 24)
#define SCK_INV                      (0x01UL << 25)
#define LRCK_RP                      (0x01UL << 26)
#define MODE_SEL_POSI                (27)
#define MODE_SEL_MASK                (0x07)
#define MSTEN                        (0x01UL << 30)
#define SLAVEEN                      (0 << 30)
#define I2S_PCM_EN                   (0x01UL << 31)

#if (CFG_SOC_NAME == SOC_BK7252N)
#define PCM_CN                       (I2S_BASE + 5 * 4)
#define RX_INT_EN                    (0x01UL << 0)
#define TX_INT_EN                    (0x01UL << 1)
#define RX_OVF_EN                    (0x01UL << 2)
#define TX_UDF_EN                    (0x01UL << 3)
#define RX_FIFO_LEVEL_POSI           (4)
#define RX_FIFO_LEVEL_MASK           (0x03)
#define TX_FIFO_LEVEL_POSI           (6)
#define TX_FIFO_LEVEL_MASK           (0x03)
#define TX_FIFO_CLR                  (0x01UL << 8)
#define RX_FIFO_CLR                  (0x01UL << 9)
#define SMPRATIO_H2B_POSI            (10)
#define SMPRATIO_H2B_MASK            (0x3)
#define SMPRATIO_H4B_POSI            (12)
#define SMPRATIO_H4B_MASK            (0xF)
#define LRCOM_STORE                  (0x01UL << 16)
#define PARALLEL_EN                  (0x01UL << 17)
#else
#define PCM_CN                       (I2S_BASE + 1 * 4)
#define RX_INT_EN                    (0x01UL << 0)
#define TX_INT0_EN                   (0x01UL << 1)
#define TX_INT1_EN                   (0x01UL << 2)
#define TX_INT2_EN                   (0x01UL << 3)
#define RX_OVF_EN                    (0x01UL << 4)
#define TX_UDF0_EN                   (0x01UL << 5)
#define TX_UDF1_EN                   (0x01UL << 6)
#define TX_UDF2_EN                   (0x01UL << 7)
#define RX_FIFO_LEVEL_POSI           (8)
#define RX_FIFO_LEVEL_MASK           (0x03)
#define TX_FIFO0_LEVEL_POSI          (10)
#define TX_FIFO0_LEVEL_MASK          (0x03)
#define TX_FIFO1_LEVEL_POSI          (12)
#define TX_FIFO1_LEVEL_MASK          (0x03)
#define TX_FIFO2_LEVEL_POSI          (14)
#define TX_FIFO2_LEVEL_MASK          (0x03)
#define RX_FIFO_CLR                  (0x01UL << 16)
#define TX_FIFO0_CLR                 (0x01UL << 17)
#define TX_FIFO1_CLR                 (0x01UL << 18)
#define TX_FIFO2_CLR                 (0x01UL << 19)
#endif

#if (CFG_SOC_NAME == SOC_BK7252N)
#define PCM_STAT                     (I2S_BASE + 6 * 4)
#define RX_INT                       (0x01UL << 0)
#define TX_INT                       (0x01UL << 1)
#define RX_OVF                       (0x01UL << 2)
#define TX_UDF                       (0x01UL << 3)
#define RX_FIFO_RD_READY             (0x01UL << 4)
#define TX_FIFO_WR_READY             (0x01UL << 5)
#else
#define PCM_STAT                     (I2S_BASE + 2 * 4)
#define RX_INT                       (0x01UL << 0)
#define TX_INT0                      (0x01UL << 1)
#define TX_INT1                      (0x01UL << 2)
#define TX_INT2                      (0x01UL << 3)
#define RX_OVF                       (0x01UL << 4)
#define TX_UDF0                      (0x01UL << 5)
#define TX_UDF1                      (0x01UL << 6)
#define TX_UDF2                      (0x01UL << 7)
#define RX_FIFO0_EMPTY               (0x01UL << 8)
#define RX_FIFO0_FULL                (0x01UL << 9) // modify according 7251 datasheet
#define TX_FIFO0_FULL                (0x01UL << 10)
#define TX_FIFO0_EMPTY               (0x01UL << 11) // modify according 7251 datasheet
#define TX_FIFO1_FULL                (0x01UL << 12)
#define TX_FIFO1_ALMOST_FULL         (0x01UL << 13)
#define TX_FIFO2_FULL                (0x01UL << 14)
#define TX_FIFO2_ALMOST_FULL         (0x01UL << 15)
#endif

#if (CFG_SOC_NAME == SOC_BK7252N)
#define PCM_DAT0                     (I2S_BASE + 7 * 4)

#define PCM_CN_LT2                   (I2S_BASE + 8 * 4)
#define RX_INT2_EN                   (0x01UL << 0)
#define TX_INT2_EN                   (0x01UL << 1)
#define RX_OVF2_EN                   (0x01UL << 2)
#define TX_UDF2_EN                   (0x01UL << 3)
#define RX_INT3_EN                   (0x01UL << 4)
#define TX_INT3_EN                   (0x01UL << 5)
#define RX_OVF3_EN                   (0x01UL << 6)
#define TX_UDF3_EN                   (0x01UL << 7)
#define RX_INT4_EN                   (0x01UL << 8)
#define TX_INT4_EN                   (0x01UL << 9)
#define RX_OVF4_EN                   (0x01UL << 10)
#define TX_UDF4_EN                   (0x01UL << 11)

#define PCM_CN_LT2_RX_N_INT_EN(N)    (1 << 0) << ((N-2) * 4)
#define PCM_CN_LT2_TX_N_INT_EN(N)    (1 << 1) << ((N-2) * 4)
#define PCM_CN_LT2_RX_N_OVF_EN(N)    (1 << 2) << ((N-2) * 4)
#define PCM_CN_LT2_TX_N_UDF_EN(N)    (1 << 3) << ((N-2) * 4)

#define PCM_STAT_LT2                 (I2S_BASE + 9 * 4)
#define RX_INT2                      (0x01UL << 0)
#define TX_INT2                      (0x01UL << 1)
#define RX_OVF2                      (0x01UL << 2)
#define TX_UDF2                      (0x01UL << 3)
#define RX_INT3                      (0x01UL << 4)
#define TX_INT3                      (0x01UL << 5)
#define RX_OVF3                      (0x01UL << 6)
#define TX_UDF3                      (0x01UL << 7)
#define RX_INT4                      (0x01UL << 8)
#define TX_INT4                      (0x01UL << 9)
#define RX_OVF4                      (0x01UL << 10)
#define TX_UDF4                      (0x01UL << 11)

#define PCM_DAT2                     (I2S_BASE + 0xA * 4)

#define PCM_DAT3                     (I2S_BASE + 0xB * 4)

#define PCM_DAT4                     (I2S_BASE + 0xC * 4)
#else
#define PCM_DAT0                     (I2S_BASE + 3 * 4)

#define PCM_DAT1                     (I2S_BASE + 4 * 4)

#define PCM_DAT2                     (I2S_BASE + 5 * 4)

#define	PCM_CFG2					 (I2S_BASE + 6 * 4)
#endif

#define TXFIFO_TXFIFO_MODE       	 6
#define TXFIFO_BITSEQ_MODE        	 4
#define TXFIFO_DOWN_SMPRATIO     	 0

//#define I2S_SYS_CLK     24576000
#define I2S_SYS_CLK     48000000

#define MASTER				1
#define SLAVE				0

#define SAMPLE_RATE8K		8000
#define SAMPLE_RATE16K		16000
#define SAMPLE_RATE_44_1K	44100
#define SAMPLE_RATE_48K		48000

#define DATA_WIDTH_16BIT	16
#define DATA_WIDTH_32BIT	32

#if (CFG_SOC_NAME == SOC_BK7252N)
#define FIFO_LEVEL_1        0
#define FIFO_LEVEL_8        1
#define FIFO_LEVEL_16       2
#define FIFO_LEVEL_24       3
#else
#define FIFO_LEVEL_16		0
#define FIFO_LEVEL_32		1
#define FIFO_LEVEL_48		2
#endif

/*******************************************************************************
* mode micros
*******************************************************************************/
#if (CFG_SOC_NAME == SOC_BK7252N)
#define I2S_MODE							(0 << 27)
#define I2S_LEFT_JUSTIFIED					(1 << 27)
#define I2S_RIGHT_JUSTIFIED					(2 << 27)
#define I2S_RESERVE							(3 << 27)
#define I2S_SHORT_FRAME_SYNC				(4 << 27)
#define I2S_LONG_FRAME_SYNC					(5 << 27)
#define I2S_NORMAL_2B_D						(6 << 27)
#define I2S_DELAY_2B_D						(7 << 27)

#define I2S_LRCK_NO_TURN					(0 << 26)
#define I2S_SCK_NO_TURN						(0 << 25)
#define I2S_MSB_FIRST						(0 << 24)

#define I2S_SYNC_LENGTH_BIT					(21)
#define I2S_PCM_DATA_LENGTH_BIT				(13)
#else
#define I2S_MODE							(0 << 0)
#define I2S_LEFT_JUSTIFIED					(1 << 0)
#define I2S_RIGHT_JUSTIFIED					(2 << 0)
#define I2S_RESERVE							(3 << 0)
#define I2S_SHORT_FRAME_SYNC				(4 << 0)
#define I2S_LONG_FRAME_SYNC					(5 << 0)
#define I2S_NORMAL_2B_D						(6 << 0)
#define I2S_DELAY_2B_D						(7 << 0)

#define I2S_LRCK_NO_TURN					(0 << 3)
#define I2S_SCK_NO_TURN						(0 << 4)
#define I2S_MSB_FIRST						(0 << 5)

#define I2S_SYNC_LENGTH_BIT					(8)
#define I2S_PCM_DATA_LENGTH_BIT				(12)
#endif

/*******************************************************************************
* Function Declarations
*******************************************************************************/
UINT32 i2s_configure(UINT32 fifo_level, UINT32 sample_rate, UINT32 bits_per_sample, UINT32 mode);

UINT32 i2s_transfer(UINT32 *i2s_send_buf , UINT32 *i2s_recv_buf, UINT32 count , UINT32 param );

#endif //_I2S_H_
