#ifndef _QSPI_H_
#define _QSPI_H_

#define QSPI_DEBUG   0

#if QSPI_DEBUG
#define QSPI_PRT      os_printf
#define QSPI_WARN     warning_prf
#define QSPI_FATAL    fatal_prf
#else
#define QSPI_PRT      null_prf
#define QSPI_WARN     null_prf
#define QSPI_FATAL    null_prf
#endif

#if (CFG_SOC_NAME == SOC_BK7252N)
#define QSPI_BASE                           (0x00A02000)

#define REG_QSPI_DEVICE_ID                  (QSPI_BASE + 0x00 * 4)

#define REG_QSPI_VERSION_ID                 (QSPI_BASE + 0x01 * 4)

#define REG_QSPI_GLB_CTRL                   (QSPI_BASE + 0x02 * 4)
#define QSPI_SOFT_RESET                     (0x01UL << 0)
#define QSPI_BPS_CLKGATE                    (0x01UL << 1)

#define REG_QSPI_CORE_STATUS                (QSPI_BASE + 0x03 * 4)
#define QSPI_STATUS_IDLE                    (0x0)
#define QSPI_STATUS_SEND_CMD                (0x2)
#define QSPI_STATUS_DUMMY_CYCLE             (0x4)
#define QSPI_STATUS_WRITE_DATA              (0x8)
#define QSPI_STATUS_READ_DATA               (0x10)
#define QSPI_STATUS_DONE                    (0x20)
#define QSPI_STATUS_SPI_CSN_H               (0x40)

#define REG_QSPI_CMD_A_L                    (QSPI_BASE + 0x08 * 4)
#define QSPI_CMD_5_POSI                     (0)
#define QSPI_CMD_5_MASK                     (0xFF)
#define QSPI_CMD_6_POSI                     (8)
#define QSPI_CMD_6_MASK                     (0xFF)
#define QSPI_CMD_7_POSI                     (16)
#define QSPI_CMD_7_MASK                     (0xFF)
#define QSPI_CMD_8_POSI                     (24)
#define QSPI_CMD_8_MASK                     (0xFF)

#define REG_QSPI_CMD_A_H                    (QSPI_BASE + 0x09 * 4)
#define QSPI_CMD_1_POSI                     (0)
#define QSPI_CMD_1_MASK                     (0xFF)
#define QSPI_CMD_2_POSI                     (8)
#define QSPI_CMD_2_MASK                     (0xFF)
#define QSPI_CMD_3_POSI                     (16)
#define QSPI_CMD_3_MASK                     (0xFF)
#define QSPI_CMD_4_POSI                     (24)
#define QSPI_CMD_4_MASK                     (0xFF)

#define REG_QSPI_CMD_A_CFG_1                (QSPI_BASE + 0x0A * 4)
#define QSPI_CMD_1_LINE_POSI                (0)
#define QSPI_CMD_1_LINE_MASK                (0x3)
#define QSPI_CMD_2_LINE_POSI                (2)
#define QSPI_CMD_2_LINE_MASK                (0x3)
#define QSPI_CMD_3_LINE_POSI                (4)
#define QSPI_CMD_3_LINE_MASK                (0x3)
#define QSPI_CMD_4_LINE_POSI                (6)
#define QSPI_CMD_4_LINE_MASK                (0x3)
#define QSPI_CMD_5_LINE_POSI                (8)
#define QSPI_CMD_5_LINE_MASK                (0x3)
#define QSPI_CMD_6_LINE_POSI                (10)
#define QSPI_CMD_6_LINE_MASK                (0x3)
#define QSPI_CMD_7_LINE_POSI                (12)
#define QSPI_CMD_7_LINE_MASK                (0x3)
#define QSPI_CMD_8_LINE_POSI                (14)
#define QSPI_CMD_8_LINE_MASK                (0x3)

#define REG_QSPI_CMD_A_CFG_2                (QSPI_BASE + 0x0B * 4)
#define QSPI_DATA_LINE_POSI                 (14)
#define QSPI_DATA_LINE_MASK                 (0x3)
#define QSPI_DUMMY_CLK_POSI                 (16)
#define QSPI_DUMMY_CLK_MASK                 (0x7F)
#define QSPI_DUMMY_MODE_POSI                (24)
#define QSPI_DUMMY_MODE_MASK                (0x7)
#define QSPI_CMD_MODE_POSI                  (30)
#define QSPI_CMD_MODE_MASK                  (0x3)

#define REG_QSPI_CMD_B_L                    (QSPI_BASE + 0x0C * 4)
#define REG_QSPI_CMD_B_H                    (QSPI_BASE + 0x0D * 4)
#define REG_QSPI_CMD_B_CFG_1                (QSPI_BASE + 0x0E * 4)
#define REG_QSPI_CMD_B_CFG_2                (QSPI_BASE + 0x0F * 4)

#define REG_QSPI_CMD_C_L                    (QSPI_BASE + 0x10 * 4)
#define REG_QSPI_CMD_C_H                    (QSPI_BASE + 0x11 * 4)
#define REG_QSPI_CMD_C_CFG_1                (QSPI_BASE + 0x12 * 4)
#define REG_QSPI_CMD_C_CFG_2                (QSPI_BASE + 0x13 * 4)
#define QSPI_CMD_START                      (0x01 << 0)
#define QSPI_DATA_LEN_POSI                  (2)
#define QSPI_DATA_LEN_MASK                  (0x3FF)

#define REG_QSPI_CMD_D_L                    (QSPI_BASE + 0x14 * 4)
#define REG_QSPI_CMD_D_H                    (QSPI_BASE + 0x15 * 4)
#define REG_QSPI_CMD_D_CFG_1                (QSPI_BASE + 0x16 * 4)
#define REG_QSPI_CMD_D_CFG_2                (QSPI_BASE + 0x17 * 4)

#define REG_QSPI_SPI_CFG                    (QSPI_BASE + 0x18 * 4)
#define QSPI_SPI_EN                         (0x01UL << 0)
#define QSPI_CPOL                           (0x01UL << 1)
#define QSPI_CPHA                           (0x01UL << 2)
#define QSPI_IO2_IO3_MODE                   (0x01UL << 3)
#define QSPI_IO2                            (0x01UL << 4)
#define QSPI_IO3                            (0x01UL << 5)
#define QSPI_FORCE_SPI_CS_LOW               (0x01UL << 6)
#define QSPI_NSS_H_GEN_SCK                  (0x01UL << 7)
#define QSPI_CLK_RATE_POSI                  (8)
#define QSPI_CLK_RATE_MASK                  (0xFF)
#define QSPI_DIS_CMD_SCK                    (0x01UL << 16)
#define QSPI_LED_DAHB_RD_BPS                (0x01UL << 17)
#define QSPI_FIRST_BIT_MODE                 (0x01UL << 18)
#define QSPI_DAHB_TRANS_TYPE_POSI           (19)
#define QSPI_DAHB_TRANS_TYPE_MASK           (0x3)
#define QSPI_TX_FIFO_CLR_SYNC_SCK_BPS       (0x01UL << 21)
#define QSPI_IO_CPU_MEM_SEL                 (0x01UL << 22)
#define QSPI_SPI_RCV_4BYTE_MODE             (0x01UL << 23)
#define QSPI_SPI_CS_H_WAIT_POSI             (24)
#define QSPI_SPI_CS_H_WAIT_MASK             (0xFF)

#define REG_QSPI_FIFO_CFG                   (QSPI_BASE + 0x19 * 4)
#define QSPI_SW_RST_FIFO                    (0x01UL << 1)

#define REG_QSPI_SPI_INT_EN                 (QSPI_BASE + 0x1A * 4)
#define QSPI_INT_SPI_RX_DONE                (0x01UL << 0)
#define QSPI_INT_SPI_TX_DONE                (0x01UL << 1)
#define QSPI_INT_CMD_START_DONE             (0x01UL << 2)
#define QSPI_INT_SPI_DONE                   (0x01UL << 3)
#define QSPI_INT_CMD_START_FAIL             (0x01UL << 4)
#define QSPI_INT_ADDR_CNT                   (0x01UL << 5)

#define REG_QSPI_SPI_STATUS_CLR             (QSPI_BASE + 0x1B * 4)
#define QSPI_CLR_SPI_RX_DONE                (0x01UL << 0)
#define QSPI_CLR_SPI_TX_DONE                (0x01UL << 1)
#define QSPI_CLR_CMD_START_DONE             (0x01UL << 2)
#define QSPI_CLR_SPI_DONE                   (0x01UL << 3)
#define QSPI_CLR_CMD_START_FAIL             (0x01UL << 4)
#define QSPI_CLR_ADDR_CNT                   (0x01UL << 5)

#define REG_QSPI_SPI_STATUS                 (QSPI_BASE + 0x1C * 4)
#define QSPI_SPI_RX_DONE                    (0x01UL << 0)
#define QSPI_SPI_TX_DONE                    (0x01UL << 1)
#define QSPI_CMD_START_DONE                 (0x01UL << 2)
#define QSPI_CMD_START_DONE_POSI            (2)
#define QSPI_SPI_DONE                       (0x01UL << 3)
#define QSPI_CMD_START_FAIL                 (0x01UL << 4)
#define QSPI_SPI_CS                         (0x01UL << 12)
#define QSPI_SPI_RX_BUSY                    (0x01UL << 13)
#define QSPI_SPI_TX_BUSY                    (0x01UL << 14)
#define QSPI_FIFO_EMPTY                     (0x01UL << 16)

#define REG_QSPI_LED_WR_CONT_CMD            (QSPI_BASE + 0x1D * 4)
#define REG_QSPI_CMD_LAST_ADDR              (QSPI_BASE + 0x1E * 4)
#define REG_QSPI_LCD_HEAD_CMD_N(N)          (QSPI_BASE + (0x1F + (N)) * 4)

#define REG_QSPI_LCD_HEAD_CNT               (QSPI_BASE + 0x23 * 4)
#define QSPI_LCD_HEAD_H_CNT_POSI            (0)
#define QSPI_LCD_HEAD_H_CNT_MASK            (0xFFFF)
#define QSPI_LCD_HEAD_V_CNT_POSI            (16)
#define QSPI_LCD_HEAD_V_CNT_MASK            (0xFFFF)

#define REG_QSPI_LCD_HEAD_CFG               (QSPI_BASE + 0x24 * 4)
#define QSPI_LCD_HEAD_DLY_POSI              (0)
#define QSPI_LCD_HEAD_DLY_MASK              (0xFF)
#define QSPI_LCD_HEAD_LEN_POSI              (8)
#define QSPI_LCD_HEAD_LEN_MASK              (0x3F)
#define QSPI_LCD_HEAD_SEL                   (0x01UL << 14)
#define QSPI_LCD_HEAD_CLR                   (0x01UL << 15)

#define REG_QSPI_DATA_FIFO                  (QSPI_BASE + 0x40 * 4)

#define QSPI_READ_CMD(type,idx,v)           do { \
                                                if(idx < 5) \
                                                    v = REG_READ(QSPI_BASE + ((0x8 + type * 4) + 1) * 4) \
                                                else \
                                                    v = REG_READ(QSPI_BASE + (0x8 + type * 4) * 4) \
                                            }while(0)

#define QSPI_WRITE_CMD(type,idx,v)          do { \
                                                if(idx < 5) \
                                                    REG_WRITE(QSPI_BASE + ((0x8 + type * 4) + 1) * 4, v) \
                                                else \
                                                    REG_WRITE(QSPI_BASE + (0x8 + type * 4) * 4, v) \
                                            }while(0)

#define QSPI_CMD_N_POSI(N)                  ((((N)-1)%4) << 3)
#define QSPI_CMD_N_MASK                     (0xFF)

#define QSPI_READ_CMD_LINE(type,v)          do { \
                                                v = REG_READ(QSPI_BASE + (0xA + type * 4) * 4) \
                                            }while(0)

#define QSPI_WRITE_CMD_LINE(type,v)         do { \
                                                REG_WRITE(QSPI_BASE + (0xA + type * 4) * 4, v) \
                                            }while(0)

#define QSPI_CMD_N_LINE_POSI(N)             (((N)-1) << 1)
#define QSPI_CMD_N_LINE_MASK                (0x3)

#define QSPI_READ_CMD_CFG(type,v)           do { \
                                                v = REG_READ(QSPI_BASE + (0xB + type * 4) * 4) \
                                            }while(0)

#define QSPI_WRITE_CMD_CFG(type,v)          do { \
                                                REG_WRITE(QSPI_BASE + (0xB + type * 4) * 4, v) \
                                            }while(0)

#else
#define QSPI_BASE                           (0x0080D000)

#define REG_QSPI_SW_CMD                 	(QSPI_BASE + 0x00 * 4)
#define REG_QSPI_SW_ADDR                 	(QSPI_BASE + 0x01 * 4)
#define REG_QSPI_SW_DUM                 	(QSPI_BASE + 0x02 * 4)
#define REG_QSPI_SW_DAT                 	(QSPI_BASE + 0x03 * 4)
#define REG_QSPI_ADDR_VID_INI               (QSPI_BASE + 0x04 * 4)
#define REG_QSPI_SW_OP                 		(QSPI_BASE + 0x09 * 4)
#define REG_QSPI_GE0_DATA               	(QSPI_BASE + 0x0C * 4)
#define REG_QSPI_GE1_DATA               	(QSPI_BASE + 0x0D * 4)
#define REG_QSPI_GE0_TH                 	(QSPI_BASE + 0x12 * 4)
#define REG_QSPI_GE1_TH                 	(QSPI_BASE + 0x13 * 4)
#define REG_QSPI_GE0_DEP                 	(QSPI_BASE + 0x17 * 4)
#define REG_QSPI_GE1_DEP                 	(QSPI_BASE + 0x18 * 4)
#define REG_QSPI_ENABLE						(QSPI_BASE + 0x1A * 4)
#define QSPI_CTRL							(QSPI_BASE + 0x1C * 4)

#define QSPI_DCACHE_WR_CMD					(QSPI_BASE + 0x28 * 4)
#define QSPI_DCACHE_WR_CMD_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_WR_ADDR					(QSPI_BASE + 0x29 * 4)
#define QSPI_DCACHE_WR_ADDR_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_WR_DUM					(QSPI_BASE + 0x2A * 4)

#define QSPI_DCACHE_WR_DAT					(QSPI_BASE + 0x2B * 4)
#define QSPI_DCACHE_WR_DAT_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_RD_CMD					(QSPI_BASE + 0x24 * 4)
#define QSPI_DCACHE_ED_CMD_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_RD_ADDR					(QSPI_BASE + 0x25 * 4)
#define QSPI_DCACHE_RD_ADDR_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_RD_DUM					(QSPI_BASE + 0x26 * 4)
#define QSPI_DCACHE_RD_DAT					(QSPI_BASE + 0x27 * 4)
#define QSPI_DCACHE_RD_DAT_ENABLE			(0x01 << 0)

#define QSPI_DCACHE_REQUEST					(QSPI_BASE + 0x2C * 4)
#define REG_QSPI_FIFO_STATUS       			(QSPI_BASE + 0x35 * 4)
#define REG_QSPI_INT_STATUS        			(QSPI_BASE + 0x36 * 4)
#endif

static UINT32 qspi_open(UINT32 op_flag);
static UINT32 qspi_close(void);
static UINT32 qspi_ctrl(UINT32 cmd, void *param);
#endif //_QSPI_H_

