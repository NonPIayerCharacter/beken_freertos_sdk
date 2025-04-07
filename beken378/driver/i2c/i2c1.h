#ifndef _I2C1_H_
#define _I2C1_H_

#include "uart_pub.h"
// #define I2C1_DEBUG

#ifdef I2C1_DEBUG
#define I2C1_PRT                 os_printf
#define I2C1_WPRT                warning_prf
#define I2C1_EPRT                os_printf
#define I2C1_DEBUG_PRINTF		 os_printf
#else
#define I2C1_PRT                 os_null_printf
#define I2C1_WPRT                os_null_printf
#define I2C1_EPRT                os_printf
#define I2C1_DEBUG_PRINTF		 os_null_printf
#endif

#define I2C1_BASE_ADDR                       (0x0802300)

#if (CFG_SOC_NAME == SOC_BK7252N)

#define REG_I2C1_CONFIG                      (I2C1_BASE_ADDR + 4 * 0)
#define I2C1_IDLE_CR_POSI                      (0)
#define I2C1_IDLE_CR_MASK                      (0x7)
#define I2C1_SCL_CR_POSI                       (3)
#define I2C1_SCL_CR_MASK                       (0x7)
#define I2C1_FREQ_DIV_POSI                     (6)
#define I2C1_FREQ_DIV_MASK                     (0x3FF)
#define I2C1_SLV_ADDR_POSI                     (16)
#define I2C1_SLV_ADDR_MASK                     (0x3FF)
#define I2C1_SMB_CS_POSI                       (26)
#define I2C1_SMB_CS_MASK                       (0x3)
#define I2C1_SMB_TOE                           (1 << 28)
#define I2C1_SMB_FTE                           (1 << 29)
#define I2C1_INH                               (1 << 30)
#define I2C1_ENSMB                             (1 << 31)

#define REG_I2C1_STA                         (I2C1_BASE_ADDR + 4 * 1)
#define I2C1_SMBUS_SI                          (1 << 0)
#define I2C1_SCL_TIMEOUT                       (1 << 1)
#define I2C1_ARB_LOST                          (1 << 3)
#define I2C1_RXFIFO_EMPTY                      (1 << 4)
#define I2C1_TXFIFO_FULL                       (1 << 5)
#define I2C1_INT_MODE_POSI                     (6		)
#define I2C1_INT_MODE_MASK                     (0x03<< 6)
#define I2C1_SMBUS_ACK                         (1 << 8)
#define I2C1_SMBUS_STOP                        (1 << 9)
#define I2C1_SMBUS_STA                         (1 << 10)
#define I2C1_ADDR_MATCH                        (1 << 11)
#define I2C1_ACK_REQ                           (1 << 12)
#define I2C1_TX_MODE                           (1 << 13)
#define I2C1_MASTER                            (1 << 14)
#define I2C1_BUSY                              (1 << 15)

#define REG_I2C1_DAT                         (I2C1_BASE_ADDR + 4 * 2)
#define I2C1_DAT_MASK                          (0xFF)

#define REG_I2C1_SMB_EXT_CN                    (I2C1_BASE_ADDR + 4 * 3)
#define I2C1_HOLD_TIME                         (1 << 16)
#define I2C1_BYTE_INTEV                        (1 << 8)
#define I2C1_ADDR_H_OUTEN                      (1 << 1)
#define I2C1_DATA_H_OUTEN                      (1 << 0)

#else

#define REG_I2C1_CONFIG                      (I2C1_BASE_ADDR + 4 * 0)
#define I2C1_ENSMB                             (1 << 0)
#define I2C1_STA                               (1 << 1)
#define I2C1_STO                               (1 << 2)
#define I2C1_ACK_TX                            (1 << 3)
#define I2C1_TX_MODE                           (1 << 4)
#define I2C1_FREQ_DIV_POSI                     (6)
#define I2C1_FREQ_DIV_MASK                     (0x3FF)
#define I2C1_SI                                (1 << 16)
#define I2C1_ACK_RX                            (1 << 17)
#define I2C1_ACK_REQ                           (1 << 18)
#define I2C1_BUSY                              (1 << 19)

#define REG_I2C1_DAT                         (I2C1_BASE_ADDR + 4 * 1)
#define I2C1_DAT_MASK                          (0xFF)

#endif

static UINT32 i2c1_open(UINT32 op_flag);
static UINT32 i2c1_close(void);
static UINT32 i2c1_read(char *user_buf, UINT32 count, UINT32 op_flag);
static UINT32 i2c1_write(char *user_buf, UINT32 count, UINT32 op_flag);
static UINT32 i2c1_ctrl(UINT32 cmd, void *param);

#endif  // _I2C1_H_

