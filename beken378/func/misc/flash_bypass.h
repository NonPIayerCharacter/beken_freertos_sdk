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

#ifndef __FLASH_BYPASS_H__
#define __FLASH_BYPASS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define CURRENT_ROUTINE_TEXT_SIZE            (0x400)

/*Write Enable for Volatile Status Register*/
#define FLASH_CMD_WR_EN_VSR                 (0x50)
/*Write Status Register*/
#define FLASH_CMD_WR_SR                     (0x01)

#define SPI_VAL_TAKE_CS                     (0x02)
#define SPI_VAL_RELEASE_CS                  (0x03)

#if CFG_FLASH_BYPASS_OTP
/* flash_status_register */
#define FLASH_STA_REG_WIP_BIT               (1 << 0)

#define OTP_WRITE_LEN_MAX                   (256)

typedef enum {
    CMD_FLASH_WR_EN = 0x06,

    CMD_FLASH_OTP_WRITE = 0x42,
    CMD_FLASH_OTP_EARSE = 0x44,
    CMD_FLASH_OTP_READ  = 0x48,

    CMD_FLASH_STA_REG_BIT07_BIT00_READ  = 0x05,
    CMD_FLASH_STA_REG_BIT15_BIT08_READ  = 0x35,
    CMD_FLASH_STA_REG_BIT23_BIT16_READ  = 0x15,
    CMD_FLASH_STA_REG_BIT07_BIT00_WRITE = 0x01,
    CMD_FLASH_STA_REG_BIT15_BIT08_WRITE = 0x31,
    CMD_FLASH_STA_REG_BIT23_BIT16_WRITE = 0x11,

    CMD_FLASH_STA_REG_BIT15_BIT00_WRITE = 0x01,

    CMD_FLASH_RDID = 0x9F,
} flash_bypass_cmd_t;

typedef enum {
    CMD_OTP_READ,
    CMD_OTP_EARSE,
    CMD_OTP_WRITE,
    CMD_OTP_SET_LOCK,
    CMD_OTP_GET_LOCK,
} flash_bypass_otp_cmd_t;

typedef struct {
    uint8_t  otp_idx;
    uint32_t addr_offset;
    uint32_t read_len;
    uint32_t write_len;
    uint8_t  *read_buf;
    uint8_t  *write_buf;
} flash_bypass_otp_ctrl_t;

typedef struct {
    uint32_t flash_id;
    uint8_t  otp_addr_idx_offset;
    uint8_t  otp_lock_regular_flag;
    uint32_t op_len_max;
    uint32_t wr_len_max;
    uint32_t idx_max;
    uint32_t idx_min;
} flash_bypass_otp_feature_t;
#endif

extern uint32_t flash_bypass_operate_sr_init(void);
extern int flash_bypass_op_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len);
extern int flash_bypass_op_write(uint8_t *op_code, uint8_t *tx_buf, uint32_t tx_len);

#ifdef __cplusplus
}
#endif
#endif //__FLASH_BYPASS_H__
// eof

