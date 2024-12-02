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

extern uint32_t flash_bypass_operate_sr_init(void);
extern int flash_bypass_op_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len);
extern int flash_bypass_op_write(uint8_t *op_code, uint8_t *tx_buf, uint32_t tx_len);

#ifdef __cplusplus
}
#endif
#endif //__FLASH_BYPASS_H__
// eof

