#ifndef _BK_SPI_FLASH_H_
#define _BK_SPI_FLASH_H_

extern int spi_flash_init(void);
extern void spi_flash_deinit(void);
extern UINT32 spi_flash_read_id(void);
extern int spi_flash_read(UINT32 addr, UINT32 size, UINT8 *dst);
extern int spi_flash_write(UINT32 addr, UINT32 size, UINT8 *src);
extern int spi_flash_erase(UINT32 addr, UINT32 size);
extern void spi_flash_protect(void);
extern void spi_flash_unprotect(void);
#endif //_BK_SPI_FLASH_H_