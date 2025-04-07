#include "include.h"
#include "sdcard.h"
#include "sdcard_pub.h"
#include "sd_card.h"
#include "sdio_driver.h"

UINT32 sdcard_open(UINT32 op_flag)
{
	return bk_sd_card_init();
}

UINT32 sdcard_close(void)
{
	bk_sd_card_deinit();
	return SDCARD_SUCCESS;
}

SDIO_Error sdcard_read_multi_block(UINT8 *read_buffer, int first_block, int block_num)
{
	bk_sd_card_read_blocks(read_buffer, first_block, block_num);
	return SDCARD_SUCCESS;
}

SDIO_Error sdcard_write_multi_block(UINT8 *write_buff, UINT32 first_block, UINT32 block_num)
{
 	return bk_sd_card_write_blocks((const uint8_t *)write_buff, first_block, block_num);
}

void sdcard_get_card_info(SDCARD_S *card_info)
{
	card_info->block_size = 512;
	card_info->total_block = bk_sd_card_get_card_size();
}