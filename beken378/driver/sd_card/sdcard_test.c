#include "include.h"
#include "sdcard_config.h"

#if 0//CONFIG_SDCARD
#include "sdcard_pub.h"
#include "mem_pub.h"

#define SDCARD_TEST_BUFFER_SIZE (512 * 2)
static SDIO_Error sdcard_test_open(void)
{
	//bk_gpio_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO, SDCARD_LDO_CTRL_GPIO, GPIO_OUTPUT_STATE_HIGH);

	return bk_sd_card_init();
}

static void sdcard_test_close(void)
{
	//bk_gpio_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO, SDCARD_LDO_CTRL_GPIO, GPIO_OUTPUT_STATE_LOW);

	bk_sd_card_deinit();
}

UINT32 sdcard_intf_test(void)
{
	SDIO_Error ret = SD_OK;
	UINT32 err = SDCARD_SUCCESS;

	ret = sdcard_test_open();
	if (ret != SD_OK) {
		SDCARD_FATAL("Can't open sdcard, please check\r\n");
		goto err_out;
	}
	SDCARD_PRT("sdcard_open is ok \r\n");
	return err;

err_out:
	SDCARD_FATAL("sdcard_test err, ret:%d\r\n", ret);
	return SDCARD_FAILURE;

}

UINT32 test_sdcard_read(UINT32 blk, UINT32 blk_cnt)
{
	UINT32 ret=0, j;
	uint8_t *testbuf;

	os_printf("%s[+]\r\n", __func__);

	testbuf = os_malloc(SDCARD_TEST_BUFFER_SIZE);
	os_memset(testbuf, 0, SDCARD_TEST_BUFFER_SIZE);
	if (testbuf == NULL)
		return 1;

	for (j = 0; j < blk_cnt; j+=(SDCARD_TEST_BUFFER_SIZE/512))
	{
		ret = bk_sd_card_read_blocks(testbuf, blk + j, SDCARD_TEST_BUFFER_SIZE/512);

		for (int i = 0; i < SDCARD_TEST_BUFFER_SIZE; i+=16)
		{
			os_printf("0x%08x,0x%08x,0x%08x,0x%08x\r\n",
							   *(uint32_t *)&testbuf[i], 
							   *(uint32_t *)&testbuf[i+4],
							   *(uint32_t *)&testbuf[i+8],
							   *(uint32_t *)&testbuf[i+12]);
			if(i & 0x100)
				rtos_delay_milliseconds(4);
		}
	}

	os_free(testbuf);
	os_printf("%s[-]\r\n", __func__);
	return ret;
}

UINT32 test_sdcard_write(UINT32 blk, UINT32 blk_cnt, UINT32 wr_val)
{
	UINT32 ret=0, j = 0;
	uint8_t *testbuf;
	uint32 buf_size = blk_cnt * 512;	//SDCARD_TEST_BUFFER_SIZE
	testbuf = os_malloc(buf_size);
	os_memset(testbuf, 0, buf_size);
	if (testbuf == NULL)
		return 1;

	os_printf("%s[+]\r\n", __func__);
	for (int i = 0; i < buf_size; i+=16)
	{
		if(wr_val != 0x12345678)
		{
			*(uint32_t *)&testbuf[i] = wr_val; 
			*(uint32_t *)&testbuf[i+4] = wr_val;
			*(uint32_t *)&testbuf[i+8] = wr_val;
			*(uint32_t *)&testbuf[i+12] = wr_val;
		}
		else
		{
			*(uint32_t *)&testbuf[i] = j | (j << 8) | (j << 16) | (j << 24);
			j++;
			*(uint32_t *)&testbuf[i+4] = j | (j << 8) | (j << 16) | (j << 24);
			j++;
			*(uint32_t *)&testbuf[i+8] = j | (j << 8) | (j << 16) | (j << 24);
			j++;
			*(uint32_t *)&testbuf[i+12] = j | (j << 8) | (j << 16) | (j << 24);
			j++;
		}

		/* os_printf("0x%08x,0x%08x,0x%08x,0x%08x\r\n",
					   *(uint32_t *)&testbuf[i], 
					   *(uint32_t *)&testbuf[i+4],
					   *(uint32_t *)&testbuf[i+8],
					   *(uint32_t *)&testbuf[i+12]); */
	}

#if 1	//no issues.
	//for (j = 0; j < blk_cnt; j+=(buf_size/512))
	{
		ret = bk_sd_card_write_blocks(testbuf, blk, blk_cnt);
	}
#else
	for (j = 0; j < blk_cnt; j++)
	{
		ret = bk_sd_card_write_blocks(testbuf+j*512, blk+j, 1);
	}
#endif
	os_free(testbuf);
	os_printf("%s[-]\r\n", __func__);
	return ret;
}

UINT32 test_sdcard_auto(UINT32 blk, UINT32 blk_cnt, UINT32 wr_val, UINT32 test_cnt)
{
	//UINT32 j;
	UINT32 ret = test_sdcard_write(blk, blk_cnt, wr_val);

	uint8_t *testbuf;

	os_printf("%s[+]\r\n", __func__);

	testbuf = os_malloc(blk_cnt * 512);
	if (testbuf == NULL)
		return 1;
	os_memset(testbuf, 0, blk_cnt * 512);

	//for (j = 0; j < blk_cnt; j+=(SDCARD_TEST_BUFFER_SIZE/512))
	{
		ret = bk_sd_card_read_blocks(testbuf, blk, blk_cnt);

		for (int i = 0; i < blk_cnt*512; i+=16)
		{
			if((*(uint32_t *)&testbuf[i] != wr_val) ||
				(*(uint32_t *)&testbuf[i+4] != wr_val) ||
				(*(uint32_t *)&testbuf[i+8] != wr_val) ||
				(*(uint32_t *)&testbuf[i+12] != wr_val))
			os_printf("%s:Err:Fail:i=%d,0x%08x,0x%08x,0x%08x,0x%08x\r\n",
								__func__, 
								i,
							   *(uint32_t *)&testbuf[i], 
							   *(uint32_t *)&testbuf[i+4],
							   *(uint32_t *)&testbuf[i+8],
							   *(uint32_t *)&testbuf[i+12]);
		}
	}

	os_free(testbuf);
	
	return ret;
}

void sdcard_intf_close(void)
{
	sdcard_test_close();
}

#endif
