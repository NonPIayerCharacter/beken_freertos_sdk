#include "include.h"
#include "arm_arch.h"
#include "test_psram.h"
#include "qspi_pub.h"
#include "BkDriverQspi.h"

#if CFG_USE_QSPI
void qspi_psram_single_init(void)
{
	uint8_t line_mode;
	uint32_t is_single_mode = 0;
	qspi_dcache_drv_desc qspi_cfg;

	PSRAM_PRT("[psram]:qspi_psram_single_init\r\n");

	line_mode = MODE_PSRAM_SINGLE;
	PSRAM_PRT("line_mode = 0x%x\r\n", line_mode);

	qspi_cfg.mode = 0x00 | line_mode;		         // 0: 1 line mode    3: 4 line mode
	qspi_cfg.clk_set = 0x10;
	qspi_cfg.wr_command = line_mode ? 0x38 : 0x02;	 //write
	qspi_cfg.rd_command = line_mode ? 0xEB : 0x03;	 //read
	qspi_cfg.wr_dummy_size = 0;
	qspi_cfg.rd_dummy_size = line_mode ? 0x06 : 0x00;
	qspi_cfg.voltage_level = 2;

	if (MODE_PSRAM_QUAD == line_mode)
		is_single_mode = 1;

	bk_qspi_psram_quad_mode_switch(is_single_mode);
	bk_qspi_dcache_initialize(&qspi_cfg);
	bk_qspi_start();
}

void qspi_psram_quad_init(void)
{
	uint8_t line_mode;
	uint32_t is_single_mode = 0;
	qspi_dcache_drv_desc qspi_cfg;

	PSRAM_PRT("[psram]:qspi_psram_quad_init\r\n");

	line_mode = MODE_PSRAM_QUAD;
	PSRAM_PRT("line_mode = 0x%x\r\n", line_mode);

	qspi_cfg.mode = 0x00 | line_mode;		         // 0: 1 line mode    3: 4 line mode
	qspi_cfg.clk_set = 0x10;
	qspi_cfg.wr_command = line_mode ? 0x38 : 0x02;	 //write
	qspi_cfg.rd_command = line_mode ? 0xEB : 0x03;	 //read
	qspi_cfg.wr_dummy_size = 0;
	qspi_cfg.rd_dummy_size = line_mode ? 0x06 : 0x00;
	qspi_cfg.voltage_level = 2;

	if (MODE_PSRAM_QUAD == line_mode)
		is_single_mode = 1;

	bk_qspi_psram_quad_mode_switch(is_single_mode);
	bk_qspi_dcache_initialize(&qspi_cfg);
	bk_qspi_start();
}

uint32_t qspi_psram_8bit_increase_init_memory(uint8_t *buf, uint32_t count)
{
	int i;
	uint8_t *ptr = buf;

	ASSERT(buf);
	ASSERT(count);

	for (i = 0; i < count; i ++)
		ptr[i] = i & 0xff;

	return 0;
}

uint32_t qspi_psram_8bit_increase_compare(uint8_t *buf, uint32_t count)
{
	int i;
	uint8_t *ptr = buf;

	ASSERT(buf);
	ASSERT(count);

	for (i = 0; i < count; i ++) {
		if ((i & 0xff) != ptr[i]) {
			PSRAM_PRT("FAILED [%d] addr:0x%x, 0x%02x\r\n", i, &ptr[i], (i & 0xff));
			return -1;
		}
	}

	return 0;
}

uint32_t qspi_psram_8bit_init_fixed_value(uint8_t *buf, uint32_t count, uint8_t val)
{
	int i;
	uint8_t *ptr = buf;

	ASSERT(buf);
	ASSERT(count);

	for (i = 0; i < count; i ++)
		ptr[i] = val;

	return 0;
}

uint32_t qspi_psram_8bit_cmp_fixed_value(uint8_t *buf, uint32_t count, uint8_t val)
{
	int i;
	uint8_t *ptr = buf;

	ASSERT(buf);
	ASSERT(count);

	for (i = 0; i < count; i ++) {
		if (val != ptr[i]) {
			PSRAM_PRT("FAILED [%d] addr:0x%x, 0x%02x\r\n", i, &ptr[i], val);
			return -1;
		}
	}

	return 0;
}

uint32_t single_psram_demo(void)
{
	qspi_psram_single_init();

	qspi_psram_8bit_increase_init_memory((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN);
	qspi_psram_8bit_increase_compare((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN);

	PSRAM_PRT("[psram]:0xff\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xff);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xff);

	PSRAM_PRT("[psram]:0x55\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x55);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x55);

	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xaa);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xaa);

	PSRAM_PRT("[psram]:0x5a\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x5a);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x5a);

	PSRAM_PRT("[psram]:0xa5\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xa5);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xa5);

	return 0;
}

uint32_t quad_psram_demo(void)
{
	qspi_psram_quad_init();

	qspi_psram_8bit_increase_init_memory((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN);
	qspi_psram_8bit_increase_compare((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN);

	PSRAM_PRT("[psram]:0xff\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xff);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xff);

	PSRAM_PRT("[psram]:0x55\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x55);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x55);

	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xaa);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xaa);

	PSRAM_PRT("[psram]:0x5a\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x5a);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0x5a);

	PSRAM_PRT("[psram]:0xa5\r\n");
	qspi_psram_8bit_init_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xa5);
	qspi_psram_8bit_cmp_fixed_value((uint8_t *)PSRAM_TEST_START_ADDR, PSRAM_TEST_LEN, 0xa5);

	return 0;
}
#endif // CFG_USE_QSPI
// eof

