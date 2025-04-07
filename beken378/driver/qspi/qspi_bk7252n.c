#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "intc_pub.h"

#if CFG_USE_QSPI
#include "qspi.h"
#include "qspi_pub.h"
#include "sys_ctrl_pub.h"
#include "drv_model_pub.h"
#include "intc_pub.h"
#include "mcu_ps_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "uart_pub.h"
#include "mem_pub.h"

#define QSPI_VID_DEP                    0x80
#define QSPI_AUD_DEP                    0x10
#define QSPI_GE0_DEP                    0x10
#define QSPI_GE1_DEP                    0x10
#define QSPI_FLS_DEP                    0x40

static const SDD_OPERATIONS qspi_op =
{
	qspi_ctrl
};

void qspi_init(void)
{
	intc_service_register(FIQ_QSPI, PRI_IRQ_QSPI, qspi_isr);
	sddev_register_dev(QSPI_DEV_NAME, (SDD_OPERATIONS *)&qspi_op);
}

void qspi_exit(void)
{
	sddev_unregister_dev(QSPI_DEV_NAME);
}

static void qspi_gpio_configuration(UINT8 LineMode)
{
	uint32_t val;

	val = GFUNC_MODE_QSPI_CLK;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);//gpio24

	val = GFUNC_MODE_QSPI_CSN;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);//gpio26

	if (LineMode == 1) {
		val = GFUNC_MODE_QSPI_1LINE;
		sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);
		gpio_config(18, GMODE_INPUT_PULLDOWN);
		gpio_config(19, GMODE_INPUT_PULLDOWN);
	} else if(LineMode == 4) {
		//set io0/io1 gpio16/gpio17
		val = GFUNC_MODE_QSPI_1LINE;
		sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);

		//set io2/io3 gpio18/gpio19
		val = GFUNC_MODE_QSPI_4LINE;
		sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);
	}
}

static void qspi_icu_configuration(UINT32 enable)
{
	UINT32 param;

	if (enable) {
		param = PWD_QSPI_CLK_BIT;
		sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);

		param = (FIQ_QSPI_BIT);
		sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
	} else {
		param = (FIQ_QSPI_BIT);
		sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);

		param = PWD_QSPI_CLK_BIT;
		sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
	}
}

static void qspi_clk_set_dco(void)
{
	UINT32 param;
	param = QSPI_CLK_MUX_DCO;
	sddev_control(ICU_DEV_NAME, CMD_QSPI_CLK_SEL, &param);
}

static void qspi_clk_set_120M(void)
{
	UINT32 param;
	param = QSPI_CLK_MUX_120M;
	sddev_control(ICU_DEV_NAME, CMD_QSPI_CLK_SEL, &param);
}

static void qspi_div_clk_set(UINT32 clk)
{
	UINT32 val;

	val = REG_READ(REG_QSPI_SPI_CFG);
	val &= ~(QSPI_CLK_RATE_MASK << QSPI_CLK_RATE_POSI);
	val |= (clk & QSPI_CLK_RATE_MASK) << QSPI_CLK_RATE_POSI;
	REG_WRITE(REG_QSPI_SPI_CFG, val);

	val = ((clk & 0x3) << 11);
	sddev_control(ICU_DEV_NAME, CMD_QSPI_CLK_SEL, &val);
}

static void qspi_mem_sel_mode(UINT8 mode)
{
	UINT32 param;

	param = REG_READ(REG_QSPI_SPI_CFG);
	if (mode == QSPI_MEM_FOR_CPU) {
		param |= QSPI_IO_CPU_MEM_SEL;
	} else if (mode == QSPI_MEM_FOR_IO) {
		param &= ~(QSPI_IO_CPU_MEM_SEL);
	} else {
		return;
	}
	REG_WRITE(REG_QSPI_SPI_CFG, param);
}

static void qspi_clear_addr_cnt(void)
{
	UINT32 param;

	param = REG_READ(REG_QSPI_SPI_STATUS_CLR);
	param |= QSPI_CLR_ADDR_CNT;
	REG_WRITE(REG_QSPI_SPI_STATUS_CLR, param);

	param = REG_READ(REG_QSPI_SPI_STATUS_CLR);
	param &= ~(QSPI_CLR_ADDR_CNT);
	REG_WRITE(REG_QSPI_SPI_STATUS_CLR, param);
}

static bool qspi_is_cmd_start_done(void)
{
	return !!((REG_READ(REG_QSPI_SPI_STATUS) & QSPI_CMD_START_DONE)
	>> QSPI_CMD_START_DONE_POSI);
}

static void qspi_wait_cmd_done(void)
{
	for (int i = 0; i <= 10000; i++) {
		if (0 != qspi_is_cmd_start_done()) {
			break;
		}
		if (i == 10000) {
			os_printf("ERROR: qspi_ll_wait_cmd_done timeout!!!\r\n");
		}
	}
	REG_WRITE(REG_QSPI_SPI_STATUS_CLR,
	REG_READ(REG_QSPI_SPI_STATUS_CLR) | QSPI_CLR_CMD_START_DONE);
	REG_WRITE(REG_QSPI_SPI_STATUS_CLR,
	REG_READ(REG_QSPI_SPI_STATUS_CLR) & ~QSPI_CLR_CMD_START_DONE);
}

static UINT32 qspi_open(UINT32 op_flag)
{
	UINT32 param;

	qspi_icu_configuration(1);

	saradc_config_vddram_voltage(PSRAM_VDD_3_3V);

	param = REG_READ(REG_QSPI_SPI_CFG);
	param |= (QSPI_IO2_IO3_MODE | QSPI_SPI_EN);
	REG_WRITE(REG_QSPI_SPI_CFG, param);

	REG_WRITE(REG_QSPI_GLB_CTRL, QSPI_SOFT_RESET);

	return QSPI_SUCCESS;
}

static UINT32 qspi_close(void)
{
	qspi_icu_configuration(0);
	return QSPI_SUCCESS;
}

static int qspi_command_process(qspi_cmd_t *qspi_cmd)
{
	UINT32 param;

	if (qspi_cmd == NULL) {
		return QSPI_FAILURE;
	}

	if (qspi_cmd->work_mode == MEMORY_MAPPED_MODE) {
		if (qspi_cmd->op == QSPI_WRITE) { //write
			/*set cmd*/
			REG_WRITE(REG_QSPI_CMD_A_L, 0);
			param = (qspi_cmd->cmd & QSPI_CMD_1_MASK) << QSPI_CMD_1_POSI;
			REG_WRITE(REG_QSPI_CMD_A_H, param);

			/*set cmd line*/
			if (qspi_cmd->cmd == APS6404_CMD_QUAD_WRITE) {
				param = (QSPI_4_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_4_LINE << QSPI_CMD_2_LINE_POSI
					| QSPI_4_LINE << QSPI_CMD_3_LINE_POSI
					| QSPI_4_LINE << QSPI_CMD_4_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_5_LINE_POSI);
			} else if (qspi_cmd->cmd == APS6404_CMD_WRITE) {
				param = (QSPI_1_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_2_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_3_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_4_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_5_LINE_POSI);
			} else {
				return QSPI_FAILURE;
			}
			REG_WRITE(REG_QSPI_CMD_A_CFG_1, param);

			/*set cmd cfg*/
			param = ((qspi_cmd->wire_mode & QSPI_DATA_LINE_MASK) << QSPI_DATA_LINE_POSI
				| (qspi_cmd->dummy_cycle & QSPI_DUMMY_CLK_MASK) << QSPI_DUMMY_CLK_POSI);
			if (qspi_cmd->dummy_cycle) {
				param |= (QSPI_DUMMY_MODE_CMD4_CMD5 << QSPI_DUMMY_MODE_POSI);
			} else {
				param |= (QSPI_DUMMY_MODE_NO_INSERT << QSPI_DUMMY_MODE_POSI);
			}
			REG_WRITE(REG_QSPI_CMD_A_CFG_2, param);
		} else { //read
			/*set cmd*/
			REG_WRITE(REG_QSPI_CMD_B_L, 0);
			param = (qspi_cmd->cmd & QSPI_CMD_1_MASK) << QSPI_CMD_1_POSI;
			REG_WRITE(REG_QSPI_CMD_B_H, param);

			/*set cmd line*/
			if (qspi_cmd->cmd == APS6404_CMD_FAST_READ_QUAD) {
				if (qspi_cmd->wire_mode == QSPI_1WIRE) {
					param = (QSPI_1_LINE << QSPI_CMD_1_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_2_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_3_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_4_LINE_POSI
						| QSPI_CMD_DISABLE << QSPI_CMD_5_LINE_POSI);
				} else if (qspi_cmd->wire_mode == QSPI_4WIRE) {
					param = (QSPI_4_LINE << QSPI_CMD_1_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_2_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_3_LINE_POSI
						| QSPI_4_LINE << QSPI_CMD_4_LINE_POSI
						| QSPI_CMD_DISABLE << QSPI_CMD_5_LINE_POSI);
				} else {
					return QSPI_FAILURE;
				}
			} else {
				param = (QSPI_1_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_2_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_3_LINE_POSI
					| QSPI_1_LINE << QSPI_CMD_4_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_5_LINE_POSI);
			}
			REG_WRITE(REG_QSPI_CMD_B_CFG_1, param);

			/*set cmd cfg*/
			param = ((qspi_cmd->wire_mode & QSPI_DATA_LINE_MASK) << QSPI_DATA_LINE_POSI
				| (qspi_cmd->dummy_cycle & QSPI_DUMMY_CLK_MASK) << QSPI_DUMMY_CLK_POSI);
			if (qspi_cmd->dummy_cycle) {
				param |= (QSPI_DUMMY_MODE_CMD4_CMD5 << QSPI_DUMMY_MODE_POSI);
			} else {
				param |= (QSPI_DUMMY_MODE_NO_INSERT << QSPI_DUMMY_MODE_POSI);
			}
			REG_WRITE(REG_QSPI_CMD_B_CFG_2, param);
		}
	} else {
		if (qspi_cmd->op == QSPI_WRITE) { //write
			/*set cmd*/
			REG_WRITE(REG_QSPI_CMD_C_L, 0);
			param = (qspi_cmd->cmd & QSPI_CMD_1_MASK) << QSPI_CMD_1_POSI;
			REG_WRITE(REG_QSPI_CMD_C_H, param);

			/*set cmd line*/
			if (qspi_cmd->cmd == APS6404_CMD_ENTER_QUAD_MODE) {
				param = (QSPI_1_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_2_LINE_POSI);
			} else if (qspi_cmd->cmd == APS6404_CMD_EXIT_QUAD_MODE) {
				param = (QSPI_4_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_2_LINE_POSI);
			}
			REG_WRITE(REG_QSPI_CMD_C_CFG_1, param);

			/*set cmd cfg*/
			param = ((qspi_cmd->wire_mode & QSPI_DATA_LINE_MASK) << QSPI_DATA_LINE_POSI
				| (qspi_cmd->dummy_cycle & QSPI_DUMMY_CLK_MASK) << QSPI_DUMMY_CLK_POSI);
			if (qspi_cmd->dummy_cycle) {
				param |= (QSPI_DUMMY_MODE_CMD4_CMD5 << QSPI_DUMMY_MODE_POSI);
			} else {
				param |= (QSPI_DUMMY_MODE_NO_INSERT << QSPI_DUMMY_MODE_POSI);
			}
			param |= QSPI_CMD_START;
			REG_WRITE(REG_QSPI_CMD_C_CFG_2, param);

			/*wait cmd done*/
			qspi_wait_cmd_done();
		} else { //read
			/*set cmd*/
			REG_WRITE(REG_QSPI_CMD_D_L, 0);
			param = (qspi_cmd->cmd & QSPI_CMD_1_MASK) << QSPI_CMD_1_POSI;
			REG_WRITE(REG_QSPI_CMD_D_H, param);

			/*set cmd line*/
			param = (QSPI_1_LINE << QSPI_CMD_1_LINE_POSI
					| QSPI_CMD_DISABLE << QSPI_CMD_2_LINE_POSI);
			REG_WRITE(REG_QSPI_CMD_D_CFG_1, param);

			/*set cmd cfg*/
			param = ((qspi_cmd->wire_mode & QSPI_DATA_LINE_MASK) << QSPI_DATA_LINE_POSI
				| (qspi_cmd->dummy_cycle & QSPI_DUMMY_CLK_MASK) << QSPI_DUMMY_CLK_POSI);
			if (qspi_cmd->dummy_cycle) {
				param |= (QSPI_DUMMY_MODE_CMD4_CMD5 << QSPI_DUMMY_MODE_POSI);
			} else {
				param |= (QSPI_DUMMY_MODE_NO_INSERT << QSPI_DUMMY_MODE_POSI);
			}
			param |= QSPI_CMD_START;
			REG_WRITE(REG_QSPI_CMD_D_CFG_2, param);

			/*wait cmd done*/
			qspi_wait_cmd_done();
		}
	}

	return QSPI_SUCCESS;
}

static int qspi_data_process(qspi_data_t *qspi_data)
{
	int i;
	uint8_t param;

	if (qspi_data == NULL || qspi_data->buf == NULL || qspi_data->size == 0) {
		return QSPI_FAILURE;
	}

	qspi_mem_sel_mode(QSPI_MEM_FOR_CPU);
	qspi_clear_addr_cnt();

	if (qspi_data->op == QSPI_WRITE) {
		for (i = 0; i < qspi_data->size; i++) {
			param = *((uint8_t *)qspi_data->buf + i);
			*((uint8_t *)(QSPI_DATA_BASE_ADDR + qspi_data->addr + i)) = param;//*((uint8_t *)qspi_data->buf + i);
		}
	} else {
		for (i = 0; i < qspi_data->size; i++) {
			param = *((uint8_t *)(QSPI_DATA_BASE_ADDR + qspi_data->addr + i));
			*((uint8_t *)qspi_data->buf + i) = param;//*((uint8_t *)(QSPI_DATA_BASE_ADDR + qspi_data->addr + i));
		}
	}

	qspi_clear_addr_cnt();
	qspi_mem_sel_mode(QSPI_MEM_FOR_IO);

	return QSPI_SUCCESS;
}

static UINT32 qspi_ctrl(UINT32 cmd, void *param)
{
	UINT32 ret = QSPI_SUCCESS;

	peri_busy_count_add();

	switch (cmd) {
		case QSPI_CMD_GPIO_CONFIG:
			qspi_gpio_configuration(*(UINT8 *)param);
			break;

		case QSPI_CMD_DIV_CLK_SET:
			qspi_div_clk_set(*(UINT8 *)param);
			break;

		case QSPI_CMD_CLK_SET_DCO:
			qspi_clk_set_dco();
			break;

		case QSPI_CMD_CLK_SET_120M:
			qspi_clk_set_120M();
			break;

		case QSPI_DCACHE_CMD_OPEN:
			qspi_open(1);
			break;

		case QSPI_DCACHE_CMD_CLOSE:
			qspi_close();
			break;

		case QSPI_CMD_DCACHE_CONFIG:
			ret = qspi_command_process((qspi_cmd_t *)param);
			break;

		case QSPI_CMD_DATA_CONFIG:
			ret = qspi_data_process((qspi_data_t *)param);
			break;

		default:
			ret = QSPI_FAILURE;
			break;
	}

	peri_busy_count_dec();

	return ret;
}

void qspi_isr(void)
{
}

#endif
