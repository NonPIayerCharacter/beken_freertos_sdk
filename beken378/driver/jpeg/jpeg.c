#include "include.h"
#include "arm_arch.h"
#include "str_pub.h"

#if (CFG_SOC_NAME == SOC_BK7252N)
#include "jpeg_encoder_pub.h"
#include "jpeg.h"

#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "yuv_buf_pub.h"

#include "drv_model_pub.h"
#include "mem_pub.h"
#include "general_dma_pub.h"
#include "video_transfer.h"

#ifndef BIT
#define BIT(i) (1<<(i))
#endif

#define JPEG_BITRATE_MAX_SIZE_320_240           (20 * 1024)
#define JPEG_BITRATE_MIN_SIZE_320_240           (5 * 1024)

#define JPEG_BITRATE_MAX_SIZE_640_480           (35 * 1024)
#define JPEG_BITRATE_MIN_SIZE_640_480           (5 * 1024)

#define JPEG_BITRATE_MAX_SIZE_1280_720          (50 * 1024)
#define JPEG_BITRATE_MIN_SIZE_1280_720          (30 * 1024)

#define JPEG_BITRATE_MAX_SIZE           JPEG_BITRATE_MAX_SIZE_640_480
#define JPEG_BITRATE_MIN_SIZE           JPEG_BITRATE_MIN_SIZE_640_480

static UINT32 jpeg_ctrl(UINT32 cmd, void *param);
static UINT32 jpeg_open(UINT32 op_flag);
static UINT32 jpeg_close(void);
static void jpeg_isr(void);

const UINT32 jpeg_quant_table[JPEG_QUANT_TAB_LEN] =
{
    0x120f0e14, 0x12140d0f, 0x15171210, 0x321e1814,
    0x1c1c1e21, 0x2e2c3d1e, 0x40493224, 0x40474b4c,
    0x5a504546, 0x55506273, 0x4645566d, 0x6d658864,
    0x82817b77, 0x8d604e81, 0x967d8c97, 0x7c817e73,
    0x1e171715, 0x213b1e1a, 0x537c3b21, 0x7c7c5346,
    0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c,
    0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c,
    0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c, 0x7c7c7c7c,

};
DJPEG_DESC_PTR jpeg_config = NULL;
volatile uint8_t err_flag = false;

static const DD_OPERATIONS ejpeg_op =
{
	jpeg_open,
	jpeg_close,
	NULL,
	NULL,
	jpeg_ctrl
};

static void jpeg_soft_reset_enable(UINT8 enable)
{
	UINT32 reg_addr = JPEG_REG0X2_CLK_CTRL_ADDR;
	UINT32 reg_val = REG_READ(reg_addr);

	if (enable)
	{
		reg_val |= JPEG_SOFT_RESET;
	}
	else
	{
		reg_val &= ~JPEG_SOFT_RESET;
	}
	REG_WRITE(reg_addr, reg_val);
}

static void jpeg_gpio_config(void)
{
	UINT32 param;
	param = GFUNC_MODE_PCLK;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
	param = GFUNC_MODE_HSYNC_VSYNC;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
	param = GFUNC_MODE_DVP_DATA;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
	param = GFUNC_MODE_MLCK;
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
}

static void jpeg_power_up(void)
{
	UINT32 param;
	param = PWD_JEPG_CLK_BIT;
	sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);
}

static void jpeg_power_down(void)
{
	UINT32 param;
	param = PWD_JEPG_CLK_BIT;
	sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
}

static void jpeg_set_clock(void)
{
	// jpeg/mclk set
	UINT32 jpeg_clk = 0x1;   //120M
	sddev_control(ICU_DEV_NAME, CMD_JPEG_CLK_SEL, &jpeg_clk);
	UINT32 jpeg_mclk = 0x3;  //24M
	sddev_control(ICU_DEV_NAME, CMD_JPEG_MCLK_SEL, &jpeg_mclk);
}

static void jpeg_enable_interrupt(void)
{
	UINT32 param;
	param = (FIQ_JPEG_BIT);
	sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
}

static void jpeg_disable_interrupt(void)
{
	UINT32 param;
	param = (FIQ_JPEG_BIT);
	sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

static UINT32 jpeg_get_interrupt_status(void)
{
	return (REG_READ(JPEG_REG0X6_ADDR) & JPEG_INT_STATUS_MASK);
}

static void jpeg_clear_interrupt_status(UINT32 int_sta)
{
	REG_WRITE(JPEG_REG0X6_ADDR, int_sta);
}

static void jpeg_reset_config_to_default(void)
{
	/* 1.reset REG_0x0
	* 2.clear int_status(REG_0x6)
	*/
	REG_WRITE(JPEG_REG0XC_ADDR, 0);
	REG_WRITE(JPEG_REG0X6_ADDR, REG_READ(JPEG_REG0X6_ADDR));
}

static void jpeg_int_config(void)
{
	UINT32 reg = REG_READ(JPEG_REG0XC_ADDR);

	reg &= ~(JPEG_EOF_INT_EN | JPEG_SOF_INT_EN);

	if (jpeg_config->start_frame_handler)
	{
		reg |= JPEG_SOF_INT_EN;
	}
	if (jpeg_config->end_frame_handler)
	{
		reg |= JPEG_EOF_INT_EN;
	}

	reg |= (JPEG_ERR_INT_EN | JPEG_HEAD_INT_EN | JPEG_LCLEAR_DIR);
	REG_WRITE(JPEG_REG0XC_ADDR, reg);
}

static void jpeg_set_pixel(UINT32 x, UINT32 y)
{
	UINT32 reg = 0;
	reg = REG_READ(JPEG_REG0XD_ADDR);
	reg &= ~(JPEG_X_PIXEL_MASK << JPEG_X_PIXEL_POSI);
	reg |= (x << JPEG_X_PIXEL_POSI);
	REG_WRITE(JPEG_REG0XD_ADDR, reg);

	reg &= ~(JPEG_Y_PIXEL_MASK << JPEG_Y_PIXEL_POSI);
	reg |= (y << JPEG_Y_PIXEL_POSI);
	REG_WRITE(JPEG_REG0XD_ADDR, reg);
}

static void jpeg_set_pixel_partial(UINT32 x_l, UINT32 x_r, UINT32 y_l, UINT32 y_r)
{
	UINT32 reg = 0;

	reg = REG_READ(JPEG_REG0X8_ADDR);
	reg |= ((x_l) + (x_r << 12));
	REG_WRITE(JPEG_REG0X8_ADDR, reg);

	reg = REG_READ(JPEG_REG0X9_ADDR);
	reg |= ((y_l) + (y_r << 12));
	REG_WRITE(JPEG_REG0X9_ADDR, reg);

	// bit reg8 24 to 0
	reg = REG_READ(JPEG_REG0X8_ADDR);
	reg &= ~(0x1 << 24);
	REG_WRITE(JPEG_REG0X8_ADDR, reg);
}

static void ejpeg_init_quant_table(void)
{
	UINT32 i;
	UINT32 reg_addr;

	for (i = 0; i < JPEG_QUANT_TAB_LEN; i++)
	{
		reg_addr = JPEG_REG20_QUANT_TAB + i * 4;
		REG_WRITE(reg_addr, (UINT32)jpeg_quant_table[i]);
	}
}

static void jpeg_enc_config(void)
{
	// disable clk gate
	UINT32 reg;
	reg = REG_READ(JPEG_REG0X2_CLK_CTRL_ADDR);
	reg |= 0x2;
	REG_WRITE(JPEG_REG0X2_CLK_CTRL_ADDR, reg);
	// eof offset
	reg = REG_READ(JPEG_REG0X4_ADDR);
	reg = (20 << 16);
	REG_WRITE(JPEG_REG0X4_ADDR, reg);

	//rev data
	reg = REG_READ(JPEG_REG0XD_ADDR);
	reg |= 0x2;
	REG_WRITE(JPEG_REG0XD_ADDR, reg);

	//mclk div
	reg = REG_READ(JPEG_REG0XC_ADDR);
	reg |= (0x2 << 4);
	REG_WRITE(JPEG_REG0XC_ADDR, reg);
}

static void jpeg_enc_enable(uint32_t enable)
{
	UINT32 reg;
	reg = REG_READ(JPEG_REG0XD_ADDR);

	if (enable)
	{
		reg |= JPEG_JPEG_ENC_EN;
		REG_WRITE(JPEG_REG0XD_ADDR, reg);
	}
	else
	{
		reg &= ~(JPEG_JPEG_ENC_EN);
		REG_WRITE(JPEG_REG0XD_ADDR, reg);
	}
}

__maybe_unused static void jpeg_fifo_addr_get(void *fifo_addr)
{
	*(UINT32 *)fifo_addr = JPEG_REG0X5_RXFIFO_DATA_ADDR;
}

static UINT32 jpeg_frame_size_get(void)
{
	return REG_READ(JPEG_REG0X7_BYTE_CNT_PFRM_ADDR) + 1; // why need +1?
}

static void jpeg_software_init(void)
{
	ddev_register_dev(EJPEG_DEV_NAME, (DD_OPERATIONS *)&ejpeg_op);
}

static void jpeg_hardware_init(void)
{
	intc_service_register(FIQ_JPEG, PRI_FIQ_JPEG, jpeg_isr);
	jpeg_reset_config_to_default();
}

static void jpeg_enable_enc_size(UINT32 enable)
{
	UINT32 reg_addr = JPEG_REG0XD_ADDR;
	UINT32 reg_val = REG_READ(reg_addr);

	if (enable)
	{
		reg_val |= JPEG_JPEG_ENC_SIZE;
	}
	else
	{
		reg_val &= ~JPEG_JPEG_ENC_SIZE;
	}
	REG_WRITE(reg_addr, reg_val);
}

__maybe_unused static void jpeg_set_video_byte_reverse(UINT32 reverse)
{
	UINT32 reg_addr = JPEG_REG0XD_ADDR;
	UINT32 reg_val = REG_READ(reg_addr);

	if (reverse)
	{
		reg_val |= JPEG_JPEG_WORD_REVERSE;
	}
	else
	{
		reg_val &= ~JPEG_JPEG_WORD_REVERSE;
	}
	REG_WRITE(reg_addr, reg_val);
}

static void jpeg_set_bitrate_step(UINT32 step)
{
	UINT32 reg_addr = JPEG_REG0XD_ADDR;
	UINT32 reg_val = REG_READ(reg_addr);

	reg_val = (reg_val & ~(JPEG_BITRATE_STEP_MASK << JPEG_BITRATE_STEP_POSI))
			| ((step & JPEG_BITRATE_STEP_MASK) << JPEG_BITRATE_STEP_POSI);
	REG_WRITE(reg_addr, reg_val);
}

__maybe_unused static void jpeg_enable_bitrate_ctrl(UINT32 enable)
{
	UINT32 reg_addr = JPEG_REG0XD_ADDR;
	UINT32 reg_val = REG_READ(reg_addr);

	if (enable)
	{
		reg_val |= JPEG_BITRATE_CTRL;
	}
	else
	{
		reg_val &= ~JPEG_BITRATE_CTRL;
	}
	REG_WRITE(reg_addr, reg_val);
}

void jpeg_set_target_high_byte(UINT32 high)
{
	REG_WRITE(JPEG_REG0XE_TARGET_BYTE_H_ADDR, high & TARGET_BYTE_H_MASK);
}

void jpeg_set_target_low_byte(UINT32 low)
{
	REG_WRITE(JPEG_REG0XF_TARGET_BYTE_L_ADDR, low & TARGET_BYTE_L_MASK);
}

#if CFG_GENERAL_DMA
static void jpeg_config_rxdma(void)
{
	GDMACFG_TPYES_ST cfg;
	GDMA_CFG_ST en_cfg;
	os_memset(&cfg, 0, sizeof(GDMACFG_TPYES_ST));

	cfg.dstdat_width = 32;
	cfg.srcdat_width = 32;
	cfg.dstptr_incr = 1;
	cfg.srcptr_incr = 0;
	cfg.src_start_addr = (void *)JPEG_REG0X5_RXFIFO_DATA_ADDR;
	cfg.dst_start_addr = &(jpeg_config->rxbuf[0]);

	cfg.channel = jpeg_config->dma_channel;
	cfg.prio = 0;
	cfg.u.type5.dst_loop_start_addr = &(jpeg_config->rxbuf[0]);
	cfg.u.type5.dst_loop_end_addr = &(jpeg_config->rxbuf[jpeg_config->rxbuf_len]);

	cfg.fin_handler = jpeg_config->dma_rx_handler;

	cfg.src_module = GDMA_X_SRC_JPEG_TX_REQ;  // why not GDMA_X_SRC_JPEG_RX_REQ
	cfg.dst_module = GDMA_X_DST_DTCM_WR_REQ;

	sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE5, &cfg);

	en_cfg.channel = jpeg_config->dma_channel;
	en_cfg.param = jpeg_config->node_len; // dma translen
	sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, &en_cfg);

	en_cfg.channel = jpeg_config->dma_channel;
	en_cfg.param = 1;
	sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
}

static void jpeg_eixt_rxdma(void)
{
	GDMA_CFG_ST en_cfg;

	en_cfg.channel = jpeg_config->dma_channel;
	en_cfg.param = 0;
	sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
}
#endif

static void dvp_camera_reset_hardware_modules_handler()
{
	sddev_control(YUV_BUF_DEV_NAME, YUV_BUF_CMD_RESET, NULL);

	bk_dma_stop(jpeg_config->dma_channel);
	jpeg_enc_enable(0);
	jpeg_enc_enable(1);

	bk_dma_start(jpeg_config->dma_channel);
}

static void yuv_buf_vsync_handler()
{
	if (err_flag)
	{
		EJPEG_PRT("jpeg reset OK!");
		err_flag = false;
		dvp_camera_reset_hardware_modules_handler();
	}
}

static void err_flag_set()
{
	if (!err_flag)
	{
		EJPEG_PRT("encode err");
		err_flag = true;
	}
}

static UINT32 jpeg_yuv_init(void)
{
	yuv_buf_config_t config;
	UINT32 base_size;
	UINT8 *base_addr = NULL;
	
	config.x_pixel = jpeg_config->x_pixel * 8; 
	config.y_pixel = jpeg_config->y_pixel * 8;

	base_size = config.x_pixel * YUV_ENCODE_UINT_LINES * YUV_PIXEL_SIZE;
	base_addr = (UINT8*)os_malloc(base_size * sizeof(UINT8));

	if(base_addr == NULL)
	{
		EJPEG_WPRT("jpeg_yuv_init no buf\r\n");
		return 0;
	}
	jpeg_config->yuv_base = base_addr;

	// yuv init
	config.base_addr = (uint8_t *)base_addr;
	config.yuv_format = YUV_FORMAT_YUYV;
	config.work_mode = JPEG_MODE;
	config.mclk_div = YUV_MCLK_DIV_2;
	config.vsync_handler = yuv_buf_vsync_handler;
	config.enc_slow_handler = err_flag_set;
	config.sensor_full_handler = err_flag_set;
	config.resol_err_handler = err_flag_set;

	sddev_control(YUV_BUF_DEV_NAME, YUV_BUF_CTRL_INIT, &config);

	return 1;
}

static void jpeg_yuv_deinit(void)
{
	sddev_control(YUV_BUF_DEV_NAME, YUV_BUF_CTRL_DEINIT, NULL);
}

static UINT32 jpeg_open(UINT32 op_flag)
{
	if (!op_flag)
	{
		EJPEG_WPRT("jpeg_open is NULL\r\n");
		return EJPEG_FAILURE;
	}

	jpeg_config = ((DJPEG_DESC_PTR)op_flag);

	if(jpeg_yuv_init() == 0)
	{
		return EJPEG_FAILURE;
	}

	jpeg_enc_enable(0);
	jpeg_power_up();
	jpeg_soft_reset_enable(1);
	jpeg_reset_config_to_default();
	jpeg_gpio_config();

	jpeg_enc_config();
	ejpeg_init_quant_table();

	jpeg_set_pixel(jpeg_config->x_pixel, jpeg_config->y_pixel);
	jpeg_set_pixel_partial(1, jpeg_config->x_pixel, 1, jpeg_config->y_pixel);
	jpeg_int_config();

	// this 4 byte size attched to the end of JPEG, use to check crc
	jpeg_enable_enc_size(1);
	jpeg_set_video_byte_reverse(1);  // no matter how, jpeg works

	jpeg_set_target_high_byte(JPEG_BITRATE_MAX_SIZE);
	jpeg_set_target_low_byte(JPEG_BITRATE_MIN_SIZE);
	jpeg_set_bitrate_step(3);
	jpeg_enable_bitrate_ctrl(1);

	jpeg_enable_interrupt();

	#if CFG_GENERAL_DMA
	jpeg_config_rxdma();
	#endif

	//camera_power_on();
	jpeg_enc_enable(1);
	EJPEG_PRT("jpeg opened\r\n");

	return EJPEG_SUCCESS;
}

void camera_power_on(void)
{
	UINT32 reg;

	jpeg_enc_enable(0);
	jpeg_power_up();
	jpeg_set_clock();
	jpeg_soft_reset_enable(1);
	jpeg_reset_config_to_default();

	//mclk div
	reg = REG_READ(JPEG_REG0XC_ADDR);
	reg |= (0x2 << 4);
	REG_WRITE(JPEG_REG0XC_ADDR, reg);

	jpeg_gpio_config();
}

static UINT32 jpeg_close(void)
{
	jpeg_yuv_deinit();

	jpeg_enc_enable(0);

	#if CFG_GENERAL_DMA
	jpeg_eixt_rxdma();
	#endif

	jpeg_soft_reset_enable(0);
	jpeg_disable_interrupt();
	jpeg_power_down();

	if(jpeg_config->yuv_base)
	{
		os_free(jpeg_config->yuv_base);
		jpeg_config->yuv_base = NULL;
	}

	EJPEG_PRT("jpeg closed\r\n");

	return EJPEG_SUCCESS;
}

void ejpeg_init(void)
{
	jpeg_software_init();
	jpeg_hardware_init();
}

void ejpeg_exit(void)
{
	jpeg_reset_config_to_default();
	ddev_unregister_dev(EJPEG_DEV_NAME);
}

static UINT32 jpeg_ctrl(UINT32 cmd, void *param)
{
	UINT32 ret = EJPEG_SUCCESS;

	switch (cmd)
	{
		case EJPEG_CMD_GET_FRAME_LEN:
			ret = jpeg_frame_size_get();
			break;
		default:
			EJPEG_PRT("unknown jpeg ctrl cmd %d\r\n", cmd);
			break;
	}

	return ret;
}

void jpeg_off(void)
{
	jpeg_close();
}

static void jpeg_isr(void)
{
	UINT32 int_status = jpeg_get_interrupt_status();
	jpeg_clear_interrupt_status(int_status);
	
	if (int_status & JPEG_INT_ERR_BIT)
	{
		EJPEG_WPRT("jpg err \r\n");
		err_flag = true;
	}
	if (int_status & JPEG_INT_HEAD_BIT)
	{
	}
	if (int_status & JPEG_INT_SOF_BIT)
	{
		if (jpeg_config->start_frame_handler)
		{
			jpeg_config->start_frame_handler();
		}
	}
	if (int_status & JPEG_INT_EOF_BIT)
	{
		if (jpeg_config->end_frame_handler)
		{
			jpeg_config->end_frame_handler();
		}
	}
	if (int_status & JPEG_INT_LINEC_BIT)
	{
		if (jpeg_config->line_clear_handler)
		{
			jpeg_config->line_clear_handler();
		}
	}
}


#endif
// eof

