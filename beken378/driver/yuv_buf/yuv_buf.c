#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "yuv_buf_pub.h"
#include "yuv_buf.h"
#include "target_util_pub.h"
#include "uart_pub.h"
#include "mem_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"

#if (CFG_SOC_NAME == SOC_BK7252N)

//#define YUV_BUF_DEBUG
#ifdef YUV_BUF_DEBUG
#define yuv_isr_debug_prt                 os_printf
#define yuv_isr_info_prt                 null_prf
#else
#define yuv_isr_debug_prt                 null_prf
#define yuv_isr_info_prt                 null_prf
#endif

static void yuv_buf_isr(void);
static UINT32 yuv_buf_ctrl(UINT32 cmd, void *param);

yuv_buf_config_t *yuv_config = NULL;

__maybe_unused static SDD_OPERATIONS yuv_buf_op = {
    yuv_buf_ctrl
};

static void yuv_buf_enable_interrupt(void)
{
    UINT32 param;
    param = (FIQ_YUV_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
}

static void yuv_buf_disable_interrupt(void)
{
    UINT32 param;
    param = (FIQ_YUV_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

static void yuv_buf_set_reset(UINT32 set)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_CLK_CTL_ADDR);
    if(set)
        reg |= YUV_BUF_SOFT_RESET;
    else
        reg &= ~YUV_BUF_SOFT_RESET;

    //reg |= YUV_BUF_CLK_GATE;
    REG_WRITE(YUV_BUF_CLK_CTL_ADDR, reg);
}

static void yuv_buf_soft_reset()
{
    yuv_buf_set_reset(0);
    yuv_buf_set_reset(1);
}

static void yuv_buf_reset_config_to_default(void)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_CLK_CTL_ADDR);
    reg |= YUV_BUF_SOFT_RESET;
    REG_WRITE(YUV_BUF_CLK_CTL_ADDR, reg);
}

static void yuv_buf_init_common(void)
{
    /* 1) power on yuv_buf
    * 2) enable yuv_buf system interrupt
    */
    yuv_buf_enable_interrupt();
    yuv_buf_set_reset(1);
}

static void yuv_buf_deinit_common(void)
{
    yuv_buf_reset_config_to_default();
    /* 1) power off yuv_buf
    * 2) disable yuv_buf system interrupt
    */
    yuv_buf_disable_interrupt();
}

__maybe_unused static void yuv_buf_driver_init(void)
{
    intc_service_register(FIQ_YUV, PRI_FIQ_YUV, yuv_buf_isr);
}

__maybe_unused static void yuv_buf_driver_deinit(void)
{
    intc_service_unregister(FIQ_YUV);
    yuv_buf_deinit_common();
}

static void yuv_buf_set_mclk_div(UINT8 mclk_div)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    reg &= ~(YUV_BUF_MCLK_DIV_MASK << YUV_BUF_MCLK_DIV_POSI);
    reg |= ((mclk_div & YUV_BUF_MCLK_DIV_MASK) << YUV_BUF_MCLK_DIV_POSI);
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_x_pixel(UINT32 x_pixel)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X5_ADDR);
    reg &= ~(YUV_BUF_X_PIXEL_MASK << YUV_BUF_X_PIXEL_POSI);
    reg |= ((x_pixel & YUV_BUF_X_PIXEL_MASK) << YUV_BUF_X_PIXEL_POSI);
    REG_WRITE(YUV_BUF_REG0X5_ADDR, reg);
}

static void yuv_buf_set_y_pixel(UINT32 y_pixel)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X5_ADDR);
    reg &= ~(YUV_BUF_Y_PIXEL_MASK << YUV_BUF_Y_PIXEL_POSI);
    reg |= ((y_pixel & YUV_BUF_Y_PIXEL_MASK) << YUV_BUF_Y_PIXEL_POSI);
    REG_WRITE(YUV_BUF_REG0X5_ADDR, reg);
}

__maybe_unused static void yuv_buf_set_frame_blk(UINT32 frame_blk)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X5_ADDR);
    reg &= ~(YUV_BUF_FRAME_BLK_MASK << YUV_BUF_FRAME_BLK_POSI);
    reg |= ((frame_blk & YUV_BUF_FRAME_BLK_MASK) << YUV_BUF_FRAME_BLK_POSI);
    REG_WRITE(YUV_BUF_REG0X5_ADDR, reg);
}

static void yuv_buf_set_pixel_partial(UINT32 x_l, UINT32 x_r, UINT32 y_l, UINT32 y_r)
{
    UINT32 reg = 0;
    reg = (x_l) + (x_r << 16);
    REG_WRITE(YUV_BUF_REG0X6_ADDR, reg);

    reg = (y_l) + (y_r << 16);
    REG_WRITE(YUV_BUF_REG0X7_ADDR, reg);

    // disable partial encode
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    reg &= ~YUV_BUF_PARTIAL_DISPLAY_ENA;
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_yuv_format(UINT8 yuv_format)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    reg &= ~(YUV_BUF_YUV_FMT_SEL_MASK << YUV_BUF_YUV_FMT_SEL_POSI);
    reg |= ((yuv_format & YUV_BUF_YUV_FMT_SEL_MASK) << YUV_BUF_YUV_FMT_SEL_POSI);
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_x_pixel_resize(UINT32 x_pixel_resize)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0XD_ADDR);
    reg &= ~(YUV_BUF_X_PIXEL_RESIZE_MASK << YUV_BUF_X_PIXEL_RESIZE_POSI);
    reg |= ((x_pixel_resize & YUV_BUF_X_PIXEL_RESIZE_MASK) << YUV_BUF_X_PIXEL_RESIZE_POSI);
    REG_WRITE(YUV_BUF_REG0XD_ADDR, reg);
}

static void yuv_buf_set_y_pixel_resize(UINT32 y_pixel_resize)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0XD_ADDR);
    reg &= ~(YUV_BUF_Y_PIXEL_RESIZE_MASK << YUV_BUF_Y_PIXEL_RESIZE_POSI);
    reg |= ((y_pixel_resize & YUV_BUF_Y_PIXEL_RESIZE_MASK) << YUV_BUF_Y_PIXEL_RESIZE_POSI);
    REG_WRITE(YUV_BUF_REG0XD_ADDR, reg);
}

__maybe_unused static void yuv_buf_enable_sync_edge_dect(UINT8 enable)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    if (enable)
        reg |= YUV_BUF_SUNC_EDGE_DECT_ENA;
    else
        reg &= ~YUV_BUF_SUNC_EDGE_DECT_ENA;
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_encode_begin_hsync_posedge(UINT8 enable)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    if (enable)
        reg |= YUV_BUF_SOI_HSYNC;
    else
        reg &= ~YUV_BUF_SOI_HSYNC;
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_int(UINT32 en, UINT32 bits)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X9_ADDR);
    reg &= ~(bits & YUV_INT_MASK);
    if (en)
        reg |= (bits & YUV_INT_MASK);
    REG_WRITE(YUV_BUF_REG0X9_ADDR, reg);
}

static void yuv_buf_enable_int(void)
{
    UINT32 bits = YUV_BUF_YSYNC_NEGE_INT_ENA;

    yuv_buf_set_int(1, bits);
}

static void yuv_buf_set_config_common(UINT32 x_pixel, UINT32 y_pixel, UINT8 mclk_div, UINT32 yuv_format)
{
    yuv_buf_set_mclk_div(mclk_div);
    yuv_buf_set_x_pixel(x_pixel / 8);
    yuv_buf_set_y_pixel(y_pixel / 8);
    //yuv_buf_set_frame_blk((x_pixel * y_pixel) / 128);
    yuv_buf_set_pixel_partial(1, x_pixel, 1, y_pixel);
    yuv_buf_set_yuv_format(yuv_format);

    yuv_buf_set_x_pixel_resize(x_pixel / 8);
    yuv_buf_set_y_pixel_resize(y_pixel / 8);

    //yuv_buf_enable_sync_edge_dect(1);
    yuv_buf_set_encode_begin_hsync_posedge(1);
    yuv_buf_enable_int();
}

static void yuv_buf_set_em_base_addr(UINT32 em_base_addr)
{
    REG_WRITE(YUV_BUF_EM_BASE_ADDR, em_base_addr);
}

static void yuv_buf_set_emr_base_addr(UINT32 emr_base_addr)
{
    REG_WRITE(YUV_BUF_EMR_BASE_ADDR, emr_base_addr);
}

static void yuv_buf_set_pingpong_mode(UINT32 mode)
{
    UINT32 reg = REG_READ(YUV_BUF_REG0XD_ADDR);
    reg &= ~(YUV_BUF_BPS_PINGPONG);
    reg |= (mode && 0x1);
    REG_WRITE(YUV_BUF_REG0XD_ADDR, reg);
}

static void yuv_buf_set_jpeg_mode_config(yuv_buf_config_t *config)
{
    yuv_buf_set_config_common(config->x_pixel, config->y_pixel, config->mclk_div, config->yuv_format);

    yuv_buf_set_emr_base_addr((UINT32)config->base_addr);
    yuv_buf_set_em_base_addr((UINT32)config->base_addr);

    //enable/disable pingpong
    yuv_buf_set_pingpong_mode(0);
}

static void yuv_buf_mode_init(yuv_buf_config_t *config)
{
    yuv_config = (yuv_buf_config_t*)os_malloc(sizeof(yuv_buf_config_t));
    if (yuv_config)
    {
        os_memset(yuv_config, 0 ,sizeof(yuv_buf_config_t));
        os_memcpy(yuv_config, config, sizeof(yuv_buf_config_t));
    }
    else
    {
        os_printf("yuv config malloc failed \r\n");
        return;
    }

    yuv_buf_init_common();

    yuv_buf_set_jpeg_mode_config(config);
}

static void yuv_buf_mode_deinit(void)
{
    yuv_buf_set_reset(0);
}

static void yuv_buf_soft_mode_config()
{
    UINT32 reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    reg |= 0x6000;
    reg |= (1 << 18);
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);

    reg = REG_READ(YUV_BUF_REG0X9_ADDR);
    reg &= ~(1 << 8);
    REG_WRITE(YUV_BUF_REG0X9_ADDR, reg);
}

static void yuv_buf_send_renc_start()
{
    REG_WRITE(YUV_BUF_RENC_START_ADDR, 0x1);
}

static void yuv_buf_em_base_addr_set(UINT32 addr)
{
    yuv_buf_set_emr_base_addr((UINT32)addr);
    yuv_buf_set_em_base_addr((UINT32)addr);
}

static void yuv_buf_em_base_addr_get(void *addr)
{
    *(UINT32 *)addr = REG_READ(YUV_BUF_EMR_BASE_ADDR);
}


__maybe_unused static void yuv_buf_deinit(void)
{
    yuv_buf_deinit_common();
}

static void yuv_buf_set_yuv_buf_mode(UINT8 enable)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    if (enable)
        reg |= YUV_BUF_YUV_MODE;
    else
        reg &= ~YUV_BUF_YUV_MODE;
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

static void yuv_buf_set_h264_encode_mode(UINT8 enable)
{
    UINT32 reg;
    reg = REG_READ(YUV_BUF_REG0X4_ADDR);
    if (enable)
        reg |= YUV_BUF_H264_MODE;
    else
        reg &= ~YUV_BUF_H264_MODE;
    REG_WRITE(YUV_BUF_REG0X4_ADDR, reg);
}

__maybe_unused static void yuv_buf_start_yuv_mode(void)
{
    yuv_buf_set_yuv_buf_mode(1);
    yuv_buf_set_h264_encode_mode(0);
}

__maybe_unused static void yuv_buf_start_h264_mode(void)
{
    yuv_buf_set_yuv_buf_mode(0);
    yuv_buf_set_h264_encode_mode(1);
}

__maybe_unused static void yuv_buf_start(yuv_mode_t mode)
{
    switch (mode) {
    case YUV_MODE:
        yuv_buf_start_yuv_mode();
        break;
    case H264_MODE:
        yuv_buf_start_h264_mode();
        break;
    default:
        break;
    }
}

__maybe_unused static void yuv_buf_stop_yuv_mode(void)
{
    yuv_buf_set_yuv_buf_mode(0);
}

__maybe_unused static void yuv_buf_stop_h264_mode(void)
{
    yuv_buf_set_h264_encode_mode(0);
}

__maybe_unused static void bk_yuv_buf_stop(yuv_mode_t mode)
{
    switch (mode) {
    case YUV_MODE:
        yuv_buf_stop_yuv_mode();
        break;
    case H264_MODE:
        yuv_buf_stop_h264_mode();
        break;
    default:
        break;
    }
}

void yuv_buf_init(void)
{
    intc_service_register(FIQ_YUV, PRI_FIQ_YUV, yuv_buf_isr);
    sddev_register_dev(YUV_BUF_DEV_NAME, &yuv_buf_op);
}

// void yuv_buf_start(void)
// {
// }

void yuv_buf_exit(void)
{
    sddev_unregister_dev(YUV_BUF_DEV_NAME);
}

UINT32 yuv_buf_ctrl(UINT32 cmd, void *param)
{
    switch (cmd) {
        case YUV_BUF_CTRL_INIT:
            yuv_buf_mode_init(param);
            break;
        case YUV_BUF_SOFT_MODE:
            yuv_buf_soft_mode_config();
            break;
        case YUV_BUF_RENC_START:
            yuv_buf_send_renc_start();
            break;
        case YUV_BUF_EMADDR_SET:
            yuv_buf_em_base_addr_set(*(UINT32 *)param);
            break;
        case YUV_BUF_EMADDR_GET:
            yuv_buf_em_base_addr_get(param);
            break;
        case YUV_BUF_CTRL_DEINIT:
            yuv_buf_mode_deinit();
            break;
        case YUV_BUF_CMD_RESET:
            yuv_buf_soft_reset();
            break;
        default:
            break;
    }

    return 0;
}

void yuv_buf_isr(void)
{
    UINT32 yuv_buf_int_status;
    yuv_buf_int_status = REG_READ(YUV_BUF_REG0XA_ADDR);
    REG_WRITE(YUV_BUF_REG0XA_ADDR, yuv_buf_int_status);
    if (yuv_buf_int_status & YUV_BUF_YSYNC_NEGE_INT)
    {
        yuv_isr_info_prt("vsync \r\n");
        if (yuv_config->vsync_handler)
        {
            yuv_config->vsync_handler();
        }
    }
    if (yuv_buf_int_status & YUV_BUF_YUV_ARV_INT)
    {
        yuv_isr_info_prt("arr \r\n");
    }
    if (yuv_buf_int_status & YUV_BUF_SM0_WR_INT)
    {
        yuv_isr_info_prt("sm0 \r\n");
    }
    if (yuv_buf_int_status & YUV_BUF_SM1_WR_INT)
    {
        yuv_isr_info_prt("sm1 \r\n");
    }
    if (yuv_buf_int_status & YUV_BUF_SEN_FULL_INT)
    {
        yuv_isr_debug_prt("full \r\n");
        if (yuv_config->sensor_full_handler)
        {
            yuv_config->sensor_full_handler();
        }
    }
    if (yuv_buf_int_status & YUV_BUF_ENC_LINE_INT)
    {
        yuv_isr_info_prt("enc_line \r\n");
    }
    if (yuv_buf_int_status & YUV_BUF_SEN_RESL_INT)
    {
        yuv_isr_debug_prt("resol \r\n");
        if (yuv_config->resol_err_handler)
        {
            yuv_config->resol_err_handler();
        }
    }
    if (yuv_buf_int_status & YUV_BUF_H264_ERR_INT)
    {
        yuv_isr_debug_prt("h264 \r\n");
    }
    if (yuv_buf_int_status & YUV_BUF_ENC_SLOW_INT)
    {
        yuv_isr_debug_prt("slow \r\n");
        if (yuv_config->enc_slow_handler)
        {
            yuv_config->enc_slow_handler();
        }
    }
}

#endif

// eof
