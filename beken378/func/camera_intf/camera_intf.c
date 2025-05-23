#include "include.h"
#include "arm_arch.h"

#if CFG_USE_CAMERA_INTF
//#include "process.h"
#include "llc.h"

#include "video_transfer.h"

#include "jpeg_encoder_pub.h"
#include "i2c_pub.h"

#include "camera_intf_config.h"
#include "camera_intf.h"
#include "camera_intf_pub.h"

#include "drv_model_pub.h"
#include "general_dma_pub.h"
#include "mem_pub.h"
#include "bk_timer_pub.h"
#include "gpio_pub.h"


#define CAMERA_RESET_GPIO_INDEX		GPIO16
#define CAMERA_RESET_HIGH_VAL       1
#define CAMERA_RESET_LOW_VAL        0
extern void delay100us(INT32 num);

#define EJPEG_DMA_CHNAL             GDMA_CHANNEL_5
#define EJPEG_DELAY_HTIMER_CHNAL    5
#define EJPEG_DELAY_HTIMER_VAL      (2)  // 2ms
#define USE_JTAG_FOR_DEBUG          0
#define I2C_WIRTE_TIMEOUT_COUNT     20
DJPEG_DESC_ST ejpeg_cfg;
DD_HANDLE i2c_hdl = DD_HANDLE_UNVALID, ejpeg_hdl = DD_HANDLE_UNVALID;
I2C_OP_ST i2c_operater;

void camera_intf_delay_timer_hdl(UINT8 param)
{
    #if CFG_GENERAL_DMA
    GDMA_CFG_ST en_cfg;
    UINT16 already_len = ejpeg_cfg.rx_read_len;
    UINT32 channel = ejpeg_cfg.dma_channel;
    GLOBAL_INT_DECLARATION();

    if (ejpeg_hdl == DD_HANDLE_UNVALID)
    {
        return;
    }

    //REG_WRITE((0x00802800+(16*4)), 0x02);

    int left_len = sddev_control(GDMA_DEV_NAME, CMD_GDMA_GET_LEFT_LEN, (void *)channel);
    int rec_len = ejpeg_cfg.node_len - left_len;
    UINT32 frame_len = 0;
    frame_len = ddev_control(ejpeg_hdl, EJPEG_CMD_GET_FRAME_LEN, NULL);

    if ((ejpeg_cfg.node_full_handler != NULL) && (rec_len > 0))
    {
        ejpeg_cfg.node_full_handler(ejpeg_cfg.rxbuf + already_len, rec_len, 1, frame_len);
    }

    already_len += rec_len;
    if (already_len >= ejpeg_cfg.rxbuf_len)
    {
        already_len -= ejpeg_cfg.rxbuf_len;
    }

    GLOBAL_INT_DISABLE();
    ejpeg_cfg.rx_read_len = already_len;
    GLOBAL_INT_RESTORE();

    // turn off dma, so dma can start from first configure. for easy handler
    en_cfg.channel = ejpeg_cfg.dma_channel;
    en_cfg.param = 0;
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);

    ejpeg_cfg.rx_read_len = 0;
    en_cfg.param = 1;
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
    #endif

    if ((ejpeg_cfg.data_end_handler))
    {
        ejpeg_cfg.data_end_handler();
    }

    channel = EJPEG_DELAY_HTIMER_CHNAL;
    sddev_control(TIMER_DEV_NAME, CMD_TIMER_UNIT_DISABLE, &channel);

    //REG_WRITE((0x00802800+(16*4)), 0x00);
}

static void camera_intf_start_delay_timer(void)
{
    timer_param_t param;

    if (ejpeg_hdl == DD_HANDLE_UNVALID)
    {
        return;
    }
    //REG_WRITE((0x00802800+(16*4)), 0x02);

    param.channel = EJPEG_DELAY_HTIMER_CHNAL;
    param.div = 1;
    param.period = EJPEG_DELAY_HTIMER_VAL;
    param.t_Int_Handler = camera_intf_delay_timer_hdl;

    sddev_control(TIMER_DEV_NAME, CMD_TIMER_INIT_PARAM, &param);

    //REG_WRITE((0x00802800+(16*4)), 0x00);
}

static void camera_intf_ejpeg_rx_handler(UINT32 dma)
{
    UINT16 already_len = ejpeg_cfg.rx_read_len;
    UINT16 copy_len = ejpeg_cfg.node_len;
    GLOBAL_INT_DECLARATION();

    if (ejpeg_hdl == DD_HANDLE_UNVALID)
    {
        return;
    }

    //REG_WRITE((0x00802800+(17*4)), 0x02);

    if (ejpeg_cfg.node_full_handler != NULL)
    {
        ejpeg_cfg.node_full_handler(ejpeg_cfg.rxbuf + already_len, copy_len, 0, 0);
    }

    already_len += copy_len;

    if (already_len >= ejpeg_cfg.rxbuf_len)
    {
        already_len = 0;
    }

    GLOBAL_INT_DISABLE();
    ejpeg_cfg.rx_read_len = already_len;
    GLOBAL_INT_RESTORE();

    //REG_WRITE((0x00802800+(17*4)), 0x00);
}

static void camera_intf_ejpeg_end_handler(void)
{
    camera_intf_start_delay_timer();
}

static void camera_intf_init_ejpeg_pixel(UINT32 ppi_type)
{
    switch (ppi_type)
    {
    case QVGA_320_240:
        ejpeg_cfg.x_pixel = X_PIXEL_320;
        ejpeg_cfg.y_pixel = Y_PIXEL_240;
        break;

    case VGA_640_480:
        ejpeg_cfg.x_pixel = X_PIXEL_640;
        ejpeg_cfg.y_pixel = Y_PIXEL_480;
        break;

    case VGA_800_600:
        ejpeg_cfg.x_pixel = X_PIXEL_800;
        ejpeg_cfg.y_pixel = Y_PIXEL_600;
        break;

    case VGA_1280_720:
        ejpeg_cfg.x_pixel = X_PIXEL_1280;
        ejpeg_cfg.y_pixel = Y_PIXEL_720;
        break;

    default:
        CAMERA_INTF_WPRT("cm PPI unknown, use QVGA\r\n");
        ejpeg_cfg.x_pixel = X_PIXEL_640;
        ejpeg_cfg.y_pixel = Y_PIXEL_480;
        break;
    }
}

static void camera_intf_config_ejpeg(void *data)
{
    os_memset(&ejpeg_cfg, 0, sizeof(DJPEG_DESC_ST));
    os_memcpy(&ejpeg_cfg, data, sizeof(TVIDEO_DESC_ST));

    camera_intf_init_ejpeg_pixel(CMPARAM_GET_PPI(ejpeg_cfg.sener_cfg));

    ejpeg_cfg.start_frame_handler = NULL;
    ejpeg_cfg.end_frame_handler = camera_intf_ejpeg_end_handler;

    #if CFG_GENERAL_DMA
    ejpeg_cfg.dma_rx_handler = camera_intf_ejpeg_rx_handler;
    ejpeg_cfg.dma_channel = GDMA_CHANNEL_4;
    #endif
}

#if (USE_CAMERA != HM_1055_DEV)
int camera_intf_sccb_write(UINT8 addr, UINT8 data)
{
    unsigned int status;
    unsigned int err_count = 0;
    int ret = 0;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;
    do
    {
        status = ddev_write(i2c_hdl, (char *)&data, 1, (UINT32)&i2c_operater);
        if (err_count++ > I2C_WIRTE_TIMEOUT_COUNT)
        {
            ret = -1;
            break;
        }
    }
    while (status != 0);

    return ret;
}

int camera_intf_sccb_read(UINT8 addr, UINT8 *data)
{
    unsigned int status;
    unsigned int err_count = 0;
    int ret = 0;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;
    do
    {
        status = ddev_read(i2c_hdl, (char *)data, 1, (UINT32)&i2c_operater);
        if (err_count++ > I2C_WIRTE_TIMEOUT_COUNT)
        {
            ret = -1;
            break;
        }
    }
    while (status != 0);

    return ret;
}
#else
int camera_intf_sccb_write(UINT16 addr, UINT8 data)
{
    unsigned int status;
    unsigned int err_count = 0;
    int ret = 0;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_16;
    do
    {
        status = ddev_write(i2c_hdl, (char *)&data, 1, (UINT32)&i2c_operater);
        if (err_count++ > I2C_WIRTE_TIMEOUT_COUNT)
        {
            ret = -1;
            break;
        }
    }
    while (status != 0);

    return ret;
}

int camera_intf_sccb_read(UINT16 addr, UINT8 *data)
{
    unsigned int status;
    unsigned int err_count = 0;
    int ret = 0;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_16;
    do
    {
        status = ddev_read(i2c_hdl, (char *)data, 1, (UINT32)&i2c_operater);
        if (err_count++ > I2C_WIRTE_TIMEOUT_COUNT)
        {
            ret = -1;
            break;
        }
    }
    while (status != 0);

    return ret;
}
#endif

#if (USE_CAMERA == GC0328C_DEV)
static int camera_inf_cfg_gc0328c_ppi(UINT32 ppi_type)
{
    UINT32 i, size;
    UINT8 addr, data;
    int ret = 0;

    switch (ppi_type)
    {
    case QVGA_320_240:
        size = sizeof(gc0328c_QVGA_320_240_talbe) / 2;
        for (i = 0; i < size; i++)
        {
            addr = gc0328c_QVGA_320_240_talbe[i][0];
            data = gc0328c_QVGA_320_240_talbe[i][1];
            if(camera_intf_sccb_write(addr, data))
            {
                ret = 1;
                goto cfg_gc0328c_ppi_exit;
            }
        }
        break;

    case VGA_640_480:
        size = sizeof(gc0328c_VGA_640_480_talbe) / 2;
        for (i = 0; i < size; i++)
        {
            addr = gc0328c_VGA_640_480_talbe[i][0];
            data = gc0328c_VGA_640_480_talbe[i][1];
            if(camera_intf_sccb_write(addr, data))
            {
                ret = 1;
                goto cfg_gc0328c_ppi_exit;
            }
        }
        break;

    default:
        CAMERA_INTF_WPRT("set PPI unknown\r\n");
        break;
    }

cfg_gc0328c_ppi_exit:
    return ret;
}

static int camera_inf_cfg_gc0328c_fps(UINT32 fps_type)
{
    UINT32 i, size;
    UINT8 addr, data;
    int ret = 0;

    switch (fps_type)
    {
    case TYPE_5FPS:
        size = sizeof(gc0328c_5pfs_talbe) / 2;
        for (i = 0; i < size; i++)
        {
            addr = gc0328c_5pfs_talbe[i][0];
            data = gc0328c_5pfs_talbe[i][1];
            if(camera_intf_sccb_write(addr, data))
            {
                ret = 1;
                goto cfg_gc0328c_fps_exit;
            }
        }
        break;

    case TYPE_10FPS:
        size = sizeof(gc0328c_10pfs_talbe) / 2;
        for (i = 0; i < size; i++)
        {
            addr = gc0328c_10pfs_talbe[i][0];
            data = gc0328c_10pfs_talbe[i][1];
            if(camera_intf_sccb_write(addr, data))
            {
                ret = 1;
                goto cfg_gc0328c_fps_exit;
            }
        }
        break;

    case TYPE_20FPS:
        size = sizeof(gc0328c_20pfs_talbe) / 2;
        for (i = 0; i < size; i++)
        {
            addr = gc0328c_20pfs_talbe[i][0];
            data = gc0328c_20pfs_talbe[i][1];
            if(camera_intf_sccb_write(addr, data))
            {
                ret = 1;
                goto cfg_gc0328c_fps_exit;
            }
        }
        break;

    default:
        CAMERA_INTF_WPRT("set FPS unknown\r\n");
        break;
    }

cfg_gc0328c_fps_exit:
    return ret;
}
#endif

#if (USE_CAMERA == GC_2145_DEV)
static void camera_inf_cfg_ppi(UINT32 ppi_type)
{
    switch (ppi_type)
    {
    case QVGA_320_240:
        gc2145_init(0);
        break;

    case VGA_640_480:
        gc2145_init(1);
        break;

    default:
        CAMERA_INTF_WPRT("set PPI unknown %d\r\n", ppi_type);
        break;
    }
}

static void camera_inf_cfg_fps(UINT32 fps_type)
{
    switch (fps_type)
    {
    case TYPE_10FPS:
        gc2145_fps(12);
        break;

    case TYPE_15FPS:
        gc2145_fps(15);
        break;

    case TYPE_20FPS:
        gc2145_fps(20);
        break;

    default:
        CAMERA_INTF_WPRT("set FPS unknown %d\r\n", fps_type);
        break;
    }
}
#endif

static int camera_intf_config_senser(void)
{
    UINT32 i, size;
    UINT8 addr, data;
    int ret = 0;

    #if (USE_CAMERA == PAS6329_DEV)

    i2c_operater.salve_id = PAS6329_DEV_ID;

    size = sizeof(pas6329_page0) / 2;
    PAS6329_SET_PAGE0;

    for (i = 0; i < size; i++)
    {
        addr = pas6329_page0[i][0];
        data = pas6329_page0[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }

    size = sizeof(pas6329_page1) / 2;
    PAS6329_SET_PAGE1;
    for (i = 0; i < size; i++)
    {
        addr = pas6329_page1[i][0];
        data = pas6329_page1[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }

    size = sizeof(pas6329_page2) / 2;
    PAS6329_SET_PAGE2;
    for (i = 0; i < size; i++)
    {
        addr = pas6329_page2[i][0];
        data = pas6329_page2[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }

    PAS6329_SET_PAGE0;
    CAMERA_INTF_WPRT("PAS6329 init finish\r\n");

    #elif (USE_CAMERA == OV_7670_DEV)

    i2c_operater.salve_id = OV_7670_DEV_ID;

    size = sizeof(ov_7670_init_talbe) / 2;

    for (i = 0; i < size; i++)
    {
        addr = ov_7670_init_talbe[i][0];
        data = ov_7670_init_talbe[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }
    CAMERA_INTF_WPRT("OV_7670 init finish\r\n");

    #elif (USE_CAMERA == PAS6375_DEV)

    i2c_operater.salve_id = PAS6375_DEV_ID;

    size = sizeof(pas6375_init_talbe) / 2;

    for (i = 0; i < size; i++)
    {
        addr = pas6375_init_talbe[i][0];
        data = pas6375_init_talbe[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }
    CAMERA_INTF_WPRT("PAS6375 init finish\r\n");

    #elif (USE_CAMERA == GC0328C_DEV)
    i2c_operater.salve_id = GC0328C_DEV_ID;

    size = sizeof(gc0328c_init_talbe) / 2;

    for (i = 0; i < size; i++)
    {
        addr = gc0328c_init_talbe[i][0];
        data = gc0328c_init_talbe[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }

    camera_inf_cfg_gc0328c_ppi(CMPARAM_GET_PPI(ejpeg_cfg.sener_cfg));
	camera_inf_cfg_gc0328c_fps(TYPE_5FPS);

    CAMERA_INTF_WPRT("GC0328C init finish\r\n");
    #elif (USE_CAMERA == BF_2013_DEV)
    i2c_operater.salve_id = BF_2013_DEV_ID;

    size = sizeof(bf_2013_init_talbe) / 2;

    for (i = 0; i < size; i++)
    {
        addr = bf_2013_init_talbe[i][0];
        data = bf_2013_init_talbe[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }
    CAMERA_INTF_WPRT("BF_2013 init finish\r\n");

    #elif (USE_CAMERA == GC0308C_DEV)
    i2c_operater.salve_id = GC0308C_DEV_ID;

    size = sizeof(gc0308c_init_talbe) / 2;

    for (i = 0; i < size; i++)
    {
        addr = gc0308c_init_talbe[i][0];
        data = gc0308c_init_talbe[i][1];
        if(camera_intf_sccb_write(addr, data))
        {
            ret = 1;
            goto config_exit;
        }
    }
    CAMERA_INTF_WPRT("GC0308C init finish\r\n");
    #elif (USE_CAMERA == HM_1055_DEV)
    i2c_operater.salve_id = HM_1055_DEV_ID;

    size = sizeof(hm_1055_init_talbe) / 4;

    for (i = 0; i < size; i++)
    {
        UINT16 addr1;
        addr1 = hm_1055_init_talbe[i][0];
        data = hm_1055_init_talbe[i][1];
        addr = addr;
        if(camera_intf_sccb_write(addr1, data))
        {
            ret = 1;
            goto config_exit;
        }
    }
    CAMERA_INTF_WPRT("HM_1055 init finish\r\n");
    #elif (USE_CAMERA == GC_2145_DEV)
    i2c_operater.salve_id = GC_2145_DEV_ID;

    if (gc2145_probe() == 0)
    {
        gc2145_reset();

        camera_inf_cfg_ppi(CMPARAM_GET_PPI(ejpeg_cfg.sener_cfg));
        camera_inf_cfg_fps(CMPARAM_GET_FPS(ejpeg_cfg.sener_cfg));

        CAMERA_INTF_WPRT("GC2145 init finish\r\n");
    }
    else
    {
        CAMERA_INTF_WPRT("GC2145 init failed\r\n");
    }

    size = i = 0;
    addr = data = 0;
    i = i;
    size = size;
    addr = addr;
    data = data;
    #endif

config_exit:
    return ret;
}


void init_camera_resetpin(void)
{
    //bk_gpio_config_output(CAMERA_RESET_GPIO_INDEX);
    //bk_gpio_output(CAMERA_RESET_GPIO_INDEX, CAMERA_RESET_HIGH_VAL);
}

void camera_reset(void)
{
    //bk_gpio_output(CAMERA_RESET_GPIO_INDEX, CAMERA_RESET_HIGH_VAL);
    //delay100us(10);                                                   // 1ms
    //bk_gpio_output(CAMERA_RESET_GPIO_INDEX, CAMERA_RESET_LOW_VAL);
    //delay100us(10);                                                   // 1=1ms,
    //bk_gpio_output(CAMERA_RESET_GPIO_INDEX, CAMERA_RESET_HIGH_VAL);
    //delay100us(10);                                                   // 1ms
    //CAMERA_INTF_WPRT("Camera Reset\r\n");
}

void camera_flip(UINT8 n)
{
    UINT8 addr, data, dt0, dt1;

    if (i2c_operater.salve_id == GC0328C_DEV_ID)
    {
        addr = 0x17;
        dt0 = 0x14;
        dt1 = 0x17;
    }
    else
    {
        addr = 0x17;
        dt0 = 0x14;
        dt1 = 0x17;
    }

    if (n)
    {
        data = dt1;     //flip 180
    }
    else
    {
        data = dt0;     //normal
    }

    camera_intf_sccb_write(addr, data);
}

/*---------------------------------------------------------------------------*/
int camera_intfer_init(void *data)
{
    UINT32 status;
    int fail = 0;
    GLOBAL_INT_DECLARATION();

    camera_intf_config_ejpeg(data);

    ejpeg_hdl = ddev_open(EJPEG_DEV_NAME, &status, (UINT32)&ejpeg_cfg);
    if(ejpeg_hdl == DD_HANDLE_UNVALID)
    {
        return 0;
    }

    //camera_reset();
    #if USE_JTAG_FOR_DEBUG
    //set i2c2 mode master/slave
    UINT32 i2c2_trans_mode = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)// master
                              & (~I2C2_MSG_WORK_MODE_AL_BIT))// 7bit address
                             | (I2C2_MSG_WORK_MODE_IA_BIT); // with inner address
    i2c_hdl = ddev_open(I2C2_DEV_NAME, &status, i2c2_trans_mode);
    bk_printf("open I2C2\r\n");
    {
        extern void uart_hw_uninit(UINT8 uport);
        // disable uart temporarily
        uart_hw_uninit(1);
    }

    #else
    UINT32 oflag = 0;
    i2c_hdl = ddev_open(I2C1_DEV_NAME, &status, oflag);
    bk_printf("open I2C1\r\n");
    #endif
    if(i2c_hdl == DD_HANDLE_UNVALID)
    {
        fail = -1;
        bk_printf("open I2C failed \r\n");
        goto init_exit;
    }

    if(camera_intf_config_senser() != 0)
    {
        fail = -2;
        bk_printf("config_senser failed \r\n");
        goto init_exit;
    }

    CAMERA_INTF_FATAL("camera_intfer_init,%p-%p\r\n", ejpeg_hdl, i2c_hdl);
    return 1;

init_exit:
    if(ejpeg_hdl != DD_HANDLE_UNVALID)
        ddev_close(ejpeg_hdl);
    if(i2c_hdl != DD_HANDLE_UNVALID)
        ddev_close(i2c_hdl);

    GLOBAL_INT_DISABLE();
    ejpeg_hdl = i2c_hdl = DD_HANDLE_UNVALID;
    GLOBAL_INT_RESTORE();

    os_memset(&ejpeg_cfg, 0, sizeof(DJPEG_DESC_ST));
    return fail;
}

void camera_intfer_deinit(void)
{
    GLOBAL_INT_DECLARATION();
    if((ejpeg_hdl == DD_HANDLE_UNVALID) && (i2c_hdl == DD_HANDLE_UNVALID))
        return;

    CAMERA_INTF_FATAL("camera_intfer_deinit,%p-%p\r\n", ejpeg_hdl, i2c_hdl);

    ddev_close(ejpeg_hdl);
    ddev_close(i2c_hdl);

    GLOBAL_INT_DISABLE();
    ejpeg_hdl = i2c_hdl = DD_HANDLE_UNVALID;
    GLOBAL_INT_RESTORE();

    os_memset(&ejpeg_cfg, 0, sizeof(DJPEG_DESC_ST));
}

UINT32 camera_intfer_set_video_param(UINT32 ppi_type, UINT32 pfs_type)
{
    if (ejpeg_hdl == DD_HANDLE_UNVALID)
    {
        return 1;
    }

    if (ppi_type < PPI_MAX)
    {
        UINT32 param;
        camera_intf_init_ejpeg_pixel(ppi_type);

        param = ejpeg_cfg.x_pixel;
        ddev_control(ejpeg_hdl, EJPEG_CMD_SET_X_PIXEL, &param);

        param = ejpeg_cfg.y_pixel;
        ddev_control(ejpeg_hdl, EJPEG_CMD_SET_Y_PIXEL, &param);

        #if (USE_CAMERA == GC0328C_DEV)
        camera_inf_cfg_gc0328c_ppi(ppi_type);
        #elif (USE_CAMERA == GC0328C_DEV)
        camera_inf_cfg_ppi(ppi_type);
        #endif
    }

    if (pfs_type < FPS_MAX)
    {
        #if (USE_CAMERA == GC0328C_DEV)
        camera_inf_cfg_gc0328c_fps(pfs_type);
        #elif (USE_CAMERA == GC0328C_DEV)
        camera_inf_cfg_fps(pfs_type);
        #endif
    }

    return 0;

}

UINT32 camera_intfer_get_senser_reg(UINT16 addr, UINT8 *data)
{
    int ret = 0;
    if (data)
    {
        #if (USE_CAMERA != HM_1055_DEV)
        UINT8 addr_a = (UINT8)(addr & 0x00ff);
        ret = camera_intf_sccb_read(addr_a, data);
        #else
        ret = camera_intf_sccb_read(addr, data);
        #endif

        return ret;
    }

    return 0;
}

UINT32 camera_intfer_set_senser_reg(UINT16 addr, UINT8 data)
{
    int ret = 0;
    #if (USE_CAMERA != HM_1055_DEV)
    UINT8 addr_a = (UINT8)(addr & 0x00ff);
    ret = camera_intf_sccb_write(addr_a, data);
    #else
    ret = camera_intf_sccb_write(addr, data);
    #endif

    return ret;
}

/*---------------------------------------------------------------------------*/

#endif // CFG_USE_CAMERA_INTF
