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
#include "rtos_pub.h"
#include "icu_pub.h"

#include "sys_ctrl_pub.h"

#include "sensors/bf2013.h"
#include "sensors/gc0308c.h"
#include "sensors/gc0311.h"
#include "sensors/gc0328c.h"
#include "sensors/hi704.h"
#include "sensors/gc0329c.h"
#include "sensors/gc0310.h"
//#include "sensors/hm1055.h"
//#include "sensors/ov7670.h"
//#include "sensors/pas6329.h"
//#include "sensors/pas6375.h"

#include "../../../../../src/driver/drv_local.h"
static softI2C_t g_softI2C;
extern void delay100us(INT32 num);

#ifdef CAMERA_BITRATE_LOG_PRT
typedef struct camera_debug_log_t
{
    UINT32 frame_cnt;
    UINT32 byte_size;
} camera_debug_log_t;

beken_timer_t camera_debug_timer;
camera_debug_log_t camera_debug_log = {0};
camera_debug_log_t camera_debug_log_cached = {0};
#endif

DJPEG_DESC_ST ejpeg_cfg;
extern TVIDEO_DESC_ST tvideo_st;
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
    #ifdef CAMERA_BITRATE_LOG_PRT
    camera_debug_log.byte_size += (frame_len / 1024);
    camera_debug_log.frame_cnt ++;
    #endif

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

#ifdef CAMERA_BITRATE_LOG_PRT
static void camera_log_print()
{
    os_printf("jpeg fps:[%d] bitrate:[%d KB] \r\n",
              (camera_debug_log.frame_cnt - camera_debug_log_cached.frame_cnt) / (DEBUG_TIMRT_INTERVAL / 1000),
              (camera_debug_log.byte_size - camera_debug_log_cached.byte_size) / (DEBUG_TIMRT_INTERVAL / 1000));
    camera_debug_log_cached.frame_cnt = camera_debug_log.frame_cnt;
    camera_debug_log_cached.byte_size = camera_debug_log.byte_size;
}
#endif

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

void camera_intf_sccb_write(uint8_t addr, uint8_t data)
{
    unsigned int status;
    unsigned int err_count = 0;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;
    do
    {
        status = ddev_write(i2c_hdl, (char*)&data, 1, (uint32_t)&i2c_operater);
        if(err_count++ > I2C_WIRTE_TIMEOUT_COUNT)
        {
            break;
        }
    } while(status != 0);

    //Soft_I2C_Start(&g_softI2C, addr);
    //Soft_I2C_WriteByte(&g_softI2C, data);
    //Soft_I2C_Stop(&g_softI2C);
}

void camera_intf_sccb_read(uint8_t addr, uint8_t* data)
{
    unsigned int status;
    i2c_operater.op_addr = addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;
    do
    {
        status = ddev_read(i2c_hdl, (char*)data, 1, (uint32_t)&i2c_operater);
    } while(status != 0);
    //Soft_I2C_Start(&g_softI2C, addr | 1);
    //*data = Soft_I2C_ReadByte(&g_softI2C, 1);
    //Soft_I2C_Stop(&g_softI2C);
}

uint8_t camera_intf_sccb_write2(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len)
{

    uint8_t status = 0;
    i2c_operater.salve_id = device_addr;
    i2c_operater.op_addr = register_addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;

    status = ddev_write(i2c_hdl, (char*)data, (UINT32)len, (UINT32)&i2c_operater);

    if(status != 0)
    {
        os_printf("Unable to write I2C.\r\n");
    }

    //Soft_I2C_Start(&g_softI2C, (device_addr << 1));
    //Soft_I2C_WriteByte(&g_softI2C, register_addr);
    //for(int i = 0; i < len; i++)
    //{
    //    Soft_I2C_WriteByte(&g_softI2C, data[i]);
    //}
    //Soft_I2C_Stop(&g_softI2C);

    return status;

}

uint8_t camera_intf_sccb_read2(uint8_t device_addr, uint8_t register_addr, uint8_t* data, uint8_t len)
{
    uint8_t status = 0;
    i2c_operater.salve_id = device_addr;
    i2c_operater.op_addr = register_addr;
    i2c_operater.addr_width = ADDR_WIDTH_8;

    status = ddev_read(i2c_hdl, (char*)data, (UINT32)len, (UINT32)&i2c_operater);
    if(status != 0)
    {
        os_printf("Unable to read I2C.\r\n");
    }
    //Soft_I2C_Start(&g_softI2C, (device_addr << 1));
    //Soft_I2C_WriteByte(&g_softI2C, register_addr);
    //Soft_I2C_Stop(&g_softI2C);
    //Soft_I2C_Start(&g_softI2C, (device_addr << 1) | 1);
    //*data = Soft_I2C_ReadByte(&g_softI2C, 1);
    //Soft_I2C_Stop(&g_softI2C);

    return status;

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


/*---------------------------------------------------------------------------*/
void camera_intfer_init(void* ejpeg_config, camera_sensor_t* sensor)
{
    uint32_t status;

    camera_intf_config_ejpeg(ejpeg_config);

    ejpeg_hdl = ddev_open(EJPEG_DEV_NAME, &status, (uint32_t)&ejpeg_cfg);

    camera_reset();

    if(sensor->i2c_bus == (char*)&I2C2_DEV_NAME)
    {
    
        uint32_t i2c2_trans_mode = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)// master
            & (~I2C2_MSG_WORK_MODE_AL_BIT))// 7bit address
            | (I2C2_MSG_WORK_MODE_IA_BIT); // with inner address
    
        i2c_hdl = ddev_open(I2C2_DEV_NAME, &status, i2c2_trans_mode);
    
        bk_printf("open I2C2\r\n");
    
    }
    else
    {
        uint32_t i2c2_trans_mode = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)// master
            & (~I2C2_MSG_WORK_MODE_AL_BIT))// 7bit address
            | (I2C2_MSG_WORK_MODE_IA_BIT); // with inner address
        i2c_hdl = ddev_open(I2C1_DEV_NAME, &status, i2c2_trans_mode);
    
        bk_printf("open I2C1\r\n");
    
    }


    /*{
        extern void uart_hw_uninit(uint8_t uport);
        // disable uart temporarily
        uart_hw_uninit(1);
    }*/


    /*

    In case I2C1 is used, to be tested!!

    uint32_t oflag = 0;
    i2c_hdl = ddev_open(I2C1_DEV_NAME, &status, oflag);
    bk_printf("open I2C1\r\n");
    #endif*/

    //camera_intf_config_senser();

    sensor->i2c_cfg = &i2c_operater;
    sensor->ejpeg_cfg = &ejpeg_cfg;

    sensor->init(i2c_hdl, ejpeg_hdl, sensor);

    os_printf("camera_intfer_init,%p-%p\r\n", ejpeg_hdl, i2c_hdl);
}

void camera_intfer_deinit(camera_sensor_t* sensor)
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

    #ifdef CAMERA_BITRATE_LOG_PRT
    rtos_stop_timer(&camera_debug_timer);
    rtos_deinit_timer(&camera_debug_timer);
    #endif

    os_memset(&ejpeg_cfg, 0, sizeof(DJPEG_DESC_ST));
}

camera_sensor_t* camera_detect()
{
    UINT32 status;

    camera_intf_config_ejpeg(&tvideo_st);

    ejpeg_hdl = ddev_open(EJPEG_DEV_NAME, &status, (UINT32)&ejpeg_cfg);

    //camera_reset();

    UINT32 i2c1_trans_mode = (0 & (~I2C1_MSG_WORK_MODE_MS_BIT)// master
        & (~I2C1_MSG_WORK_MODE_AL_BIT))// 7bit address
        | (I2C1_MSG_WORK_MODE_IA_BIT); // with inner address

    i2c_hdl = ddev_open(I2C1_DEV_NAME, &status, i2c1_trans_mode);

    os_printf("Searching for camera sensors, using I2C1 bus...\r\n");

    camera_sensor_t* sensor = malloc(sizeof(camera_sensor_t));

    sensor->i2c_bus = I2C2_DEV_NAME;
    //g_softI2C.pin_clk = 20;
    //g_softI2C.pin_data = 21;
    //Soft_I2C_PreInit(&g_softI2C);
    //usleep(100);
    
    
    if(gc0328c_sensor_detect())
    {
    
        sensor->name = ("GC0328C");
        sensor->init = gc0328c_sensor_init;
    
    }
    else if(gc0310_sensor_detect())
    {
    
        sensor->name = ("GC0310/GC0312");
        sensor->init = gc0311_sensor_init;
    
    }
    else if(gc0311_sensor_detect())
    {
    
        sensor->name = ("GC0311");
        sensor->init = gc0311_sensor_init;
    
    }
    else if(hi704_sensor_detect())
    {
    
        sensor->name = ("HI704");
        sensor->init = hi704_sensor_init;
    
    }
    else if(gc0329c_sensor_detect())
    {
    
        sensor->name = ("GC0329C");
        sensor->init = gc0329c_sensor_init;
    
    }
    else
    {
        os_printf("No compatible sensor found!\r\n");
        free(sensor);
        sensor = NULLPTR;
    }


    //GLOBAL_INT_DECLARATION();
    //os_printf("camera_intfer_deinit,%p-%p\r\n", ejpeg_hdl, i2c_hdl);




    if(sensor == NULLPTR)
    {

        ddev_close(i2c_hdl);

        UINT32 i2c2_trans_mode = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)// master
            & (~I2C2_MSG_WORK_MODE_AL_BIT))// 7bit address
            | (I2C2_MSG_WORK_MODE_IA_BIT); // with inner address
        i2c_hdl = ddev_open(I2C2_DEV_NAME, &status, i2c2_trans_mode);

        os_printf("Searching for camera sensors, using I2C2 bus...\r\n");

        sensor = malloc(sizeof(camera_sensor_t));

        sensor->i2c_bus = I2C2_DEV_NAME;
        //g_softI2C.pin_clk = 0;
        //g_softI2C.pin_data = 1;
        //Soft_I2C_PreInit(&g_softI2C);
        //usleep(100);

        if(gc0328c_sensor_detect())
        {

            sensor->name = ("GC0328C");
            sensor->init = gc0328c_sensor_init;

        }
        else if(gc0310_sensor_detect())
        {

            sensor->name = ("GC0310/GC0312");
            sensor->init = gc0311_sensor_init;

        }
        else if(gc0311_sensor_detect())
        {

            sensor->name = ("GC0311");
            sensor->init = gc0311_sensor_init;

        }
        else if(hi704_sensor_detect())
        {

            sensor->name = ("HI704");
            sensor->init = hi704_sensor_init;

        }
        else if(gc0329c_sensor_detect())
        {

            sensor->name = ("GC0329C");
            sensor->init = gc0329c_sensor_init;

        }
        else
        {
            os_printf("No compatible sensor found!\r\n");
            free(sensor);
            sensor = NULLPTR;
        }

    }


    GLOBAL_INT_DECLARATION();
    //os_printf("camera_intfer_deinit,%p-%p\r\n", ejpeg_hdl, i2c_hdl);

    ddev_close(ejpeg_hdl);
    ddev_close(i2c_hdl);

    GLOBAL_INT_DISABLE();
    ejpeg_hdl = i2c_hdl = DD_HANDLE_UNVALID;
    GLOBAL_INT_RESTORE();

    return sensor;
}

/*---------------------------------------------------------------------------*/

#endif // CFG_USE_CAMERA_INTF
