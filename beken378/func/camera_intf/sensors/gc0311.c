#include <stdint.h>
#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "jpeg_encoder_pub.h"
#include "video_transfer.h"
#include "i2c_pub.h"
#include "camera_intf.h"
#include "camera_intf_pub.h"

#include "gc0311.h"

uint8_t gc0311_sensor_detect(void)
{

    uint8_t data;
    uint8_t addr = 0xF0;

    camera_intf_sccb_read2(GC0311_DEV_ID, addr, &data, 1);
    os_printf("Trying GC0311 at 0x%02X, expected 0x%02X got 0x%02X\r\n", addr, GC0311_DEV_CHIPID, data);
    uint8_t found = (data == GC0311_DEV_CHIPID);

    if(found)
    {
        os_printf("Found sensor GC0311!");
    }

    return found;
}

void gc0311_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl, camera_sensor_t* sensor)
{

    os_printf("Intializing GC0311 sensor.\r\n");

    uint32_t i, size;
    uint8_t addr, data;
    //I2C_OP_ST i2c_operater;
    //DJPEG_DESC_ST ejpeg_cfg;

    sensor->i2c_cfg->salve_id = GC0311_DEV_ID;

    size = sizeof(gc0311_init_talbe) / 2;

    for(i = 0; i < size; i++)
    {

        addr = gc0311_init_talbe[i][0];
        data = gc0311_init_talbe[i][1];
        camera_intf_sccb_write(addr, data);
    }

    gc0311_camera_inf_cfg_ppi(CMPARAM_GET_PPI(sensor->ejpeg_cfg->sener_cfg));
    gc0311_camera_inf_cfg_fps(CMPARAM_GET_FPS(sensor->ejpeg_cfg->sener_cfg));

    os_printf("GC0311 initialization finished.\r\n");


}

void gc0311_camera_inf_cfg_ppi(uint32_t ppi_type)
{

    uint32_t i, size;
    uint8_t addr, data;

    os_printf("Setting PPI: %d\r\n", ppi_type);

    switch(ppi_type)
    {

        case QVGA_320_240:
            size = sizeof(gc0311_QVGA_320_240_talbe) / 2;
            for(i = 0; i < size; i++)
            {
                addr = gc0311_QVGA_320_240_talbe[i][0];
                data = gc0311_QVGA_320_240_talbe[i][1];
                camera_intf_sccb_write(addr, data);
            }
            break;

        case VGA_640_480:
            size = sizeof(gc0311_VGA_640_480_talbe) / 2;
            for(i = 0; i < size; i++)
            {
                addr = gc0311_VGA_640_480_talbe[i][0];
                data = gc0311_VGA_640_480_talbe[i][1];
                camera_intf_sccb_write(addr, data);
            }
            break;

        default:
            os_printf("set PPI unknown\r\n");
            break;
    }
}

void gc0311_camera_inf_cfg_fps(uint32_t fps_type)
{

    uint32_t i, size;
    uint8_t addr, data;

    os_printf("Setting FPS: %d\r\n", fps_type);

    switch(fps_type)
    {

        case TYPE_5FPS:
            size = sizeof(gc0311_5pfs_talbe) / 2;
            for(i = 0; i < size; i++)
            {
                addr = gc0311_5pfs_talbe[i][0];
                data = gc0311_5pfs_talbe[i][1];
                camera_intf_sccb_write(addr, data);
            }
            break;

        case TYPE_10FPS:
            size = sizeof(gc0311_10pfs_talbe) / 2;
            for(i = 0; i < size; i++)
            {
                addr = gc0311_10pfs_talbe[i][0];
                data = gc0311_10pfs_talbe[i][1];
                camera_intf_sccb_write(addr, data);
            }
            break;

        case TYPE_20FPS:
            size = sizeof(gc0311_20pfs_talbe) / 2;
            for(i = 0; i < size; i++)
            {
                addr = gc0311_20pfs_talbe[i][0];
                data = gc0311_20pfs_talbe[i][1];
                camera_intf_sccb_write(addr, data);
            }
            break;

        default:
            os_printf("set FPS unknown\r\n");
            break;
    }
}