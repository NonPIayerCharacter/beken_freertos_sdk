#include <stdint.h>
#include "include.h"
#include "arm_arch.h"
#include "drv_model_pub.h"
#include "jpeg_encoder_pub.h"
#include "video_transfer.h"
#include "i2c_pub.h"
#include "camera_intf.h"
#include "camera_intf_pub.h"

#include "gc0310.h"

uint8_t gc0310_sensor_detect(void)
{

    uint8_t data;
    uint8_t addr = 0xf1;

    camera_intf_sccb_read2(GC0310_DEV_ID, addr, &data, 1);
    os_printf("Trying GC0310 at 0x%02X, expected 0x%02X got 0x%02X\r\n", addr, GC0310_DEV_CHIPID, data);
    uint8_t found = (data == GC0310_DEV_CHIPID);

    if(found)
    {
        os_printf("Found sensor GC0310!\r\n");
    }

    return found;
}

void gc0310_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl, camera_sensor_t* sensor)
{

    os_printf("Intializing GC0310 sensor.\r\n");

    uint32_t i, size;
    uint8_t addr, data;
    //I2C_OP_ST i2c_operater;
    //DJPEG_DESC_ST ejpeg_cfg;

    sensor->i2c_cfg->salve_id = GC0310_DEV_ID;

    size = sizeof(gc0310_init_talbe) / 2;

    for(i = 0; i < size; i++)
    {

        addr = gc0310_init_talbe[i][0];
        data = gc0310_init_talbe[i][1];
        camera_intf_sccb_write(addr, data);
    }

    //gc0310_camera_inf_cfg_ppi(CMPARAM_GET_PPI(sensor->ejpeg_cfg->sener_cfg));
    //gc0310_camera_inf_cfg_fps(CMPARAM_GET_FPS(sensor->ejpeg_cfg->sener_cfg));

    os_printf("GC0310 initialization finished.\r\n");


}
