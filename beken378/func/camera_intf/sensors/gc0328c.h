#include <stdint.h>
#define GC0328C_DEV             (0xABC03)
#define GC0328C_DEV_ID          (0x21)
#define GC0328C_DEV_CHIPID      (0x9D)

#include "drv_model_pub.h"
#include "camera_intf_pub.h"


// Sensor control functions
uint8_t gc0328c_sensor_detect(void);
void gc0328c_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl,  camera_sensor_t * sensor);

void gc0328c_camera_inf_cfg_ppi(uint32_t ppi_type);
void gc0328c_camera_inf_cfg_fps(uint32_t fps_type);

// Register tables
extern const uint8_t gc0328c_init_talbe[374][2];
extern const uint8_t gc0328c_5pfs_talbe[9][2];
extern const uint8_t gc0328c_10pfs_talbe[9][2];
extern const uint8_t gc0328c_20pfs_talbe[9][2];
extern const uint8_t gc0328c_QVGA_320_240_talbe[12][2];
extern const uint8_t gc0328c_VGA_640_480_talbe[10][2];
