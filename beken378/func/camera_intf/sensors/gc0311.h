#include <stdint.h>
#define GC0311_DEV              (0xFFF01)
#define GC0311_DEV_ID           (0x33)
#define GC0311_DEV_CHIPID       (0xBB)

#include "drv_model_pub.h"
#include "camera_intf_pub.h"

// Sensor control functions
uint8_t gc0311_sensor_detect(void);
void gc0311_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl,  camera_sensor_t * sensor);

void gc0311_camera_inf_cfg_ppi(uint32_t ppi_type);
void gc0311_camera_inf_cfg_fps(uint32_t fps_type);

// Register tables
extern const uint8_t gc0311_init_talbe[522][2];
extern const uint8_t gc0311_5pfs_talbe[9][2];
extern const uint8_t gc0311_10pfs_talbe[9][2];
extern const uint8_t gc0311_20pfs_talbe[9][2];
extern const uint8_t gc0311_QVGA_320_240_talbe[12][2];
extern const uint8_t gc0311_VGA_640_480_talbe[10][2];
