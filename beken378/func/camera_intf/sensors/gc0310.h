#include <stdint.h>
#define GC0310_DEV             (0xFFF05)
#define GC0310_DEV_ID          (0x21)
#define GC0310_DEV_CHIPID      (0x10)

#include "drv_model_pub.h"
#include "camera_intf_pub.h"


// Sensor control functions
uint8_t gc0310_sensor_detect(void);
void gc0310_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl, camera_sensor_t * sensor);

// Register tables
extern const uint8_t gc0310_init_talbe[293 - 5][2];
