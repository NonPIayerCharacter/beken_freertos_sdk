#include <stdint.h>
#define GC0329C_DEV             (0xFFF05)
#define GC0329C_DEV_ID          (0x31)
#define GC0329C_DEV_CHIPID      (0xC0)

#include "drv_model_pub.h"
#include "camera_intf_pub.h"


// Sensor control functions
uint8_t gc0329c_sensor_detect(void);
void gc0329c_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl,  camera_sensor_t * sensor);

// Register tables
extern const uint8_t gc0329c_init_talbe[239 - 5][2];
