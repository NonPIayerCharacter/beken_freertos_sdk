#define HI704_DEV              (0xFFF01)
#define HI704_DEV_ID           (0x30)
#define HI704_DEV_CHIPID       (0x96)

#include "drv_model_pub.h"
#include "camera_intf_pub.h"


// Sensor control functions
uint8_t hi704_sensor_detect(void);
void hi704_sensor_init(DD_HANDLE i2c_hdl, DD_HANDLE ejpeg_hdl, camera_sensor_t* sensor);

// Register tables
const unsigned char hi704_sensor_init_sequence[704][2];

/*

Detect:

133057366-133057663 I²C: Address/data: Data write: 04
133057663-133057700 I²C: Address/data: ACK
133057886-133057886 I²C: Address/data: Start repeat
133057921-133058181 I²C: Address/data: Address read: 30
133058181-133058218 I²C: Address/data: Read
133058218-133058255 I²C: Address/data: ACK
133058471-133058767 I²C: Address/data: Data read: 96
133059064-133059101 I²C: Address/data: NACK
133059127-133059127 I²C: Address/data: Stop


*/
