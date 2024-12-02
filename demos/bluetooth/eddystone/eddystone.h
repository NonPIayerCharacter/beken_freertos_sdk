#ifndef _EDDYSTONE_H_
#define _EDDYSTONE_H_
#include "bk_err.h"
#include "ble_api_5_x.h"
#include "app_ble.h"

extern int demo_start(void);
extern ble_err_t ble_eddystone_post_msg(uint16_t msg_id, void *data,uint32_t len);

#endif // _EDDYSTONE_H_
// EOF

