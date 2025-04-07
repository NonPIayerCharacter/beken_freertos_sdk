#ifndef _CHARGE_PUB_H_
#define _CHARGE_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#include "generic.h"
#include "drv_model_pub.h"

#define CHARGE_DEV_NAME     "charge"

#define CHARGE_CMD_MAGIC              (0x0)
enum
{
    CMD_VCAL_CALIBRATE = CHARGE_CMD_MAGIC + 1,
    CMD_ICAL_CALIBRATE_PREPARE,
    CMD_ICAL_CALIBRATE_TRIGGER,
    CMD_ICAL_CALIBRATE,
    CMD_ICAL_VERIFY,
    CMD_VCAL_VERIFY,
    CMD_INT_STATE_GET
};


typedef enum {
    CHARGE_0 = 0,
    CHARGE_1,
    CHARGE_2,
    CHARGE_3,
    CHARGE_4,
    CHARGE_5,
    CHARGE_6,
    CHARGE_7 = 7
} CHARGE_INDEX;

typedef enum {
    CHARGE_INT_TYPE_L_LV = 0,
    CHARGE_INT_TYPE_H_LV,
    CHARGE_INT_TYPE_P_EDGE,
    CHARGE_INT_TYPE_N_EDGE = 3
} CHARGE_INT_TYPE;

typedef enum {
    CHARGE_INT_DIS,
    CHARGE_INT_EN
} CHARGE_INT_CTRL;

extern void charge_init(void);
extern void charge_exit(void);

#endif
#endif // _CHARGE_PUB_H_
// eof

