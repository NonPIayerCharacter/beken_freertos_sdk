#ifndef _HPM_PUB_H_
#define _HPM_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#include "generic.h"
#include "drv_model_pub.h"

#define HPM_DEV_NAME     "hpm"

typedef enum {
    HPM_MODE_MONITOR_EN,
    HPM_MODE_SINGLE_EN,
    HPM_MODE_MONITOR_DIS,
    HPM_MODE_SINGLE_DIS
} HPM_MODE;

typedef enum {
    HPM_RECORD_0,
    HPM_RECORD_1,
    HPM_RECORD_2,
    HPM_RECORD_3
} HPM_RECORD_INDEX;

extern void hpm_init(void);
extern void hpm_exit(void);

#endif

#endif // _HPM_PUB_H_

// EOF

