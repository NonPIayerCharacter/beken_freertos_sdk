#ifndef _LA_PUB_H_
#define _LA_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#include "generic.h"
#include "drv_model_pub.h"

#define LA_DEV_NAME     "la"

typedef enum {
    LA_TRIG_MODE_0_EQUAL_TO_LASMPVALUE,
    LA_TRIG_MODE_1_CHANGE,
    LA_TRIG_MODE_RES
} LA_TRIG_MODE;

typedef enum {
    LA_SMP_INT_EN_MODE_TRANSFER_FINISH_INT,
    LA_SMP_INT_EN_MODE_BUS_ERR_INT,
    LA_SMP_INT_EN_MODE_BOTH
} LA_SMP_INT_EN_MODE;

typedef enum {
    LA_BUS_ERR_FLAG_EN,
    LA_BUS_ERR_FLAG_CLR
} LA_BUS_ERR_FLAG;

extern void la_init(void);
extern void la_exit(void);

#endif

#endif // _LA_PUB_H_

// EOF

