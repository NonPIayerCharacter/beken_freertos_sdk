// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

