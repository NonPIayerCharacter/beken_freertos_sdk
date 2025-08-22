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

