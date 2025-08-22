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

#include "include.h"
#include "rtos_pub.h"
#include "BkDriverWdg.h"
#include "drv_model_pub.h"
#include "error.h"
#include "wdt_pub.h"

OSStatus bk_wdg_initialize( uint32_t timeout )
{
    UINT32 ret, param;

    ret = sddev_control(WDT_DEV_NAME, WCMD_POWER_UP, 0);
    ASSERT(WDT_SUCCESS == ret);

    param = timeout;
    ret = sddev_control(WDT_DEV_NAME, WCMD_SET_PERIOD, &param);
    ASSERT(WDT_SUCCESS == ret);

    return kNoErr;
}

void bk_wdg_reload( void )
{
    UINT32 ret;

    ret = sddev_control(WDT_DEV_NAME, WCMD_RELOAD_PERIOD, 0);
    ASSERT(WDT_SUCCESS == ret);

    return;
}

OSStatus bk_wdg_finalize( void )
{
    UINT32 ret;

    ret = sddev_control(WDT_DEV_NAME, WCMD_POWER_DOWN, 0);
    ASSERT(WDT_SUCCESS == ret);

    return kNoErr;
}

// eof

