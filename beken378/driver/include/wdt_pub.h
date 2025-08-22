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

#ifndef _WDT_PUB_H_
#define _WDT_PUB_H_

#define WDT_FAILURE                (1)
#define WDT_SUCCESS                (0)

#define WDT_DEV_NAME                "wdt"

#define WDT_CMD_MAGIC              (0xe330000)
enum
{
    WCMD_POWER_UP = WDT_CMD_MAGIC + 1,
    WCMD_SET_PERIOD,
    WCMD_RELOAD_PERIOD,
    WCMD_POWER_DOWN
};

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void wdt_init(void);
extern void wdt_exit(void);

#endif //_WDT_PUB_H_

