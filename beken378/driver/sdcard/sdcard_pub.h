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

#ifndef _SDCARD_PUB_H_
#define _SDCARD_PUB_H_

#define SDCARD_SUCCESS                 (0)
#define SDCARD_FAILURE                 ((UINT32)-1)

#define SDCARD_DEV_NAME                ("sdcard")


#define SDCARD_CMD_MAGIC               (0x8709000)

enum
{
    CMD_SDCARD_SEND_BACKGROUND         = SDCARD_CMD_MAGIC + 0,
    CMD_SDCARD_RESET                   = SDCARD_CMD_MAGIC + 1,
};

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void sdcard_init(void);
extern void sdcard_exit(void);

#endif

