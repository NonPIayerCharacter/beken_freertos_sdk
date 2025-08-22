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

#ifndef _DRV_MODEL_H_
#define _DRV_MODEL_H_

#define DD_INITIAL_SIZE                  (10)
#define DD_EXP_STEP_SIZE                 (5)

#define DD_MAX_NAME_LEN         (16)

typedef enum _dd_state_
{
    DD_STATE_NODEVICE = 0,	// find no such device when you open
    DD_STATE_CLOSED,		//
    DD_STATE_OPENED,		//
    DD_STATE_BREAK,			//
    DD_STATE_SUCCESS		//
} DD_STATE;

typedef struct _drv_dev_
{
    char *name;
    UINT32 use_cnt;

    DD_STATE state;
    DD_OPERATIONS *op;
    DD_OPEN_METHOD method;

    void *private;
} DRV_DEV_S, *DRV_DEV_PTR;

typedef struct _drv_sdev_
{
    char *name;
    UINT32 use_cnt;

    DD_STATE state;
    SDD_OPERATIONS *op;
    DD_OPEN_METHOD method;

    void *private;
} DRV_SDEV_S, *DRV_SDEV_PTR;

#endif // _DRV_MODEL_H_

