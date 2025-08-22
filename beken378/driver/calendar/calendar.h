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

#ifndef _CALENDAR_H_
#define _CALENDAR_H_

#define CALENDAR_ADDR_BASE                  (0x00800000)

#define CAL_CONFIG                          (CALENDAR_ADDR_BASE + 0x55 * 4)
#define CAL_INIT_VAL_POS                    (0)
#define CAL_INIT_VAL_MASK                   (0xFFFFFFFF)

#define CAL_ENABLE                          (CALENDAR_ADDR_BASE + 0x56 * 4)
#define CAL_ENABLE_POS                      (0)
#define CAL_ENABLE_MASK                     (0x1)

#define CAL_SECOND_VALUE                    (CALENDAR_ADDR_BASE + 0x57 * 4)
#define CAL_SEC_VAL_POS                     (0)
#define CAL_SEC_VAL_MASK                    (0xFFFFFFFF)

#define CAL_31_25US_VALUE                   (CALENDAR_ADDR_BASE + 0x58 * 4)
#define CAL_31_25US_POS                     (0)
#define CAL_31_25US_MASK                    (0x0000FFFF)
#define CAL_3125_TU_VAL                     (3125)

extern UINT32 cal_ctrl(UINT32 cmd, void *param);

#endif // _CALENDAR_H_
// eof


