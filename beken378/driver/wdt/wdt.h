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

#ifndef _WDT_H_
#define _WDT_H_

#define WDT_DEBUG

#ifdef WDT_DEBUG
#define WDT_PRT      os_printf
#define WDT_WARN     warning_prf
#define WDT_FATAL    fatal_prf
#else
#define WDT_PRT      null_prf
#define WDT_WARN     null_prf
#define WDT_FATAL    null_prf
#endif

#define WDT_BASE                                     (0x00802900)

#define WDT_CTRL_REG                                     (WDT_BASE + 0 * 4)
#define WDT_KEY_POSI                                              (16)
#define WDT_KEY_MASK                                              (0xFF)
#define WDT_1ST_KEY                                               (0x5A)
#define WDT_2ND_KEY                                               (0xA5)

#define WDT_PERIOD_POSI                                           (0)
#define WDT_PERIOD_MASK                                           (0xFFFF)

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern UINT32 wdt_ctrl(UINT32 cmd, void *param);

#endif //_WDT_H_

