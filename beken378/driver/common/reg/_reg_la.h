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

#ifndef __REG_LA_H_
#define __REG_LA_H_

#include "sys_config.h"

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
#define REG_LA_OFFSET 0x00000000

#define REG_LA_BASE_ADDR 0x00808000
#elif (CFG_SOC_NAME == SOC_BK7252N)
#define REG_LA_OFFSET 0x00000000

#define REG_LA_BASE_ADDR 0x00A05000
#else
#define REG_LA_OFFSET 0x00800000

#define REG_LA_BASE_ADDR 0x10E00000
#endif

#endif // __REG_LA_H_

