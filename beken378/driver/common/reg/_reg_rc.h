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

#ifndef __REG_RC_H_
#define __REG_RC_H_

#define REG_RC_SIZE           428

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_RC_BASE_ADDR            0x01050000
#define REG_RC_BASE_ADDR_MASK       0xffff0000
#else
#define REG_RC_BASE_ADDR            0x0080d000
#define REG_RC_BASE_ADDR_MASK       0x00fff000
#endif

#endif // __REG_RC_H_

