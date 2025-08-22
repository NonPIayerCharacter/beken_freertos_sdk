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

#ifndef _TARGET_UTIL_PUB_H_
#define _TARGET_UTIL_PUB_H_

extern void delay(INT32 num);
extern void delay_us(UINT32 ms_count);
extern void delay_ms(UINT32 ms_count);
extern void delay_sec(UINT32 ms_count);
extern void delay_tick(UINT32 tick_count);

#endif // _TARGET_UTIL_PUB_H_
