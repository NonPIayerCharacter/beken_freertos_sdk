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

#ifndef _BK7011_CAL_H_
#define _BK7011_CAL_H_

#if (CFG_SOC_NAME == SOC_BK7231)
#include "bk7231_cal.h"
#elif (CFG_SOC_NAME == SOC_BK7231U)
#include "bk7231u_cal.h"
#elif (CFG_SOC_NAME == SOC_BK7231N)
#include "bk7231n_cal.h"
#elif (CFG_SOC_NAME == SOC_BK7238)
#include "bk7238_cal.h"
#elif (CFG_SOC_NAME == SOC_BK7252N)
#include "bk7252n_cal.h"
#elif (CFG_SOC_NAME == SOC_BK7221U)
#include "bk7221u_cal.h"
#endif
#endif // _BK7011_CAL_H_

#ifndef SDK_COMMIT_ID
#define SDK_COMMIT_ID ""
#endif

typedef UINT16 heap_t;
size_t MinHeapInsert(heap_t *heap, size_t heap_size, heap_t x);
heap_t MinHeapReplace(heap_t *heap, size_t heap_size, heap_t x);

// eof

