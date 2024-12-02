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
#elif (CFG_SOC_NAME == SOC_BK7221U)
#include "bk7221u_cal.h"
#endif
#endif // _BK7011_CAL_H_

typedef UINT16 heap_t;
size_t MinHeapInsert(heap_t *heap, size_t heap_size, heap_t x);
heap_t MinHeapReplace(heap_t *heap, size_t heap_size, heap_t x);

// eof

