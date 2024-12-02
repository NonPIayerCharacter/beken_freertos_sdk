#ifndef _BK_LOS_TIMER_H_
#define _BK_LOS_TIMER_H_

#include "include.h"
#include "bk_timer_pub.h"

#if CFG_HUAWEI_ORIGINAL_ADAPT
#define LOS_TICK_TIMER_ID                  BKTIMER3
#define LOS_TICK_CTRL_REG                  TIMER3_5_CTL
#define LOS_TICK_RD_CTRL_REG               TIMER3_5_READ_CTL
#define LOS_TICK_RD_VAL_REG                TIMER3_5_READ_VALUE
#define LOS_TICK_UNIT_CLOCK                32000 
#else
#define LOS_TICK_TIMER_ID                  BKTIMER0
#define LOS_TICK_CTRL_REG                  TIMER0_2_CTL
#define LOS_TICK_RD_CTRL_REG               TIMER0_2_READ_CTL
#define LOS_TICK_RD_VAL_REG                TIMER0_2_READ_VALUE
#define LOS_TICK_UNIT_CLOCK                26000000 
#endif

#define LOS_TICK_TIMER_GROUP               ((LOS_TICK_TIMER_ID) >= 3 ? 1 : 0)
#define LOS_TICK_TIMER_RD_ID               (LOS_TICK_TIMER_ID - 3 * (LOS_TICK_TIMER_GROUP))

#define LOS_TICK_MS                        2
#define LOS_TICKS_PER_SECOND              (1000UL/LOS_TICK_MS)

extern UINT64 OsGetCurrSecond(VOID);

#endif // _BK_LOS_TIMER_H_
// eof
