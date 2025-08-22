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

#ifndef __MUSB_NONE_BOARD_H__
#define __MUSB_NONE_BOARD_H__

#include "mu_dsi.h"
#include "mu_none.h"

#include "include.h"
#include "rtos_pub.h"

#if CFG_USB
/**
 * @field iVector uHAL's vector for reverse-lookup
 * @field iIndex uHAL's timer index
 * @field pfExpired expiration callback
 * @field pParam expiration callback parameter
 * @field dwTime remaining time, due to uHAL's MAX_PERIOD limitation
 * @field bPeriodic whether currently set for periodic
 */
typedef struct
{
    /* timer implementation, and it depends on operating system*/
    beken2_timer_t timer;

    unsigned int iVector;
    unsigned int iIndex;
    MUSB_pfTimerExpired pfExpired;

    void *pParam;
    uint32_t dwTime;
    uint8_t bPeriodic;
    uint8_t bTimerStart;
} MGC_AfsTimerWrapper;

/**
 * MGC_AfsUds.
 * Board-specific UDS instance data.
 * @param pNonePrivateData non-OS UDS instance data
 * @param pfNoneIsr non-OS UDS ISR
 * @field aTimerWrapper timer wrappers
 * @field wTimerCount how many wrappers
 * @field bIndex our index into the global array
 */
typedef struct
{
    char aIsrName[8];
    void *pNonePrivateData;
    MUSB_NoneIsr pfNoneIsr;
    void *pPciAck;

    MGC_AfsTimerWrapper *aTimerWrapper;
    unsigned int dwIrq;
    uint16_t wTimerCount;
    uint8_t bIndex;
} MGC_AfsUds;

extern MUSB_NoneController MUSB_aNoneController[];
extern void MGC_AfsUdsIsr(void);
#endif

#endif	/* multiple inclusion protection */
