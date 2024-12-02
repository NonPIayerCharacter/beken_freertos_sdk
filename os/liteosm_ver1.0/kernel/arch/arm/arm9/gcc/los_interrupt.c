/*
 * Copyright (c) 2013-2019 Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 *    of conditions and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "los_interrupt.h"
#include <stdarg.h>
#include "securec.h"
#include "los_context.h"
#include "los_arch_interrupt.h"
#include "los_debug.h"
#include "los_hook.h"
#include "los_task.h"
#include "los_sched.h"
#include "los_memory.h"
#include "los_membox.h"
#include "los_reg.h"
#include "intc_pub.h"

UINT32 g_intCount = 0;
ExcInfo g_excInfo = {0};
UINT32 g_fiqBreakIrqFlag = 0;

extern void intc_irq(void);
extern void intc_fiq(void);

LITE_OS_SEC_TEXT_MINOR UINT32 HalFiqBreakinIrqGet(VOID)
{
    return g_fiqBreakIrqFlag;
}

LITE_OS_SEC_TEXT_MINOR VOID HalFiqBreakinIrqSet(VOID)
{
    g_fiqBreakIrqFlag = 1;
}

LITE_OS_SEC_TEXT_MINOR VOID HalFiqBreakinIrqClear(VOID)
{
    g_fiqBreakIrqFlag = 0;
}

/* ****************************************************************************
 Function    : HalIntNumGet
 Description : Get an interrupt number
 Input       : None
 Output      : None
 Return      : Interrupt Indexes number
 **************************************************************************** */
LITE_OS_SEC_TEXT_MINOR UINT32 HalIntNumGet(VOID)
{
    return 0;
}

inline UINT32 HalIsIntActive(VOID)
{
    return (g_intCount > 0);
}
/* ****************************************************************************
 Function    : HalHwiDefaultHandler
 Description : default handler of the hardware interrupt
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
/*lint -e529*/
LITE_OS_SEC_TEXT_MINOR VOID HalHwiDefaultHandler(VOID)
{
    UINT32 irqNum = HalIntNumGet();
    PRINT_ERR("%s irqnum:%d\n", __FUNCTION__, irqNum);
	
    while (1) 
	{
		;
	}

	(void)irqNum;
}

VOID HalIrqMask(UINT32 vector)
{
    return;
}

VOID HalIrqUnmask(UINT32 vector)
{
    return;
}


/* ****************************************************************************
 Function    : HalInterrupt_irq
 Description : Hardware interrupt entry function
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
LITE_OS_SEC_TEXT VOID HalInterrupt_irq(VOID)
{
    UINT32 intSave = 0;

    intSave = LOS_IntLock();
    g_intCount++;
    LOS_IntRestore(intSave);

#if (LOSCFG_BASE_CORE_SCHED_SLEEP == 1)
    OsSchedUpdateSleepTime();
#endif

    /* Interrupt handling function. */
    intc_irq();

    intSave = LOS_IntLock();
    g_intCount--;
    LOS_IntRestore(intSave);
}


/* ****************************************************************************
 Function    : HalInterrupt_fiq
 Description : Hardware interrupt entry function
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
LITE_OS_SEC_TEXT VOID HalInterrupt_fiq(VOID)
{
    UINT32 intSave = 0;

    intSave = LOS_IntLock();
    g_intCount++;
    LOS_IntRestore(intSave);

#if (LOSCFG_BASE_CORE_SCHED_SLEEP == 1)
    OsSchedUpdateSleepTime();
#endif

    /* Interrupt handling function. */
    intc_fiq();

    intSave = LOS_IntLock();
    g_intCount--;
    LOS_IntRestore(intSave);
}

/* ****************************************************************************
 Function    : HalHwiCreate
 Description : create hardware interrupt
 Input       : hwiNum   --- hwi num to create
               hwiPrio  --- priority of the hwi
               mode     --- unused
               handler --- hwi handler
               arg      --- param of the hwi handler
 Output      : None
 Return      : LOS_OK on success or error code on failure
 **************************************************************************** */
LITE_OS_SEC_TEXT_INIT UINT32 HalHwiCreate(HWI_HANDLE_T hwiNum,
                                          HWI_PRIOR_T hwiPrio,
                                          HWI_MODE_T mode,
                                          HWI_PROC_FUNC handler,
                                          HWI_ARG_T arg)
{
	bk_printf("HalHwiCreate TODO\r\n");
	intc_service_register((UINT8)hwiNum, (UINT8)hwiPrio, (FUNCPTR)handler);
    return LOS_OK;
}

/* ****************************************************************************
 Function    : HalHwiDelete
 Description : Delete hardware interrupt
 Input       : hwiNum   --- hwi num to delete
 Output      : None
 Return      : LOS_OK on success or error code on failure
 **************************************************************************** */
LITE_OS_SEC_TEXT_INIT UINT32 HalHwiDelete(HWI_HANDLE_T hwiNum)
{
	bk_printf("HalHwiDelete TODO\r\n");
	intc_service_unregister((UINT8)hwiNum);
    return LOS_OK;
}

LITE_OS_SEC_TEXT_INIT UINT32 HalHwiTrigger(HWI_HANDLE_T hwiNum)
{
	bk_printf("HalHwiTrigger TODO\r\n");
	intc_service_trigger((UINT8)hwiNum);
    return LOS_OK;
}

#if (LOSCFG_KERNEL_PRINTF != 0)
STATIC VOID OsExcTypeInfo(const ExcInfo *excInfo)
{
    CHAR *phaseStr[] = {"exc in init", "exc in task", "exc in hwi"};

    PRINTK("Type      = %d\n", excInfo->type);
    PRINTK("ThrdPid   = %d\n", excInfo->thrdPid);
    PRINTK("Phase     = %s\n", phaseStr[excInfo->phase]);
    PRINTK("FaultAddr = 0x%x\n", excInfo->faultAddr);
}

STATIC VOID OsExcCurTaskInfo(const ExcInfo *excInfo)
{
    PRINTK("Current task info:\n");
    if (excInfo->phase == OS_EXC_IN_TASK) {
        LosTaskCB *taskCB = OS_TCB_FROM_TID(LOS_CurTaskIDGet());
        PRINTK("Task name = %s\n", taskCB->taskName);
        PRINTK("Task ID   = %d\n", taskCB->taskID);
        PRINTK("Task SP   = 0x%x\n", taskCB->stackPointer);
        PRINTK("Task ST   = 0x%x\n", taskCB->topOfStack);
        PRINTK("Task SS   = 0x%x\n", taskCB->stackSize);
    } else if (excInfo->phase == OS_EXC_IN_HWI) {
        PRINTK("Exception occur in interrupt phase!\n");
    } else {
        PRINTK("Exception occur in system init phase!\n");
    }
}

STATIC VOID OsExcRegInfo(const ExcInfo *excInfo)
{
    PRINTK("Exception reg dump:\n");
    PRINTK("PC        = 0x%x\n", excInfo->context->uwPC);
    PRINTK("LR        = 0x%x\n", excInfo->context->uwLR);
    PRINTK("R0        = 0x%x\n", excInfo->context->uwR0);
    PRINTK("R1        = 0x%x\n", excInfo->context->uwR1);
    PRINTK("R2        = 0x%x\n", excInfo->context->uwR2);
    PRINTK("R3        = 0x%x\n", excInfo->context->uwR3);
    PRINTK("R4        = 0x%x\n", excInfo->context->uwR4);
    PRINTK("R5        = 0x%x\n", excInfo->context->uwR5);
    PRINTK("R6        = 0x%x\n", excInfo->context->uwR6);
    PRINTK("R7        = 0x%x\n", excInfo->context->uwR7);
    PRINTK("R8        = 0x%x\n", excInfo->context->uwR8);
    PRINTK("R9        = 0x%x\n", excInfo->context->uwR9);
    PRINTK("R10       = 0x%x\n", excInfo->context->uwR10);
    PRINTK("R11       = 0x%x\n", excInfo->context->uwR11);
    PRINTK("R12       = 0x%x\n", excInfo->context->uwR12);
    PRINTK("xPSR      = 0x%x\n", excInfo->context->uwPSR);
}

STATIC VOID OsExcBackTraceInfo(const ExcInfo *excInfo)
{
    UINTPTR LR[LOSCFG_BACKTRACE_DEPTH] = {0};
    UINT32 index;

    OsBackTraceHookCall(LR, LOSCFG_BACKTRACE_DEPTH, 0, 0);

    PRINTK("----- backtrace start -----\n");
    for (index = 0; index < LOSCFG_BACKTRACE_DEPTH; index++) {
        if (LR[index] == 0) {
            break;
        }
        PRINTK("backtrace %d -- lr = 0x%x\n", index, LR[index]);
    }
    PRINTK("----- backtrace end -----\n");
}

STATIC VOID OsExcMemPoolCheckInfo(VOID)
{
    PRINTK("\r\nmemory pools check:\n");
#if (LOSCFG_PLATFORM_EXC == 1)
    MemInfoCB memExcInfo[OS_SYS_MEM_NUM];
    UINT32 errCnt;
    UINT32 i;

    (VOID)memset_s(memExcInfo, sizeof(memExcInfo), 0, sizeof(memExcInfo));

    errCnt = OsMemExcInfoGet(OS_SYS_MEM_NUM, memExcInfo);
    if (errCnt < OS_SYS_MEM_NUM) {
        errCnt += OsMemboxExcInfoGet(OS_SYS_MEM_NUM - errCnt, memExcInfo + errCnt);
    }

    if (errCnt == 0) {
        PRINTK("all memory pool check passed!\n");
        return;
    }

    for (i = 0; i < errCnt; i++) {
        PRINTK("pool num    = %d\n", i);
        PRINTK("pool type   = %d\n", memExcInfo[i].type);
        PRINTK("pool addr   = 0x%x\n", memExcInfo[i].startAddr);
        PRINTK("pool size   = 0x%x\n", memExcInfo[i].size);
        PRINTK("pool free   = 0x%x\n", memExcInfo[i].free);
        PRINTK("pool blkNum = %d\n", memExcInfo[i].blockSize);
        PRINTK("pool error node addr  = 0x%x\n", memExcInfo[i].errorAddr);
        PRINTK("pool error node len   = 0x%x\n", memExcInfo[i].errorLen);
        PRINTK("pool error node owner = %d\n", memExcInfo[i].errorOwner);
    }
#endif
    UINT32 ret = LOS_MemIntegrityCheck(LOSCFG_SYS_HEAP_ADDR);
    if (ret == LOS_OK) {
        PRINTK("system heap memcheck over, all passed!\n");
    }

    PRINTK("memory pool check end!\n");
}
#endif

STATIC VOID OsExcInfoDisplay(const ExcInfo *excInfo)
{
#if (LOSCFG_KERNEL_PRINTF != 0)
    PRINTK("*************Exception Information**************\n");
    OsExcTypeInfo(excInfo);
    OsExcCurTaskInfo(excInfo);
    OsExcRegInfo(excInfo);
    OsExcBackTraceInfo(excInfo);
    OsGetAllTskInfo();
    OsExcMemPoolCheckInfo();
#endif
}

LITE_OS_SEC_TEXT_INIT VOID HalExcHandleEntry(UINT32 excType, UINT32 faultAddr, UINT32 pid, EXC_CONTEXT_S *excBufAddr)
{
    UINT16 tmpFlag = (excType >> 16) & OS_NULL_SHORT; /* 16: Get Exception Type */
    g_intCount++;
    g_excInfo.nestCnt++;

    g_excInfo.type = excType & OS_NULL_SHORT;

    if (tmpFlag & OS_EXC_FLAG_FAULTADDR_VALID) {
        g_excInfo.faultAddr = faultAddr;
    } else {
        g_excInfo.faultAddr = OS_EXC_IMPRECISE_ACCESS_ADDR;
    }
    if (g_losTask.runTask != NULL) {
        if (tmpFlag & OS_EXC_FLAG_IN_HWI) {
            g_excInfo.phase = OS_EXC_IN_HWI;
            g_excInfo.thrdPid = pid;
        } else {
            g_excInfo.phase = OS_EXC_IN_TASK;
            g_excInfo.thrdPid = g_losTask.runTask->taskID;
        }
    } else {
        g_excInfo.phase = OS_EXC_IN_INIT;
        g_excInfo.thrdPid = OS_NULL_INT;
    }
    if (excType & OS_EXC_FLAG_NO_FLOAT) {
        g_excInfo.context = (EXC_CONTEXT_S *)((CHAR *)excBufAddr - LOS_OFF_SET_OF(EXC_CONTEXT_S, uwR4));
    } else {
        g_excInfo.context = excBufAddr;
    }

    OsDoExcHook(EXC_INTERRUPT);
    OsExcInfoDisplay(&g_excInfo);
    HalSysExit();
}

/* ****************************************************************************
 Function    : HalHwiInit
 Description : initialization of the hardware interrupt
 Input       : None
 Output      : None
 Return      : None
 **************************************************************************** */
LITE_OS_SEC_TEXT_INIT VOID HalHwiInit(VOID)
{
    return;
}

