/*
 * Copyright (c) 2013-2019 Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
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

#include "osTest.h"
#include "los_config.h"
#include "los_swtmr.h"
#if (LITEOS_CMSIS_TEST == 1)
#include "cmsis_os.h"
#endif

UINT32 volatile g_testCount;
UINT32 g_testTskHandle;
UINT32 g_testTaskID01;
UINT32 g_testTaskID02;
UINT32 g_testTaskID03;
UINT32 g_testTaskID04;

EVENT_CB_S g_eventCB01;
EVENT_CB_S g_eventCB02;
EVENT_CB_S g_eventCB03;
UINT32 g_mutexTest;
EVENT_CB_S g_exampleEvent;

UINT32 g_hwiNum1;
UINT32 g_hwiNum2;
UINT32 g_usSemID;
UINT32 g_usSemID2;
UINT32 g_cpupTestCount;
UINT32 g_cmsisRobinCount1;
UINT32 g_cmsisCount;

UINT16 g_usSwTmrID;

UINT32 g_testQueueID01;
UINT32 g_testQueueID02;
UINT32 g_testQueueID03;

UINT16 g_index;
UINT32 g_loopCycle = 0xFFFFF;

UINT32 g_passResult = 0;
UINT32 g_failResult = 0;
UINT32 g_testTskHandle;

UINT32 g_leavingTaskNum;
UINT32 g_testTaskIdArray[LOSCFG_BASE_CORE_TSK_LIMIT] = {0};
UINT32 g_uwGetTickConsume = 0;

UINT32 g_usSemID3[LOSCFG_BASE_IPC_SEM_CONFIG + 1];

#define TST_RAMADDRSTART 0x20000000
#define TST_RAMADDREND 0x20010000




#if (CMSIS_OS_VER == 2)
extern SWTMR_CTRL_S *g_swtmrCBArray;
UINT32 SwtmrCountGetTest(VOID)
{
    UINT32 loop;
    UINT32 swTmrCnt = 0;
    UINT32 intSave = 0;
    SWTMR_CTRL_S *swTmrCB = (SWTMR_CTRL_S *)NULL;

    intSave = LOS_IntLock();
    swTmrCB = g_swtmrCBArray;
    for (loop = 0; loop < LOSCFG_BASE_CORE_SWTMR_LIMIT; loop++, swTmrCB++) {
        if (swTmrCB->ucState != OS_SWTMR_STATUS_UNUSED) {
            swTmrCnt++;
        }
    }
    (VOID)LOS_IntRestore(intSave);
    return swTmrCnt;
}
#endif

void TestKernel(void)
{
#if (LOS_KERNEL_CORE_TASK_TEST == 1)
    ItSuiteLosTask();
#endif
#if (LOS_KERNEL_IPC_QUEUE_TEST == 1)
    ItSuiteLosQueue();
#endif
#if (LOS_KERNEL_IPC_MUX_TEST == 1)
    ItSuiteLosMux();
#endif
#if (LOS_KERNEL_IPC_EVENT_TEST == 1)
    ItSuiteLosEvent();
#endif
#if (LOS_KERNEL_IPC_SEM_TEST == 1)
    ItSuiteLosSem();
#endif
#if (LOS_KERNEL_CORE_SWTMR_TEST == 1)
    ItSuiteLosSwtmr();
#endif
#if (LOS_KERNEL_HWI_TEST == 1)
    ItSuiteLosHwi();
#endif
#if (LOS_KERNEL_MEM_TEST == 1)
    ItSuiteLosMem();
#endif
}


#if (CMSIS_OS_VER == 2)
void TestCmsis2(void)
{
#if (LOS_CMSIS2_CORE_TASK_TEST == 1)
    ItSuite_Cmsis_Los_Task();
#endif

#if (LOS_CMSIS2_IPC_EVENT_TEST == 1)
    ItSuite_Cmsis_Los_Event();
#endif
#if (LOS_CMSIS2_CORE_SWTMR_TEST == 1)
    ItSuite_Cmsis_Los_Swtmr();
#endif
#if (LOS_CMSIS2_IPC_SEM_TEST == 1)
    ItSuite_Cmsis_Los_Sem();
#endif
#if (LOS_CMSIS2_IPC_MUX_TEST == 1)
    ItSuite_Cmsis_Los_Mux();
#endif
#if (LOS_CMSIS2_HWI_TEST == 1)
    ItSuite_Cmsis_Los_Hwi();
#endif
#if (LOS_CMSIS2_IPC_MSG_TEST == 1)
    ItSuite_Cmsis_Los_Msg();
#endif
}
#endif

VOID TestTaskEntry()
{
    PRINTF("\t\n --- Test Start --- \n\n");
    ICunitInit();

    TestKernel();

#if (CMSIS_OS_VER == 1)
    PRINTF("\n ---  cmsis 1 unsupport--- \n\n");
#elif (CMSIS_OS_VER == 2)
    TestCmsis2();
#endif

    PRINTF("\t\n --- Test End --- \n");
    PRINTF("\nfailed count : %d, success count:%d\n", g_failResult, g_passResult);
}

UINT32 los_TestInit(VOID)
{
    UINT32 ret;
    TSK_INIT_PARAM_S osTaskInitParam;

    osTaskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)TestTaskEntry;
    osTaskInitParam.uwStackSize = OS_TSK_TEST_STACK_SIZE;
    osTaskInitParam.pcName = "IT_TST_INI";
    osTaskInitParam.usTaskPrio = TASK_PRIO_TEST;
    osTaskInitParam.uwResved = LOS_TASK_STATUS_DETACHED;

    ret = LOS_TaskCreate(&g_testTskHandle, &osTaskInitParam);
    if (LOS_OK != ret) {
        PRINTF("LosTestInit  error\n");
    }
    return ret;
}

UINT32 LosAppInit(VOID)
{
    UINT32 ret;

    ret = los_TestInit();
    if (ret != LOS_OK) {
        return ret;
    }
    return LOS_OK;
}

#define OS_NVIC_SETPEND 0xE000E200
#define OS_NVIC_CLRPEND 0xE000E280
#define HWI_SHIFT_NUM   5
#define HWI_BIT         2

VOID TestHwiTrigger(UINT32 hwiNum)
{
	HalHwiTrigger(hwiNum);
}

VOID TestHwiUnTrigger(UINT32 hwiNum)
{
}

UINT32 TestHwiDelete(UINT32 hwiNum)
{
    UINT32 ret = HalHwiDelete(hwiNum);
    if (ret != LOS_OK) {
        return LOS_NOK;
    }
    return LOS_OK;
}
VOID TestHwiClear(UINT32 hwiNum) 
{
}

#if (LOSCFG_BASE_IPC_SEM == YES)
UINT32 TestSemDelete(UINT32 semHandle)
{
    return LOS_SemDelete(semHandle);
}
UINT32 g_usSemID;
UINT32 g_usSemID2;
UINT32 g_usSemID3[LOSCFG_BASE_IPC_SEM_LIMIT + 1];
#endif

