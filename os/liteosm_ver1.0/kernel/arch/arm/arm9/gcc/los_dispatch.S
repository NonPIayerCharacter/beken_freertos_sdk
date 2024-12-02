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
	.equ    ARM_PSR_THUMB,     0x20
	.equ    ARM_PSR_INT_DIS,   0xC0
	.equ    ARM_PSR_FIQ_DIS,   0x40
	.equ    ARM_PSR_IRQ_DIS,   0x80
	.equ    ARM_PSR_MODE_MASK, 0x1F
	.equ    ARM_PSR_MODE_USR,  0x10
	.equ    ARM_PSR_MODE_FIQ,  0x11
	.equ    ARM_PSR_MODE_IRQ,  0x12
	.equ    ARM_PSR_MODE_SVC,  0x13
	.equ    ARM_PSR_MODE_ABT,  0x17
	.equ    ARM_PSR_MODE_UND,  0x1B
	.equ    ARM_PSR_MODE_SYS,  0x1F

	.global _arm_exception_irq_handler
	.global _arm_exception_fiq_handler
	.global HalTaskSchedule
	.global HalStartToRun
	.global HalFiqBreakinIrqGet
	.global HalFiqBreakinIrqSet
	.global HalFiqBreakinIrqClear

	.extern g_sysNeedSched
	.extern g_intCount
	.extern g_fiqBreakIrqFlag

.code 32
.text

HalStartToRun:
    MSR     CPSR_c, #(ARM_PSR_INT_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SVC)

    LDR     R1, =g_losTask
    LDR     R0, [R1, #4]
    LDR     SP, [R0]

    LDMFD   SP!, {R0}
    MSR     SPSR_cxsf, R0

    LDMFD   SP!, {R0-R12, LR, PC}^
			
	.globl HalTaskSchedule
	.type HalTaskSchedule, %function
HalTaskSchedule:
    STMFD   SP!, {LR}
    STMFD   SP!, {LR}
    STMFD   SP!, {R0-R12}

    MRS     R0, CPSR
    MRS     R1, CPSR
    ORR     R1, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS)
    MSR     CPSR, R1

    TST     LR, #1
    ORRNE   R0, R0, #ARM_PSR_THUMB
    STMFD   SP!, {R0}

    LDR     R1, =g_fiqBreakIrqFlag
    LDR     R1, [R1]
    CMP     R1, #1
    BEQ     InterruptContextLoad

    LDR     R1, =g_sysNeedSched
    MOV     R2, #1
    STR     R2, [R1]

    LDR     R0, =g_intCount
    LDR     R0, [R0]
    CMP     R0, #0
    BNE     TaskContextLoad
    STR     R0, [R1]

    BLX     OsSchedTaskSwitch
    CMP     R0, #0
    BNE     NewTaskContextSwitch
    B       TaskContextLoad

_arm_exception_fiq_handler:

	STMFD SP!,{R0}

	MRS R0, spsr
	AND R0, R0, #ARM_PSR_MODE_MASK
	CMP R0, #ARM_PSR_MODE_IRQ
	BEQ _fiq_break_irq

	LDMIA SP!, {R0}

	LDR     SP, =__fiq_stack_top

	SUB     LR, LR, #4
	STMFD   SP!, {R0-R2}

	MOV     R0, SP
	MRS     R1, SPSR
	MOV     R2, LR

	MSR     CPSR_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SVC)
	STMFD   SP!, {R2}
	STMFD   SP!, {LR}
	STMFD   SP!, {R3-R12}
	LDMIA   R0!, {R5-R7}
	STMFD   SP!, {R5-R7}
	STMFD   SP!, {R1}

	LDR     R2, =g_losTask
	LDR     R3, [R2]
	STR     SP, [R3]

	LDR     SP, =__svc_stack_top

	BLX     HalInterrupt_fiq

	LDR     R0, =g_losTaskLock
	LDR     R0, [R0]
	CMP     R0, #0
	BNE     NoTaskContextSwitch

	LDR     R0, =g_sysNeedSched
	LDR     R0, [R0]
	CMP     R0, #0
	BNE     NeedTaskSwitch
	B       NoTaskContextSwitch
	
_fiq_break_irq:		
	LDMIA SP!, {R0}
	STMFD	SP!, {r0-r7,LR}
	NOP
	NOP

	BLX     HalFiqBreakinIrqSet
	BLX     HalInterrupt_fiq
	BLX     HalFiqBreakinIrqClear
	
	LDMIA	SP!, {r0-r7,LR}
	NOP
	NOP

	SUBS    pc, LR, #0x4

_arm_exception_irq_handler:
	LDR     SP, =__irq_stack_top

	SUB     LR, LR, #4
	STMFD   SP!, {R0-R2}

	MOV     R0, SP
	MRS     R1, SPSR
	MOV     R2, LR

	MSR     CPSR_c, #(ARM_PSR_IRQ_DIS | ARM_PSR_FIQ_DIS | ARM_PSR_MODE_SVC)
	STMFD   SP!, {R2}
	STMFD   SP!, {LR}
	STMFD   SP!, {R3-R12}
	LDMFD   R0!, {R5-R7}
	STMFD   SP!, {R5-R7}
	STMFD   SP!, {R1}

	LDR     R2, =g_losTask
	LDR     R3, [R2]
	STR     SP, [R3]

	LDR     SP, =__svc_stack_top

	BLX     HalInterrupt_irq

	LDR     R0, =g_losTaskLock
	LDR     R0, [R0]
	CMP     R0, #0
	BNE     NoTaskContextSwitch

	LDR     R0, =g_sysNeedSched
	LDR     R0, [R0]
	CMP     R0, #0
	BNE     NeedTaskSwitch
	B       NoTaskContextSwitch

NeedTaskSwitch:
	LDR     R0, =g_sysNeedSched
	MOV     R1, #0
	STR     R1, [R0]
	BLX     OsSchedTaskSwitch
	CMP     R0, #0
	BEQ     NoTaskContextSwitch
	LDR     R0, =g_losTask
	LDR     R1, [R0, #4]
	STR     R1, [R0]
	LDR     SP, [R1]
	B       TaskContextLoad

NewTaskContextSwitch:
	LDR     R0, =g_losTask
	LDR     R1, [R0]
	STR     SP, [R1]

	LDR     R1, [R0, #4]
	STR     R1, [R0]
	LDR     SP, [R1]
	B       TaskContextLoad

NoTaskContextSwitch:
	LDR     R1, =g_losTask
	LDR     R1, [R1]
	LDR     SP, [R1]

TaskContextLoad:
	LDMFD   SP!, {R0}
	MSR     SPSR_cxsf, R0
	LDMFD   SP!, {R0-R12, LR, PC}^

InterruptContextLoad:
	LDMFD   SP!, {R0}
	LDMFD   SP!, {R0-R12, LR, PC}

