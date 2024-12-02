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

    .equ CPSR_IRQ_DISABLE, 0x80U
    .equ CPSR_FIQ_DISABLE, 0x40U
    .equ CPSR_THUMB_ENABLE, 0x20U
    .equ CPSR_USER_MODE, 0x10U
    .equ CPSR_FIQ_MODE, 0x11U
    .equ CPSR_IRQ_MODE, 0x12U
    .equ CPSR_SVC_MODE, 0x13U
    .equ CPSR_ABT_MODE, 0x17U
    .equ CPSR_UNDEF_MODE, 0x1BU
    .equ EXC_STACK_INIT, 0xCACACACA
    .equ EXC_MAGIC_WORD, 0xCCCCCCCC

    .global __exc_stack_top
    .global __irq_stack_top
    .global __fiq_stack_top
    .global __svc_stack_top
    .global __abt_stack_top
    .global __undef_stack_top
    .global __exc_stack
    .global __irq_stack
    .global __fiq_stack
    .global __svc_stack
    .global __abt_stack
    .global __undef_stack
    .global main

    .extern _osExceptFiqHdl
    .extern _osExceptAddrAbortHdl
    .extern _osExceptDataAbortHdl
    .extern _osExceptPrefetchAbortHdl
    .extern _osExceptSwiHdl
    .extern _osExceptUndefInstrHdl
    .extern _osExceptIrqHdl
    .extern _bss_start
    .extern _bss_end

    .section ".vectors", "ax"
    .global _vector_start

_vector_start:
    B     _reset_vector
    B     _osExceptUndefInstrHdl
    B     _osExceptSwiHdl
    B     _osExceptPrefetchAbortHdl
    B     _osExceptDataAbortHdl
    B     _osExceptAddrAbortHdl
    B     _osExceptIrqHdl
    B     _osExceptFiqHdl


    .globl  _reset_vector
    .section ".boot", "ax"

_reset_vector:
    MOV    R0, #(CPSR_IRQ_DISABLE | CPSR_FIQ_DISABLE | CPSR_IRQ_MODE)
    MSR    CPSR, R0
    LDR    SP, =__irq_stack_top

    MOV    R0, #(CPSR_IRQ_DISABLE | CPSR_FIQ_DISABLE | CPSR_UNDEF_MODE)
    MSR    CPSR, R0
    LDR    SP, =__undef_stack_top

    MOV    R0, #(CPSR_IRQ_DISABLE | CPSR_FIQ_DISABLE | CPSR_ABT_MODE)
    MSR    CPSR, R0
    LDR    SP, =__abt_stack_top

    MOV    R0, #(CPSR_IRQ_DISABLE | CPSR_FIQ_DISABLE | CPSR_FIQ_MODE)
    MSR    CPSR, R0
    LDR    SP, =__fiq_stack_top

    MOV    R0, #(CPSR_IRQ_DISABLE | CPSR_FIQ_DISABLE | CPSR_SVC_MODE)
    MSR    CPSR, R0
    MSR    SPSR, R0
    LDR    SP, =__svc_stack_top

    BL     bss_init

    B      main
    B      .

bss_init:
    LDR     R0, =_bss_start
    LDR     R1, =_bss_end

    MOV R3, R1
    MOV R4, R0
    MOV R2, #0
1: CMP R4, R3
    STRLO R2, [R4], #4
    BLO 1b
    BX LR

    .section ".bss", "wa", %nobits
    .align 3
__undef_stack:
    .space  32
__undef_stack_top:

__abt_stack:
    .space  32
__abt_stack_top:

__irq_stack:
    .space  64
__irq_stack_top:

__fiq_stack:
    .space  64
__fiq_stack_top:

__svc_stack:
    .space  4096
__svc_stack_top:

__exc_stack:
    .space  512
__exc_stack_top:
