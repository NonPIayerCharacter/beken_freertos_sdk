/**
 ****************************************************************************************
 *
 * @file boot_handlers.s
 *
 * @brief ARM Exception Vector handler functions.
 *
 * Copyright (C) RivieraWaves 2011-2016
 *
 ****************************************************************************************
 */	
	.globl entry_main
	.globl boot_reset
	.globl boot_swi
	.globl boot_undefined
	.globl boot_pabort
	.globl boot_dabort
	.globl boot_reserved
	.globl irq_handler
	.globl fiq_handler
	.globl do_irq
	.globl do_fiq
	.globl do_swi
	.globl do_undefined
	.globl do_pabort
	.globl do_dabort
	.globl do_reserved
	.globl bk_trap_udef
	.globl bk_trap_pabt
	.globl bk_trap_dabt
	.globl bk_trap_resv
	.globl __svc_stack_top
	.globl __irq_stack_top
	.globl __fiq_stack_top
	.extern _arm_exception_irq_handler
	.extern _arm_exception_fiq_handler
	.extern _arm_exception_swi_handler
    
#include "sys_config.h"

/* ========================================================================
 *                                Macros
 * ======================================================================== */
#define _FIQ_STACK_SIZE_                  0x7F0
#define _IRQ_STACK_SIZE_                  0xFF0
#define _SVC_STACK_SIZE_                  0x3F0
#define _SYS_STACK_SIZE_                  0x3F0
#define _UND_STACK_SIZE_                  512

#define ARM_MODE_MASK                     0x1F
#define ARM_MODE_USR                      0x10
#define ARM_MODE_FIQ                      0x11
#define ARM_MODE_IRQ                      0x12
#define ARM_MODE_SVC                      0x13
#define ARM_MODE_ABT                      0x17
#define ARM_MODE_UND                      0x1B
#define ARM_MODE_SYS                      0x1F
#define BOOT_FIQ_IRQ_MASK                 0xC0
#define BOOT_IRQ_MASK                     0x80

#define BOOT_COLOR_UNUSED  	              0xAAAAAAAA      //Pattern to fill UNUSED stack
#define BOOT_COLOR_SVC     	              0xBBBBBBBB      //Pattern to fill SVC stack
#define BOOT_COLOR_IRQ     	              0xCCCCCCCC      //Pattern to fill IRQ stack
#define BOOT_COLOR_FIQ     	              0xDDDDDDDD      //Pattern to fill FIQ stack
#define BOOT_COLOR_SYS     	              0xEEEEEEEE      //Pattern to fill SYS stack

/* ========================================================================
 Context save and restore macro definitions
 * ======================================================================== */
    .macro PUSH_ALL_ARM_REG
        BOOT_CHANGE_MODE  ARM_MODE_SYS ARM_MODE_MASK
        LDR R1, =0x400044           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR

        BOOT_CHANGE_MODE  ARM_MODE_IRQ ARM_MODE_MASK
        LDR R1, =0x400068           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, SPSR
        STMFD   R1!, {R0}           // backup SPSR
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR

        BOOT_CHANGE_MODE  ARM_MODE_FIQ ARM_MODE_MASK
        LDR R1, =0x40008C           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, SPSR
        STMFD   R1!, {R0}           // backup SPSR
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR


        BOOT_CHANGE_MODE  ARM_MODE_ABT ARM_MODE_MASK
        LDR R1, =0x4000B0           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, SPSR
        STMFD   R1!, {R0}           // backup SPSR
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR


        BOOT_CHANGE_MODE  ARM_MODE_UND ARM_MODE_MASK
        LDR R1, =0x4000D4           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, SPSR
        STMFD   R1!, {R0}           // backup SPSR
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR


        BOOT_CHANGE_MODE  ARM_MODE_SVC ARM_MODE_MASK
        LDR R1, =0x4000F8           // get backup top.
        STMFD   R1!, {R8-R14}       // backup R8-R14
        MRS   R0, SPSR
        STMFD   R1!, {R0}           // backup SPSR
        MRS   R0, CPSR
        STMFD   R1!, {R0}           // backup CPSR
    .endm

/* ========================================================================
 * Push SVC reg
 */
    .macro PUSH_SVC_REG
        SUB     SP, SP, #18 * 4
        STMIA   SP, {R0 - R12}
        MOV     R0, SP
        MRS     R6, SPSR
        STR     LR, [R0, #15*4]
        STR     R6, [R0, #16*4]
        STR     SP, [R0, #13*4]
        STR     LR, [R0, #14*4]
        MRS     R6, CPSR
        STR     R6, [R0, #17*4]
    .endm
    
/* ========================================================================
 * Macro for switching ARM mode
 */
	.macro	BOOT_CHANGE_MODE, mode, mode_mask
	MRS   R0, CPSR
	BIC   R0, R0, #\mode_mask
	ORR   R0, R0, #\mode
	MSR   CPSR_c, R0
	.endm

/* ========================================================================
 * Macro for setting the stack
 */
	.macro  BOOT_SET_STACK, stackStart, stackLen, color
	LDR   R0, \stackStart
	LDR   R1, \stackLen

	ADD   R1, R1, R0
	MOV   SP, R1        //Set stack pointer

	LDR   R2, =\color

3:
	CMP   R0, R1        //End of stack?
	STRLT R2, [r0]      //Colorize stack word
	ADDLT R0, R0, #4
	BLT   3b            //branch to previous local label
	.endm

	 .section .bss
	 .align 3
	 .global und_stack_start
 und_stack_start:
 	 .space _UND_STACK_SIZE_

	.align 3
	.global fiq_stack_start
fiq_stack_start:
	 .space _FIQ_STACK_SIZE_
 __fiq_stack_top:

	.align 3
	.global irq_stack_start
 irq_stack_start:
 	 .space _IRQ_STACK_SIZE_
__irq_stack_top:
 
	.align 3
	.global sys_stack_start
sys_stack_start:
	 .space _SYS_STACK_SIZE_

	.align 3
	.global svc_stack_start
svc_stack_start:
	 .space _SVC_STACK_SIZE_
__svc_stack_top:

/* ========================================================================
 *                                Functions
 * ========================================================================
 * Function to handle reset vector
 */
	.globl	boot_reset		 
	.section ".boot", "ax"

boot_reset:
	//Disable IRQ and FIQ before starting anything
	MRS   R0, CPSR
	ORR   R0, R0, #0xC0
	MSR   CPSR_c, R0
		
	/*Init the BSS section*/
	BL _sysboot_zi_init

	//Setup all stacks //Note: Abt and Usr mode are not used
	BOOT_CHANGE_MODE ARM_MODE_SYS ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_SYS boot_stack_len_SYS BOOT_COLOR_SYS

	BOOT_CHANGE_MODE ARM_MODE_ABT ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_UNUSED boot_stack_len_UNUSED BOOT_COLOR_UNUSED

	BOOT_CHANGE_MODE ARM_MODE_UND ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_UNUSED boot_stack_len_UNUSED BOOT_COLOR_UNUSED

#if (CFG_SOC_NAME == SOC_BK7231N)
	B AFTER_FLAG
	.org  0xc0 //0x100 - sizeof(section.vector) = 0x100 - 0x40
	.word 0x32374B42
	.word 0x00003133

AFTER_FLAG:
#elif (CFG_SOC_NAME == SOC_BK7238)
	B AFTER_FLAG
	.org  0xc0 //0x100 - sizeof(section.vector) = 0x100 - 0x40
	.word 0x32374B42
	.word 0x00003833

AFTER_FLAG:
#endif

	BOOT_CHANGE_MODE ARM_MODE_IRQ ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_IRQ boot_stack_len_IRQ BOOT_COLOR_IRQ

	BOOT_CHANGE_MODE ARM_MODE_FIQ ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_FIQ boot_stack_len_FIQ BOOT_COLOR_FIQ

	//Clear FIQ banked registers while in FIQ mode
	MOV     R8,  #0
	MOV     R9,  #0
	MOV     R10, #0
	MOV     R11, #0
	MOV     R12, #0

	BOOT_CHANGE_MODE ARM_MODE_SVC ARM_MODE_MASK
	BOOT_SET_STACK   boot_stack_base_SVC boot_stack_len_SVC BOOT_COLOR_SVC

	/* Stay in Supervisor Mode
	 * copy data from binary to ram*/
	BL _sysboot_copy_data_to_ram

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
	BL _sysboot_copy_code_to_itcm
#endif

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
	BL _sysboot_tcmbss_init
#endif

	//==================
	//Clear Registers
	MOV R0, #0
	MOV R1, #0
	MOV R2, #0
	MOV R3, #0
	MOV R4, #0
	MOV R5, #0
	MOV R6, #0
	MOV R7, #0
	MOV R8, #0
	MOV R9, #0
	MOV R10, #0
	MOV R11, #0
	MOV R12, #0

	/* start main entry*/
	B entry_main
	B .
		
/* ========================================================================
 *								  Globals
 * ======================================================================== */
boot_stack_base_UNUSED:
	.word und_stack_start

boot_stack_len_UNUSED:
	.word _UND_STACK_SIZE_

boot_stack_base_IRQ:
	.word irq_stack_start

boot_stack_len_IRQ:
	.word _IRQ_STACK_SIZE_

boot_stack_base_SVC:
	.word svc_stack_start

boot_stack_len_SVC:
	 .word _SVC_STACK_SIZE_

boot_stack_base_FIQ:
	 .word fiq_stack_start

boot_stack_len_FIQ:
	.word _FIQ_STACK_SIZE_

boot_stack_base_SYS:
	 .word sys_stack_start

boot_stack_len_SYS:
	 .word _SYS_STACK_SIZE_  

/*FUNCTION:     _sysboot_copy_data_to_ram*/
/*DESCRIPTION:  copy main stack code from FLASH/ROM to SRAM*/
_sysboot_copy_data_to_ram:
    LDR     R0, =_data_flash_begin
    LDR     R1, =_data_ram_begin
    LDR     R2, =_data_ram_end
	
4: CMP R1, R2
    LDRLO   R4, [R0], #4
    STRLO   R4, [R1], #4
    BLO     4b
    BX LR
        
/*FUNCTION:     _sysboot_zi_init*/
/*DESCRIPTION:  Initialise Zero-Init Data Segment*/
_sysboot_zi_init:
    LDR     R0, =_bss_start
    LDR     R1, =_bss_end
    
    MOV R3, R1
    MOV R4, R0
    MOV R2, #0
5: CMP R4, R3
    STRLO R2, [R4], #4
    BLO 5b
    BX LR

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
/*FUNCTION:     _sysboot_copy_code_to_itcm*/
/*DESCRIPTION:  copy itcm code from FLASH/ROM to SRAM*/
_sysboot_copy_code_to_itcm:
    LDR     R0, =_itcmcode_flash_begin
    LDR     R1, =_itcmcode_ram_begin
    LDR     R2, =_itcmcode_ram_end

6: CMP R1, R2
    LDRLO   R4, [R0], #4
    STRLO   R4, [R1], #4
    BLO     6b
    BX LR

/*FUNCTION:     _sysboot_sdbss_init*/
/*DESCRIPTION:  Initialise Zero-Init Data Segment of TCM */
_sysboot_tcmbss_init:
    LDR     R0, =_tcmbss_start
    LDR     R1, =_tcmbss_end

    MOV R3, R1
    MOV R4, R0
    MOV R2, #0
8: CMP R4, R3
    STRLO R2, [R4], #4
    BLO 8b
    BX LR
#endif
		
		.align	5
boot_swi:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400008
	LDR    r0, [R1]
	BX	   r0

		.align	5
irq_handler:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400000
	LDR    r0, [R1]
	BX	   r0

		.align	5
fiq_handler:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400004
	LDR    r0, [R1]
	BX	   r0			

        .align  5
boot_undefined:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x40000c
	LDR    r0, [R1]
	BX     r0

        .align  5
boot_pabort:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400010
	LDR    r0, [R1]
	BX     r0

        .align  5
boot_dabort:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400014
	LDR    r0, [R1]
	BX     r0

        .align  5
boot_reserved:
	STMFD  sp!,{r0-r1}
	LDR    R1, =0x400018
	LDR    r0, [R1]
	BX     r0

		.align	5 
do_undefined:
	LDMFD	SP!, {R0-R1}
	PUSH_SVC_REG

	STMFD  sp!,{r0-r1}
	PUSH_ALL_ARM_REG
	BOOT_CHANGE_MODE  ARM_MODE_UND ARM_MODE_MASK
	LDMFD	SP!, {R0-R1}

	B bk_trap_udef
	
		.align	5 
do_pabort:
	LDMFD	SP!, {R0-R1}
	PUSH_SVC_REG

	STMFD  sp!,{r0-r1}
	PUSH_ALL_ARM_REG
	BOOT_CHANGE_MODE  ARM_MODE_ABT ARM_MODE_MASK
	LDMFD	SP!, {R0-R1}
	
	B bk_trap_pabt
		
	.align	5 
do_dabort:
	LDMFD	SP!, {R0-R1}
	PUSH_SVC_REG

	STMFD  sp!,{r0-r1}
	PUSH_ALL_ARM_REG
	BOOT_CHANGE_MODE  ARM_MODE_ABT ARM_MODE_MASK
	LDMFD	SP!, {R0-R1}
	
	B bk_trap_dabt
		
	.align	5 
do_reserved:
	LDMFD	SP!, {R0-R1}
	PUSH_SVC_REG
	B bk_trap_resv	

	.align	5 
do_swi:
	LDMFD	SP!, {R0-R1}
    B HalExceptSwiHdl
		
	.align	5 
do_irq:
	LDMFD	SP!, {R0-R1}
	B HalExceptIrqHdl
	
	.align	5    
do_fiq:
	LDMFD	SP!, {R0-R1}
	B HalExceptFiqHdl
	
    .code 32
    .global WFI
    .type WFI,%function
WFI:
	MOV R0, #0
	MCR p15, 0, R0, c7, c0, 4
	BX LR	
		
	.code 32
	.globl arm9_enable_alignfault
	.type arm9_enable_alignfault, %function
arm9_enable_alignfault:
	MRC p15, 0, R0, c1, c0, 0
	ORR R0, R0, #0x02
	MCR p15, 0, R0, c1, c0, 0
	BX LR

	.globl arm9_disable_alignfault
	.type arm9_disable_alignfault, %function
arm9_disable_alignfault:
	MRC p15, 0, R0, c1, c0, 0
	BIC R0, R0, #0x02
	MCR p15, 0, R0, c1, c0, 0
	BX LR
/*EOF*/

