/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ST STR91x ARM9
 * port.
 *----------------------------------------------------------*/

/* Standard includes. */
#include <stdlib.h>
#include <assert.h>

#include "sys_rtos.h"
#include "include.h"
#include "uart_pub.h"
#include "ke_event.h"
#include "rw_pub.h"
#include "fake_clock_pub.h"
#include "power_save_pub.h"
#include "osk_revision.h"

#if (NX_POWERSAVE)
#include "ps.h"
#endif //(NX_POWERSAVE)

#ifndef configUSE_WATCHDOG_TICK
	#error configUSE_WATCHDOG_TICK must be set to either 1 or 0 in FreeRTOSConfig.h to use either the Watchdog or timer 2 to generate the tick interrupt respectively.
#endif

/*-----------------------------------------------------------*/
/* Setup the watchdog to generate the tick interrupts. */
static void prvSetupTimerInterrupt( void );

/* ulCriticalNesting will get set to zero when the first task starts.  It
cannot be initialised to 0 as this will cause interrupts to be enabled
during the kernel initialization process. */
uint32_t ulCriticalNesting = ( uint32_t ) 9999;
void * volatile pxNestChangedCurrentTCB = NULL;
uint32_t volatile g_preempt_delayed_schedule_flag = 0;

/* Tick interrupt routines for cooperative and preemptive operation
respectively.  The preemptive version is not defined as __irq as it is called
from an asm wrapper function. */
#if configUSE_WATCHDOG_TICK == 0
	/* Used to update the OCR timer register */
	static uint16_t s_nPulseLength;
#endif

FUNCPTR func_irda_bg_check = NULL;

void bg_register_irda_check_func(FUNCPTR func)
{
	func_irda_bg_check = func;
}

void bg_unregister_irda_check_func()
{
	func_irda_bg_check = NULL;
}

/*-----------------------------------------------------------*/
void vApplicationIdleHook( void )
{
#if (CFG_TASK_WDG_ENABLED)
        extern void bk_task_wdt_feed(void);
        bk_task_wdt_feed();
#endif
	if(func_irda_bg_check)
	{
		(*func_irda_bg_check)();
	}
}

/*-----------------------------------------------------------*/
/*
 * Initialize the stack of a task to look exactly as if a call to
 * portSAVE_CONTEXT had been called.
 *
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
	StackType_t *pxOriginalTOS;

	pxOriginalTOS = pxTopOfStack;

	/* To ensure asserts in tasks.c don't fail, although in this case the assert
	is not really required. */
	pxTopOfStack--;

	/* Setup the initial stack of the task.  The stack is set exactly as
	expected by the portRESTORE_CONTEXT() macro. */

	/* First on the stack is the return address - which in this case is the
	start of the task.  The offset is added to make the return address appear
	as it would within an IRQ ISR. */
	*pxTopOfStack = ( StackType_t ) pxCode + portINSTRUCTION_SIZE;		
	pxTopOfStack--;

	*pxTopOfStack = ( StackType_t ) 0xaaaaaaaa;	/* R14 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) pxOriginalTOS; /* Stack used when task starts goes in R13. */
	pxTopOfStack--;
	*pxTopOfStack = ( StackType_t ) 0x12121212;	/* R12 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x11111111;	/* R11 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x10101010;	/* R10 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x09090909;	/* R9 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x08080808;	/* R8 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x07070707;	/* R7 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x06060606;	/* R6 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x05050505;	/* R5 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x04040404;	/* R4 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x03030303;	/* R3 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x02020202;	/* R2 */
	pxTopOfStack--;	
	*pxTopOfStack = ( StackType_t ) 0x01010101;	/* R1 */
	pxTopOfStack--;	

	/* When the task starts is will expect to find the function parameter in
	R0. */
	*pxTopOfStack = ( StackType_t ) pvParameters; /* R0 */
	pxTopOfStack--;

	/* The status register is set for system mode, with interrupts enabled. */
	if (( StackType_t ) pxCode & 0x01)
		*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR | 0x20; /*thumb mode*/
	else
		*pxTopOfStack = ( StackType_t ) portINITIAL_SPSR; /*arm mode*/
	pxTopOfStack--;

	/* Interrupt flags cannot always be stored on the stack and will
	instead be stored in a variable, which is then saved as part of the
	tasks context. */
	*pxTopOfStack = portNO_CRITICAL_NESTING;

	return pxTopOfStack;	
}

/*-----------------------------------------------------------*/
extern void vPortStartFirstTask( void );
BaseType_t xPortStartScheduler( void )
{
	os_printf("OSK Rev: %s %s\r\n", BEKEN_OSK_REV, OSK_COMMIT_ID);

	/* Start the timer that generates the tick ISR.  
	   Interrupts are disabled here already. */
	prvSetupTimerInterrupt();

	/* Start the first task. */
	vPortStartFirstTask();	

	/* Should not get here! */
	return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
	/* It is unlikely that the ARM port will require this function as there
	is nothing to return to.  */
}
/*-----------------------------------------------------------*/

/* This function is called from an asm wrapper, so does not require the __irq
keyword. */
/*-----------------------------------------------------------*/

static void prvSetupTimerInterrupt( void )
{
    fclk_init();
}

/*-----------------------------------------------------------*/
void vPortEnterCritical( void )
{
	portDISABLE_INTERRUPTS();

	ulCriticalNesting ++;
}

/*-----------------------------------------------------------*/
void vPortExitCritical( void )
{
	if( ulCriticalNesting > portNO_CRITICAL_NESTING )
	{
		ulCriticalNesting --;

		if( ulCriticalNesting == portNO_CRITICAL_NESTING )
		{
			portENABLE_INTERRUPTS();
		}
	}
}

/*-----------------------------------------------------------*/
uint32_t platform_is_in_irq_enable( void )
{
    uint32_t interrupt;

    __asm volatile(
		"MRS %0,CPSR\n"
		"AND %0,%0,#0xC0\n"
		:"=r" (interrupt)
		:
		:"memory"
	);

    return (!(interrupt & ARM968_IRQ_ENABLE));
}

/*-----------------------------------------------------------*/
uint32_t platform_is_in_fiq_enable( void )
{
    uint32_t interrupt;

    __asm volatile(
		"MRS %0,CPSR\n"
		"AND %0,%0,#0xC0\n"
		:"=r" (interrupt)
		:
		:"memory"
	);

    return (!(interrupt & ARM968_FIQ_ENABLE));
}

/*-----------------------------------------------------------*/
void printf_lr_register(void)
{
    uint32_t value;

    __asm volatile(
		"MOV %0,lr\n"
		:"=r" (value)
		:
		:"memory"
	);

	os_printf("lr:%x\r\n", value);
}

uint32_t platform_is_in_irq_context( void )
{
    uint32_t mode;

    __asm volatile(
		"MRS %0,CPSR\n"
		"AND %0,%0,#0x1f\n"
		:"=r" (mode)
		:
		:"memory"
	);

    return (ARM968_IRQ_MODE == mode);
}

/*-----------------------------------------------------------*/
uint32_t platform_is_in_fiq_context( void )
{
    uint32_t mode;

    __asm volatile(
		"MRS %0,CPSR\n"
		"AND %0,%0,#0x1f\n"
		:"=r" (mode)
		:
		:"memory"
	);

    return (ARM968_FIQ_MODE == mode);
}

/*-----------------------------------------------------------*/
uint32_t platform_entry_system_mode_zero_ptr(void)
{
	uint32_t system_md_value = 0xdf;
	FUNCPTR boot = 0;

    __asm volatile(
		"MSR CPSR,%0\n"
		:"=r" (system_md_value)
		:
		:"memory"
	);

	(*boot)();

	return 0;
}

uint32_t platform_spsr_content( void )
{
    uint32_t mode;

    __asm volatile(
		"MRS %0,SPSR\n"
		:"=r" (mode)
		:
		:"memory"
	);

    return mode;
}

/*-----------------------------------------------------------*/
uint32_t platform_sp_content( void )
{
    uint32_t val;

    __asm volatile(
		"mov %0,SP\n"
		:"=r" (val)
		:
		:"memory"
	);

    return val;
}

/*-----------------------------------------------------------*/
uint32_t platform_cpsr_content( void )
{
    uint32_t mode;

    __asm volatile(
		"MRS %0,CPSR\n"
		:"=r" (mode)
		:
		:"memory"
	);

    return mode;
}

uint32_t platform_interrupt_compare( void )
{
	uint32_t ret = 0;
	uint32_t status = platform_cpsr_content();
	uint32_t sta_irq = ((status & 0x80) >> 7);
	uint32_t sta_fiq = ((status & 0x40) >> 6);

	if(sta_fiq == sta_irq)
	{
		ret = 1;
	}

	return ret;
}

uint32_t platform_is_fiq_disable( void )
{
	uint32_t value = platform_cpsr_content();

	return (value & 0x40);
}

uint32_t platform_is_irq_disable( void )
{
	uint32_t value = platform_cpsr_content();

	return (value & 0x80);
}

uint32_t platform_is_in_interrupt_context( void )
{
    return ((platform_is_in_fiq_context())
                || (platform_is_in_irq_context()));
}

void platform_enable_intc_at_restore_context(void)
{
	if(!platform_is_in_interrupt_context())
		portENABLE_IRQ();

	if(!platform_is_in_fiq_context())
		portENABLE_FIQ();
}

void rtos_stack_overflow(char *taskname)
{
	os_printf("stack overflow: %s\r\n", taskname);
	while(1);
}

uint8_t preempt_delayed_schedule_check(void)
{
	uint32_t previous_mode;
	uint8_t ret = 0;
	GLOBAL_INT_DECLARATION();

	GLOBAL_INT_DISABLE();
	previous_mode = platform_spsr_content() & ARM968_MODE_MASK;
	if (ARM_MODE_SYS != previous_mode) {
		g_preempt_delayed_schedule_flag = 1;
		ret = 1;
	}
	GLOBAL_INT_RESTORE();

	return ret;
}

void preempt_delayed_schedule_handler(void)
{
	uint32_t previous_mode;
	GLOBAL_INT_DECLARATION();

	GLOBAL_INT_DISABLE();
	previous_mode = platform_spsr_content() & ARM968_MODE_MASK;
	if(g_preempt_delayed_schedule_flag
		&& (ARM_MODE_SYS == previous_mode)) {
		g_preempt_delayed_schedule_flag = 0;
		vTaskSwitchContext();
	}

	GLOBAL_INT_RESTORE();
}


#ifdef CONTROL_IRQ_WITH_NORMAL_FUNCTION
/*
 * Enable Interrupts
 */
void port_enable_interrupts_flag(int val)
{
	unsigned long cpsr_val;
	unsigned long mask;

	mask = val & ARM968_IF_MASK;
	cpsr_val = platform_cpsr_content();
	cpsr_val &= (~ARM968_IF_MASK);
	cpsr_val += mask;

	__asm volatile(
	"msr	cpsr_c, %0"
	::"r" (cpsr_val)
	:);
}

void portENABLE_IRQ(void)
{
	unsigned long temp;
	__asm volatile(
	"mrs	%0, cpsr		@ local_irq_enable\n"
       "bic	%0, %0, #0x80\n"
       "msr	cpsr_c, %0"
	: "=r" (temp)
	:
	: "memory");
}

void portENABLE_FIQ(void)
{
	unsigned long temp;
	__asm volatile(
	"mrs	%0, cpsr		@ local_irq_enable\n"
       "bic	%0, %0, #0x40\n"
       "msr	cpsr_c, %0"
	: "=r" (temp)
	:
	: "memory");
}

/*
 * Disable Interrupts
 */
int portDISABLE_FIQ(void)
{
	unsigned long temp;
	unsigned long mask;

	__asm volatile(
	"mrs	%1, cpsr		@ local_irq_disable\n"
	"orr	%0, %1, #0x40\n"
	"msr	cpsr_c, %0"
	: "=r" (temp),"=r" (mask)
	:
	: "memory");

	return (!!(mask & 0x40));
}

int port_disable_interrupts_flag(void)
{
	unsigned long temp;
	unsigned long mask;

	__asm volatile(
	"mrs	%1, cpsr		@ local_irq_disable\n"
	"orr	%0, %1, #0xC0\n"
	"msr	cpsr_c, %0"
	: "=r" (temp),"=r" (mask)
	:
	: "memory");

	return (mask & 0xC0);
}

int portDISABLE_IRQ(void)
{
	unsigned long temp;
	unsigned long mask;

	__asm volatile(
	"mrs	%1, cpsr		@ local_irq_disable\n"
	"orr	%0, %1, #0x80\n"
	"msr	cpsr_c, %0"
	: "=r" (temp),"=r" (mask)
	:
	: "memory");

	return (!!(mask & 0x80));
}
#endif

//eof

