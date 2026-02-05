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

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */
#include "include.h"
#include "mem_pub.h"

#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#if CFG_MEM_DEBUG
#include "doubly_list.h"
#include "str_pub.h"
#include "fake_clock_pub.h"
#endif

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( xHeapStructSize << 1 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE    ( ( size_t ) ( xHeapStructSize << 1 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE         ( ( size_t ) 8 )

/* Max value that fits in a size_t type. */
#define heapSIZE_MAX              ( ~( ( size_t ) 0 ) )
/* Check if multiplying a and b will result in overflow. */
#define heapMULTIPLY_WILL_OVERFLOW( a, b )     ( ( ( a ) > 0 ) && ( ( b ) > ( heapSIZE_MAX / ( a ) ) ) )

/* Check if adding a and b will result in overflow. */
#define heapADD_WILL_OVERFLOW( a, b )          ( ( a ) > ( heapSIZE_MAX - ( b ) ) )

/* Check if the subtraction operation ( a - b ) will result in underflow. */
#define heapSUBTRACT_WILL_UNDERFLOW( a, b )    ( ( a ) < ( b ) )

/* MSB of the xBlockSize member of an BlockLink_t structure is used to track
 * the allocation status of a block.  When MSB of the xBlockSize member of
 * an BlockLink_t structure is set then the block belongs to the application.
 * When the bit is free the block is still part of the free heap space. */
#define heapBLOCK_ALLOCATED_BITMASK    ( ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 ) )
#define heapBLOCK_SIZE_IS_VALID( xBlockSize )    ( ( ( xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) == 0 )
#define heapBLOCK_IS_ALLOCATED( pxBlock )        ( ( ( pxBlock->xBlockSize ) & heapBLOCK_ALLOCATED_BITMASK ) != 0 )
#define heapALLOCATE_BLOCK( pxBlock )            ( ( pxBlock->xBlockSize ) |= heapBLOCK_ALLOCATED_BITMASK )
#define heapFREE_BLOCK( pxBlock )                ( ( pxBlock->xBlockSize ) &= ~heapBLOCK_ALLOCATED_BITMASK )

#define heapPROTECT_BLOCK_POINTER( pxBlock )    ( pxBlock )

#define heapVALIDATE_BLOCK_POINTER( pxBlock )                          \
    configASSERT( ( ( uint8_t * ) ( pxBlock ) >= &( ucHeap[ 0 ] ) ) && \
                  ( ( uint8_t * ) ( pxBlock ) <= &( ucHeap[ configTOTAL_HEAP_SIZE - 1 ] ) ) )

/* Allocate the memory for the heap. */
#if configDYNAMIC_HEAP_SIZE
uint8_t *ucHeap;
#elif( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* The application writer has already defined the array used for the RTOS
	heap - probably so it can be placed in a special segment or address. */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct A_BLOCK_LINK
{
	struct A_BLOCK_LINK *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
#if CFG_MEM_DEBUG
	struct list_head node;					/*<< linked to xUsed */
	unsigned int leakTime;					/*<< the leak time (*1sec)*/
	char funcName[16];						/*<< the function name */
	unsigned int line;						/*<< the function line */
	int wantedSize;							/*<< malloc size */
#endif
} BlockLink_t;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert );

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void prvHeapInit( void );
extern void bk_printf(const char *fmt, ...);

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const size_t xHeapStructSize	= ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

/* Create a couple of list links to mark the start and end of the list. */
static BlockLink_t xStart, *pxEnd = NULL;
#if CFG_MEM_DEBUG
static struct list_head xUsed;
#endif

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining = 0U;
static size_t xMinimumEverFreeBytesRemaining = 0U;

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
member of an BlockLink_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static size_t xBlockAllocatedBit = 0;

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
uint8_t *psram_ucHeap;
/* Create a couple of list links to mark the start and end of the list. */
static BlockLink_t psram_xStart, *psram_pxEnd = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t psram_xFreeBytesRemaining = 0U;
static size_t psram_xMinimumEverFreeBytesRemaining = 0U;

#if (CFG_SOC_NAME == SOC_BK7221U)
#define PSRAM_START_ADDRESS    (void*)(0x00900000)
#define PSRAM_END_ADDRESS      (void*)(0x00900000 + 256 * 1024)
#elif(CFG_SOC_NAME == SOC_BK7252N)
#define PSRAM_START_ADDRESS    (void*)(0x02000000)
#define PSRAM_END_ADDRESS      (void*)(0x02000000 + 256 * 1024)
#endif
/*-----------------------------------------------------------*/

static void psram_prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &psram_xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != psram_pxEnd )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = psram_pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's pxNextFreeBlock pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}

/*-----------------------------------------------------------*/

static void *psram_malloc_without_lock( size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( psram_pxEnd == NULL )
		{
			prvHeapInit();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top bit is
		set.  The top bit of the block size member of the BlockLink_t structure
		is used to determine who owns the block - the application or the
		kernel, so it must be free. */
		if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
		{
			/* The wanted size is increased so it can contain a BlockLink_t
			structure in addition to the requested amount of bytes. */
			if( xWantedSize > 0 )
			{
				xWantedSize += xHeapStructSize;

				/* Ensure that blocks are always aligned to the required number
				of bytes. */
				if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
				{
					/* Byte alignment required. */
					xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
					configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= psram_xFreeBytesRemaining ) )
			{
				/* Traverse the list from the start	(lowest address) block until
				one	of adequate size is found. */
				pxPreviousBlock = &psram_xStart;
				pxBlock = psram_xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				was	not found. */
				if( pxBlock != psram_pxEnd )
				{
					/* Return the memory space pointed to - jumping over the
					BlockLink_t structure at its start. */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

					/* This block is being returned for use so must be taken out
					of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					two. */
					if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
					{
						/* This block is to be split into two.  Create a new
						block following the number of bytes requested. The void
						cast is used to prevent byte alignment warnings from the
						compiler. */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

						/* Calculate the sizes of two blocks split from the
						single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						psram_prvInsertBlockIntoFreeList( pxNewBlockLink );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					psram_xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( psram_xFreeBytesRemaining < psram_xMinimumEverFreeBytesRemaining )
					{
						psram_xMinimumEverFreeBytesRemaining = psram_xFreeBytesRemaining;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					by the application and has no "next" block. */
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}

#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
void *psram_malloc_cm(const char *call_func_name, int line, size_t xWantedSize, int need_zero )
#else
void *psram_malloc( size_t xWantedSize )
#endif
{
	void *pvReturn = NULL;

	if (xWantedSize == 0)
		xWantedSize = 4;

	vTaskSuspendAll();
	pvReturn = psram_malloc_without_lock(xWantedSize);
	#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
	{
	if(pvReturn && call_func_name) {
	BlockLink_t *pxLink = (BlockLink_t *)((u8*)pvReturn - xHeapStructSize);
	bk_printf("\r\nm:%p,%d|%s,%d\r\n", pxLink, (pxLink->xBlockSize & ~xBlockAllocatedBit), call_func_name, line);
	}
	}
	#endif
	( void ) xTaskResumeAll();

	#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
	if(pvReturn && need_zero)
		os_memset(pvReturn, 0, xWantedSize);
	#endif
	return pvReturn;
}

/*yhb added 2015-08-13. wzl added:this function has an issue*/
void *psram_realloc( void *pv, size_t xWantedSize )
{
	uint8_t *puc = ( uint8_t * ) pv;
	BlockLink_t *pxLink;
	int presize, datasize;
	void *pvReturn = NULL;
	BlockLink_t *pxIterator, *pxPreviousBlock, *tmp;

	if (pv == NULL)
		return psram_malloc(xWantedSize);

	/* The memory being freed will have an BlockLink_t structure immediately
	before it. */
	puc -= xHeapStructSize;
	/* This casting is to keep the compiler from issuing warnings. */
	pxLink = ( void * ) puc;

	presize = (pxLink->xBlockSize & ~xBlockAllocatedBit);
	datasize = presize - xHeapStructSize;
	if (datasize >= xWantedSize) // have enough memory don't need realloc
		return pv;

	pxLink->xBlockSize &= ~xBlockAllocatedBit;
	vTaskSuspendAll();
	/* Add this block to the list of free blocks. */
	psram_xFreeBytesRemaining += pxLink->xBlockSize;
	psram_prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
	pvReturn = psram_malloc_without_lock(xWantedSize);
	if (pvReturn != NULL) {
		if (pvReturn != pv)
			os_memcpy(pvReturn, pv, datasize);
	} else { // if can't realloc such big memory, we should NOT put pv in free list.
		pxPreviousBlock = &psram_xStart;
		pxIterator = psram_xStart.pxNextFreeBlock;
		while( pxIterator != NULL )
		{
			if (pxIterator > pxLink) {
				break;
			}
			if (pxIterator == pxLink) {// find pxLink at the begin
				if (pxIterator->xBlockSize > presize) {
					tmp = (BlockLink_t *)((uint8_t*)pxLink + presize);
					tmp->xBlockSize = (pxIterator->xBlockSize - presize);
					tmp->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
					pxPreviousBlock->pxNextFreeBlock = tmp;
				} else {
					pxPreviousBlock->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
				}
				goto INSERTED;
			}
			if ((uint8_t*)pxIterator+pxIterator->xBlockSize == (uint8_t*)pxLink + presize) { // find pxLink at the end
				pxIterator->xBlockSize -= presize;
				goto INSERTED;
			}
			if ((uint8_t*)pxIterator+pxIterator->xBlockSize > (uint8_t*)pxLink + presize) { // find pxLink in the middle
				pxPreviousBlock = pxIterator;
				pxIterator = (BlockLink_t *)((uint8_t*)pxLink + presize);
				pxIterator->xBlockSize = ((uint8_t*)pxPreviousBlock+pxPreviousBlock->xBlockSize)-
					((uint8_t*)pxLink + presize);
				tmp = pxPreviousBlock->pxNextFreeBlock;
				pxPreviousBlock->pxNextFreeBlock = pxIterator;
				pxPreviousBlock->xBlockSize = (uint8_t*)pxLink - (uint8_t*)pxPreviousBlock;
				pxIterator->pxNextFreeBlock = tmp;
				goto INSERTED;
			}
			pxPreviousBlock = pxIterator;
			pxIterator = pxIterator->pxNextFreeBlock;
		}

INSERTED:
		pxLink->xBlockSize = presize|xBlockAllocatedBit;;
		pxLink->pxNextFreeBlock = NULL;
		psram_xFreeBytesRemaining -= presize;
	}
	( void ) xTaskResumeAll();

	return pvReturn;
}
#endif

/*-----------------------------------------------------------*/

static void *malloc_without_lock( size_t xWantedSize )
{
BlockLink_t *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( pxEnd == NULL )
		{
			prvHeapInit();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		/* Check the requested block size is not so large that the top bit is
		set.  The top bit of the block size member of the BlockLink_t structure
		is used to determine who owns the block - the application or the
		kernel, so it must be free. */
		if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
		{
			/* The wanted size must be increased so it can contain a BlockLink_t
			structure in addition to the requested amount of bytes. */
			if( ( xWantedSize > 0 ) &&
				( ( xWantedSize + xHeapStructSize ) >  xWantedSize ) ) /* Overflow check */
			{
				xWantedSize += xHeapStructSize;

				/* Ensure that blocks are always aligned. */
				if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
				{
					/* Byte alignment required. Check for overflow. */
					if( ( xWantedSize + ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) ) )
							> xWantedSize )
					{
						xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
						configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
					}
					else
					{
						xWantedSize = 0;
					}
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				xWantedSize = 0;
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
			{
				/* Traverse the list from the start	(lowest address) block until
				one	of adequate size is found. */
				pxPreviousBlock = &xStart;
				pxBlock = xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				was	not found. */
				if( pxBlock != pxEnd )
				{
					/* Return the memory space pointed to - jumping over the
					BlockLink_t structure at its start. */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

					/* This block is being returned for use so must be taken out
					of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					two. */
					if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
					{
						/* This block is to be split into two.  Create a new
						block following the number of bytes requested. The void
						cast is used to prevent byte alignment warnings from the
						compiler. */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
						configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

						/* Calculate the sizes of two blocks split from the
						single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						prvInsertBlockIntoFreeList( pxNewBlockLink );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
					{
						xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					by the application and has no "next" block. */
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;

#if CFG_MEM_DEBUG
					list_add_tail(&pxBlock->node, &xUsed);
#endif

				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
	return pvReturn;
}

#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
void *pvPortMalloc_cm(const char *call_func_name, int line, size_t xWantedSize, int need_zero )
#else
void *pvPortMalloc( size_t xWantedSize )
#endif
{
	void *pvReturn = NULL;

	if (xWantedSize == 0)
		xWantedSize = 4;

	vTaskSuspendAll();
	pvReturn = malloc_without_lock(xWantedSize);
	#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
	{
		BlockLink_t *pxLink = (BlockLink_t *)((u8*)pvReturn - xHeapStructSize);
		if(pvReturn && call_func_name) {
#if OSMALLOC_STATISTICAL
			bk_printf("\r\nm:%p,%d|%s,%d\r\n", pxLink, (pxLink->xBlockSize & ~xBlockAllocatedBit), call_func_name, line);
#endif
		}
#if CFG_MEM_DEBUG
		pxLink->leakTime = fclk_get_second();
		os_strlcpy(pxLink->funcName, call_func_name, sizeof(pxLink->funcName) - 1);
		pxLink->line = line;
		pxLink->wantedSize = xWantedSize;
#endif
	}
	#endif
	( void ) xTaskResumeAll();

	#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
	if(pvReturn && need_zero)
		os_memset(pvReturn, 0, xWantedSize);
	#endif
	return pvReturn;
}

/*-----------------------------------------------------------*/
#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
void *vPortFree_cm(const char *call_func_name, int line, void *pv )
#else
void vPortFree( void *pv )
#endif
{
	uint8_t *puc = ( uint8_t * ) pv;
	BlockLink_t *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an BlockLink_t structure immediately
		before it. */
		puc -= xHeapStructSize;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		/* Check the block is actually allocated. */
		configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
		configASSERT( pxLink->pxNextFreeBlock == NULL );

		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				/* The block is being returned to the heap - it is no longer
				allocated. */
				pxLink->xBlockSize &= ~xBlockAllocatedBit;

				vTaskSuspendAll();
#if OSMALLOC_STATISTICAL
                if (call_func_name)
                {
                    bk_printf("\r\nf:%p,%d|%s,%d\r\n", pxLink, pxLink->xBlockSize, call_func_name, line);
                }
#endif
#if CFG_MEM_DEBUG
				list_del(&pxLink->node);
				pxLink->leakTime = 0;
				pxLink->funcName[0] = 0;
				pxLink->line = 0;
#endif
#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
				if (puc > psram_ucHeap)
                {
                    /* Add this block to the list of psram free blocks. */
                    psram_xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    psram_prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
                }
                else
#endif
				{
					/* Add this block to the list of ram free blocks. */
					xFreeBytesRemaining += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					prvInsertBlockIntoFreeList( ( ( BlockLink_t * ) pxLink ) );
				}
				( void ) xTaskResumeAll();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
	return NULL;
#endif
}

#if OSMALLOC_STATISTICAL || CFG_MEM_DEBUG
void printLeakMem(int leaktime)
{
	BlockLink_t *pxLink;
	UINT32 now = fclk_get_second();

	vTaskSuspendAll();

	list_for_each_entry(pxLink, &xUsed, node) {
		if (now - pxLink->leakTime > leaktime)
			bk_printf("%d,0x%x,%d,%s,%d\r\n", now - pxLink->leakTime,
				(u8*)pxLink + xHeapStructSize, pxLink->wantedSize,
				pxLink->funcName, pxLink->line);
	}

	xTaskResumeAll();
}
#endif

/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
    return xFreeBytesRemaining + psram_xFreeBytesRemaining;
#else
	return xFreeBytesRemaining;
#endif
}
/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( void )
{
#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
    return xMinimumEverFreeBytesRemaining + psram_xMinimumEverFreeBytesRemaining;
#else
	return xMinimumEverFreeBytesRemaining;
#endif
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

#if configDYNAMIC_HEAP_SIZE
extern unsigned char _empty_ram;

#define HEAP_START_ADDRESS    (void*)&_empty_ram
#if (CFG_SOC_NAME == SOC_BK7231N)
#define HEAP_END_ADDRESS      (void*)(0x00400000 + 192 * 1024)
#else
#define HEAP_END_ADDRESS      (void*)(0x00400000 + 256 * 1024)
#endif

static void *prvHeapGetHeaderPointer(void)
{
	return (void *)HEAP_START_ADDRESS;
}

size_t prvHeapGetTotalSize(void)
{
	ASSERT(HEAP_END_ADDRESS > HEAP_START_ADDRESS);
	return (HEAP_END_ADDRESS - HEAP_START_ADDRESS);
}
#endif

static void prvHeapInit( void )
{
	BlockLink_t *pxFirstFreeBlock;
	uint8_t *pucAlignedHeap;
	size_t uxAddress;
	size_t xTotalHeapSize;

	#if configDYNAMIC_HEAP_SIZE
	xTotalHeapSize = prvHeapGetTotalSize();
	ucHeap = prvHeapGetHeaderPointer();

	bk_printf("prvHeapInit-start addr:0x%x, size:%d\r\n", ucHeap, xTotalHeapSize);
	#else
	xTotalHeapSize = configTOTAL_HEAP_SIZE;
	#endif

	/* Ensure the heap starts on a correctly aligned boundary. */
	uxAddress = ( size_t ) ucHeap;

	if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
	{
		uxAddress += ( portBYTE_ALIGNMENT - 1 );
		uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
		xTotalHeapSize -= uxAddress - ( size_t ) ucHeap;
	}

	pucAlignedHeap = ( uint8_t * ) uxAddress;

	/* xStart is used to hold a pointer to the first item in the list of free
	blocks.  The void cast is used to prevent compiler warnings. */
	xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
	xStart.xBlockSize = ( size_t ) 0;

	/* pxEnd is used to mark the end of the list of free blocks and is inserted
	at the end of the heap space. */
	uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
	uxAddress -= xHeapStructSize;
	uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
	pxEnd = ( void * ) uxAddress;
	pxEnd->xBlockSize = 0;
	pxEnd->pxNextFreeBlock = NULL;

	/* To start with there is a single free block that is sized to take up the
	entire heap space, minus the space taken by pxEnd. */
	pxFirstFreeBlock = ( void * ) pucAlignedHeap;
	pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
	pxFirstFreeBlock->pxNextFreeBlock = pxEnd;

	/* Only one block exists - and it covers the entire usable heap space. */
	xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
	xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7252N))
    xTotalHeapSize = PSRAM_END_ADDRESS - PSRAM_START_ADDRESS;
    psram_ucHeap = PSRAM_START_ADDRESS;

    bk_printf("prvHeapInit-start addr:0x%x, size:%d\r\n", psram_ucHeap, xTotalHeapSize);

    /* Ensure the heap starts on a correctly aligned boundary. */
    uxAddress = ( size_t ) psram_ucHeap;

    if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        uxAddress += ( portBYTE_ALIGNMENT - 1 );
        uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
        xTotalHeapSize -= uxAddress - ( size_t ) psram_ucHeap;
    }

    pucAlignedHeap = ( uint8_t * ) uxAddress;

    /* psram_xStart is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
    psram_xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    psram_xStart.xBlockSize = ( size_t ) 0;

    /* psram_pxEnd is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
    uxAddress = ( ( size_t ) pucAlignedHeap ) + xTotalHeapSize;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
    psram_pxEnd = ( void * ) uxAddress;
    psram_pxEnd->xBlockSize = 0;
    psram_pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by psram_pxEnd. */
    pxFirstFreeBlock = ( void * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = psram_pxEnd;

    /* Only one block exists - and it covers the entire usable heap space. */
    psram_xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    psram_xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
#endif

#if CFG_MEM_DEBUG
	INIT_LIST_HEAD(&xUsed);
#endif

	/* Work out the position of the top bit in a size_t variable. */
	xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( BlockLink_t *pxBlockToInsert )
{
BlockLink_t *pxIterator;
uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != pxEnd )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's pxNextFreeBlock pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}

/*
 * pvPortRealloc - Reallocate memory block size
 *
 * Description: Resize an allocated memory block, attempting to expand or shrink
 * the block in place. If in-place resize is not possible, allocate a new block
 * and copy the data.
 *
 * Parameters:
 *   pv          - Pointer to the previously allocated memory block
 *   xWantedSize - New requested size of user data (in bytes)
 *
 * Return Value:
 *   On success: Pointer to the new memory block (may be the same as original)
 *   On failure: NULL
 *
 * Behavior:
 *   1) If pv == NULL, behaves like pvPortMalloc(xWantedSize).
 *   2) If xWantedSize == 0, behaves like vPortFree(pv) and returns NULL.
 *   3) Align the requested size and include the block header size; if the aligned
 *      size is invalid, return NULL.
 *   4) If the aligned requested size is <= current block size, shrink in place and
 *      insert any sufficiently large remainder as a free block.
 *   5) If expansion is required and there are enough free bytes in the heap, try to
 *      expand into adjacent free blocks in this order:
 *        - Merge with next free block if it is immediately after the current block.
 *        - Merge with previous free block if it is immediately before the current block.
 *        - Merge with both previous and next if combined they provide enough space.
 *      If none of the above succeed, fall back to allocating a new block, memcpy'ing
 *      the payload and freeing the old block.
 */
void* pvPortRealloc(void* pv,
	size_t xWantedSize)
{
	BlockLink_t* pxBlock;
	BlockLink_t* pxNewBlockLink;
	BlockLink_t* pxNextFreeBlock;
	BlockLink_t* pxPreviousFreeBlock;
	BlockLink_t* pxBeforePreviousFreeBlock;
	uint8_t* puc;
	void* pvReturn = NULL;
	size_t xAlignedWantedSize;
	size_t xAdditionalRequiredSize;
	size_t xCurrentBlockSize;
	size_t xRemainingBlockSize;
	size_t xNextBlockSize;
	size_t xPreviousBlockSize;
	BaseType_t xHasNextBlock;
	BaseType_t xHasPreviousBlock;

	/* Ensure the end marker has been set up. */
	configASSERT(pxEnd);

	/* If pv is NULL behave like malloc. */
	if(pv == NULL)
	{
		pvReturn = pvPortMalloc(xWantedSize);
		goto realloc_exit;
	}

	/* If requested size is zero behave like free. */
	if(xWantedSize == 0)
	{
		vPortFree(pv);
		pvReturn = NULL;
		goto realloc_exit;
	}

	/* Calculate the internal aligned size including the header. */
	xAlignedWantedSize = xWantedSize;

	/* Add the header size and check for overflow. */
	if(heapADD_WILL_OVERFLOW(xAlignedWantedSize, xHeapStructSize) == 0)
	{
		xAlignedWantedSize += xHeapStructSize;

		/* Ensure byte alignment. */
		if((xAlignedWantedSize & portBYTE_ALIGNMENT_MASK) != 0x00)
		{
			xAdditionalRequiredSize = portBYTE_ALIGNMENT - (xAlignedWantedSize & portBYTE_ALIGNMENT_MASK);

			if(heapADD_WILL_OVERFLOW(xAlignedWantedSize, xAdditionalRequiredSize) == 0)
			{
				xAlignedWantedSize += xAdditionalRequiredSize;
			}
			else
			{
				/* Overflow -> invalid request. */
				xAlignedWantedSize = 0;
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	else
	{
		xAlignedWantedSize = 0;
	}

	/* Validate the aligned size. */
	if((xAlignedWantedSize == 0) || (heapBLOCK_SIZE_IS_VALID(xAlignedWantedSize) == 0))
	{
		pvReturn = NULL;
		goto realloc_exit;
	}

	/* Get the block header for the allocated block. */
	puc = (uint8_t*)pv;
	puc -= xHeapStructSize;
	pxBlock = (BlockLink_t*)puc;

	heapVALIDATE_BLOCK_POINTER(pxBlock);
	configASSERT(heapBLOCK_IS_ALLOCATED(pxBlock));

	/* Current block size without the allocated bit. */
	xCurrentBlockSize = pxBlock->xBlockSize & ~heapBLOCK_ALLOCATED_BITMASK;

	/* 1) Shrink in place if possible. */
	if(xAlignedWantedSize <= xCurrentBlockSize)
	{
		xRemainingBlockSize = xCurrentBlockSize - xAlignedWantedSize;

		/* Only split if the remaining space is large enough to form a free block. */
		if(xRemainingBlockSize > heapMINIMUM_BLOCK_SIZE)
		{
			vTaskSuspendAll();
			{
				/* Set the block to the new size and mark as allocated. */
				pxBlock->xBlockSize = xAlignedWantedSize;
				heapALLOCATE_BLOCK(pxBlock);

				/* Create a new free block from the remainder and insert it. */
				pxNewBlockLink = (BlockLink_t*)(((uint8_t*)pxBlock) + xAlignedWantedSize);
				configASSERT((((size_t)pxNewBlockLink) & portBYTE_ALIGNMENT_MASK) == 0);

				pxNewBlockLink->xBlockSize = xRemainingBlockSize;
				xFreeBytesRemaining += xRemainingBlockSize;
				prvInsertBlockIntoFreeList(pxNewBlockLink);
			}
			(void)xTaskResumeAll();
		}
		else
		{
			/* Remainder too small to split. */
			mtCOVERAGE_TEST_MARKER();
		}
		pvReturn = pv;
		goto realloc_exit;
	}
	/* 2) Expansion path: try to use adjacent free blocks if overall free bytes suffice. */
	else if((xAlignedWantedSize - xCurrentBlockSize) <= xFreeBytesRemaining)
	{
		vTaskSuspendAll();
		{
			/* Walk the free list to find the free blocks immediately before and after pxBlock. */
			pxBeforePreviousFreeBlock = &xStart;
			pxPreviousFreeBlock = &xStart;
			pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(xStart.pxNextFreeBlock);
			heapVALIDATE_BLOCK_POINTER(pxNextFreeBlock);

			while((pxNextFreeBlock < pxBlock) && (pxNextFreeBlock->pxNextFreeBlock != heapPROTECT_BLOCK_POINTER(NULL)))
			{
				pxBeforePreviousFreeBlock = pxPreviousFreeBlock;
				pxPreviousFreeBlock = pxNextFreeBlock;
				pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(pxNextFreeBlock->pxNextFreeBlock);
				heapVALIDATE_BLOCK_POINTER(pxNextFreeBlock);
			}

			/* Check if next is immediately after current. */
			if((pxNextFreeBlock != pxEnd) &&
				(((size_t)pxBlock + xCurrentBlockSize) == (size_t)pxNextFreeBlock))
			{
				xHasNextBlock = pdTRUE;
			}
			else
			{
				xHasNextBlock = pdFALSE;
			}

			/* Check if previous is immediately before current. */
			if((pxPreviousFreeBlock != &xStart) &&
				(((size_t)pxPreviousFreeBlock + pxPreviousFreeBlock->xBlockSize) == (size_t)pxBlock))
			{
				xHasPreviousBlock = pdTRUE;
			}
			else
			{
				xHasPreviousBlock = pdFALSE;
			}

			/* Compute required extra size and neighbor sizes. */
			xRemainingBlockSize = xAlignedWantedSize - xCurrentBlockSize;
			xNextBlockSize = pxNextFreeBlock->xBlockSize;
			xPreviousBlockSize = pxPreviousFreeBlock->xBlockSize;
			configASSERT(heapBLOCK_SIZE_IS_VALID(xNextBlockSize) != 0);
			configASSERT(heapBLOCK_SIZE_IS_VALID(xPreviousBlockSize) != 0);

			/* a) If next exists and is large enough, merge with next. */
			if((xHasNextBlock == pdTRUE) &&
				(xNextBlockSize >= xRemainingBlockSize))
			{
				/* Remove next from free list and update free bytes. */
				pxPreviousFreeBlock->pxNextFreeBlock = pxNextFreeBlock->pxNextFreeBlock;
				pxNextFreeBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(NULL);
				xFreeBytesRemaining -= xNextBlockSize;

				/* Temporarily free the current block for merging. */
				heapFREE_BLOCK(pxBlock);

				/* Remaining bytes after creating the requested size. */
				xRemainingBlockSize = xCurrentBlockSize + xNextBlockSize - xAlignedWantedSize;

				if(xRemainingBlockSize > heapMINIMUM_BLOCK_SIZE)
				{
					/* Set block to requested size and insert leftover as a free block. */
					pxBlock->xBlockSize = xAlignedWantedSize;

					pxNewBlockLink = (BlockLink_t*)(((uint8_t*)pxBlock) + xAlignedWantedSize);
					configASSERT((((size_t)pxNewBlockLink) & portBYTE_ALIGNMENT_MASK) == 0);

					pxNewBlockLink->xBlockSize = xRemainingBlockSize;
					xFreeBytesRemaining += xRemainingBlockSize;
					prvInsertBlockIntoFreeList(pxNewBlockLink);
				}
				else
				{
					/* Leftover too small, keep as part of allocated block. */
					pxBlock->xBlockSize = xCurrentBlockSize + xNextBlockSize;
				}

				/* Mark merged block as allocated. */
				heapALLOCATE_BLOCK(pxBlock);
				pvReturn = pv;
			}
			/* b) If previous exists and is large enough, merge with previous (data must be moved). */
			else if((xHasPreviousBlock == pdTRUE) &&
				(xPreviousBlockSize >= xRemainingBlockSize))
			{
				/* Remove previous from free list and update free bytes. */
				pxBeforePreviousFreeBlock->pxNextFreeBlock = pxPreviousFreeBlock->pxNextFreeBlock;
				pxPreviousFreeBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(NULL);
				xFreeBytesRemaining -= xPreviousBlockSize;

				heapFREE_BLOCK(pxBlock);

				/* Move the payload forward into the previous block's payload area. */
				puc = (uint8_t*)pxPreviousFreeBlock;
				puc += xHeapStructSize;
				/* Ensure memmove length will not underflow. */
				configASSERT(heapSUBTRACT_WILL_UNDERFLOW(xCurrentBlockSize, xHeapStructSize) == 0);
				(void)memmove(puc, pv, xCurrentBlockSize - xHeapStructSize);

				/* Remaining bytes after creating the requested size. */
				xRemainingBlockSize = xCurrentBlockSize + xPreviousBlockSize - xAlignedWantedSize;

				if(xRemainingBlockSize > heapMINIMUM_BLOCK_SIZE)
				{
					/* previous becomes the allocated block of requested size, insert leftover. */
					pxPreviousFreeBlock->xBlockSize = xAlignedWantedSize;

					pxNewBlockLink = (BlockLink_t*)(((uint8_t*)pxPreviousFreeBlock) + xAlignedWantedSize);
					configASSERT((((size_t)pxNewBlockLink) & portBYTE_ALIGNMENT_MASK) == 0);

					pxNewBlockLink->xBlockSize = xRemainingBlockSize;
					xFreeBytesRemaining += xRemainingBlockSize;
					prvInsertBlockIntoFreeList(pxNewBlockLink);
				}
				else
				{
					/* Leftover too small, treat entire previous+current as allocated. */
					pxPreviousFreeBlock->xBlockSize = xCurrentBlockSize + xPreviousBlockSize;
				}

				heapALLOCATE_BLOCK(pxPreviousFreeBlock);
				/* Return the payload pointer in the previous block. */
				pvReturn = (void*)puc;
			}
			/* c) If both neighbors exist and combined are large enough, merge both sides (move data). */
			else if((xHasNextBlock == pdTRUE) &&
				(xHasPreviousBlock == pdTRUE) &&
				((xNextBlockSize + xPreviousBlockSize) >= xRemainingBlockSize))
			{
				/* Remove both previous and next from the free list and update free bytes. */
				pxBeforePreviousFreeBlock->pxNextFreeBlock = pxNextFreeBlock->pxNextFreeBlock;
				pxNextFreeBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(NULL);
				pxPreviousFreeBlock->pxNextFreeBlock = heapPROTECT_BLOCK_POINTER(NULL);
				xFreeBytesRemaining -= xNextBlockSize + xPreviousBlockSize;

				heapFREE_BLOCK(pxBlock);

				/* Move payload forward into previous block's payload area. */
				puc = (uint8_t*)pxPreviousFreeBlock;
				puc += xHeapStructSize;
				configASSERT(heapSUBTRACT_WILL_UNDERFLOW(xCurrentBlockSize, xHeapStructSize) == 0);
				(void)memmove(puc, pv, xCurrentBlockSize - xHeapStructSize);

				/* Remaining bytes after allocation. */
				xRemainingBlockSize = xCurrentBlockSize + xNextBlockSize + xPreviousBlockSize - xAlignedWantedSize;

				if(xRemainingBlockSize > heapMINIMUM_BLOCK_SIZE)
				{
					pxPreviousFreeBlock->xBlockSize = xAlignedWantedSize;

					pxNewBlockLink = (BlockLink_t*)(((uint8_t*)pxPreviousFreeBlock) + xAlignedWantedSize);
					configASSERT((((size_t)pxNewBlockLink) & portBYTE_ALIGNMENT_MASK) == 0);

					pxNewBlockLink->xBlockSize = xRemainingBlockSize;
					xFreeBytesRemaining += xRemainingBlockSize;
					prvInsertBlockIntoFreeList(pxNewBlockLink);
				}
				else
				{
					pxPreviousFreeBlock->xBlockSize = xCurrentBlockSize + xNextBlockSize + xPreviousBlockSize;
				}

				heapALLOCATE_BLOCK(pxPreviousFreeBlock);
				pvReturn = (void*)puc;
			}
			else
			{
				/* None of the merge strategies worked on this path. */
				mtCOVERAGE_TEST_MARKER();
			}

			/* Update historical minimum free bytes. */
			if(xFreeBytesRemaining < xMinimumEverFreeBytesRemaining)
			{
				xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		(void)xTaskResumeAll();
	}
	else
	{
		/* Not enough free bytes in the entire heap to satisfy expansion. */
		pvReturn = NULL;
		goto realloc_exit;
	}

	/* If still NULL, fall back to allocating a new block and copying the payload. */
	if(pvReturn == NULL)
	{
		puc = pvPortMalloc(xWantedSize);

		if(puc != NULL)
		{
			/* Copy the old payload (old payload size = xCurrentBlockSize - xHeapStructSize). */
			configASSERT(heapSUBTRACT_WILL_UNDERFLOW(xCurrentBlockSize, xHeapStructSize) == 0);
			(void)memcpy(puc, pv, xCurrentBlockSize - xHeapStructSize);
			vPortFree(pv);

			pvReturn = (void*)puc;
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}

realloc_exit:
	/* Ensure returned pointer is properly aligned (NULL also satisfies this). */
	configASSERT(((size_t)pvReturn & (size_t)portBYTE_ALIGNMENT_MASK) == 0);
	return pvReturn;
}
