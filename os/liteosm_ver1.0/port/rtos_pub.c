#include "include.h"
#include "rtos_pub.h"
#include "los_config.h"
#include "los_context.h"
#include "los_task.h"
#include "los_queue.h"
#include "los_sem.h"
#include "los_mux.h"
#include "los_memory.h"
#include "los_interrupt.h"
#include "los_swtmr.h"
#include "bk_los_timer.h"

/* OS_TASK_PRIORITY_HIGHEST and OS_TASK_PRIORITY_LOWEST is reserved for internal TIMER and IDLE task use only. */
#define ISVALID_LOS_PRIORITY(losPrio) ((losPrio) > OS_TASK_PRIORITY_HIGHEST && (losPrio) < OS_TASK_PRIORITY_LOWEST)
/**
* @brief Enumerates thread states.
*
*/
typedef enum {
  /** The thread is inactive. */
  osThreadInactive        =  0,
  /** The thread is ready. */
  osThreadReady           =  1,
  /** The thread is running. */
  osThreadRunning         =  2,
  /** The thread is blocked. */
  osThreadBlocked         =  3,
  /** The thread is terminated. */
  osThreadTerminated      =  4,
  /** The thread is abnormal. */
  osThreadError           = -1,
  /** Reserved */
  osThreadReserved        = 0x7FFFFFFF
} osThreadState_t;
  
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == 1)
  /**
  * @brief Enumerates timer permissions.
  *
  * @since 1.0
  * @version 1.0
  */
  typedef enum	{
	/** The timer is not allowed to wake up the RTOS. */
	osTimerRousesIgnore 	  = 	0,
	/** The timer is allowed to wake up the RTOS. */
	osTimerRousesAllow		  = 	1
  } osTimerRouses_t;
  
  /**
  * @brief Enumerates timer alignment modes.
  *
  */
  typedef enum	{
	/** The timer ignores alignment. */
	osTimerAlignIgnore		  = 	0,
	/** The timer allows alignment. */
	osTimerAlignAllow		  = 	1
  } osTimerAlign_t;
#endif //(LOSCFG_BASE_CORE_SWTMR_ALIGN == 1)

/******************************************************
 *               Function Declarations
 ******************************************************/
uint32_t ms_to_tick_ratio = LOS_TICK_MS; // wangzhilei
beken_time_t beken_time_offset = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/
OSStatus rtos_create_thread( beken_thread_t *thread, uint8_t priority, const char *name, 
						beken_thread_function_t function, uint32_t stack_size, beken_thread_arg_t arg )
{
    uint32_t uwTid;
    UINT32 uwRet = LOS_OK;
	OSStatus ret;
    LosTaskCB *pstTaskCB = NULL;
    TSK_INIT_PARAM_S stTskInitParam = {NULL};
    UINT16 usPriority;

	ret = kNoErr;
    if (OS_INT_ACTIVE) {
        ret = kGeneralErr;
		goto tinit_exit;
    }

    usPriority = priority;
    if (!ISVALID_LOS_PRIORITY(usPriority)) {
        ret = kUnsupportedErr;/* unsupported priority */
		goto tinit_exit;
    }

    stTskInitParam.pfnTaskEntry = (TSK_ENTRY_FUNC)function;
    stTskInitParam.uwArg = (uint32_t)arg;
    stTskInitParam.uwStackSize = stack_size;
    stTskInitParam.pcName = (CHAR *)name;
    stTskInitParam.usTaskPrio = usPriority;

    uwRet = LOS_TaskCreate((UINT32 *)&uwTid, &stTskInitParam);
	bk_printf("[wzl_debug]LOS_TaskCreate:%s:%d\r\n", name, uwRet);
	if(LOS_OK != uwRet)
	{
		ret = kGeneralErr;
		goto tinit_exit;
	}
    pstTaskCB = OS_TCB_FROM_TID(uwTid);

	if(thread)
	{
		*thread = (beken_thread_t *)pstTaskCB;
	}

tinit_exit:
	ASSERT(LOS_OK == uwRet);
	return ret;
}

OSStatus rtos_delete_thread(beken_thread_t *thread)
{
    UINT32 uwRet;
	uint32_t selfTid;
    LosTaskCB *pstTaskCB = NULL;

    if (OS_INT_ACTIVE) {
        return kUnknownErr;
    }

    if (thread && (*thread == NULL)) {
        return kParamErr;
    }

	if(NULL == thread)
	{
	    selfTid = LOS_CurTaskIDGet();
	    pstTaskCB = OS_TCB_FROM_TID(selfTid);
	}
	else
	{
	    pstTaskCB = (LosTaskCB *)*thread;
	}

    uwRet = LOS_TaskDelete(pstTaskCB->taskID);

    switch (uwRet) {
        case LOS_ERRNO_TSK_OPERATE_IDLE:
        case LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED:
        case LOS_ERRNO_TSK_ID_INVALID:
            return kParamErr;

        case LOS_ERRNO_TSK_NOT_CREATED:
            return kNoResourcesErr;

        default:
            return kNoErr;
    }
}

uint32_t _thread_get_status(beken_thread_t *thread)
{
    UINT16 taskStatus;
    osThreadState_t stState;
    LosTaskCB *pstTaskCB = NULL;

	ASSERT(thread);
    if (OS_INT_ACTIVE || *thread == NULL) {
        return osThreadError;
    }

    pstTaskCB = (LosTaskCB *)*thread;
    taskStatus = pstTaskCB->taskStatus;

    if (taskStatus & OS_TASK_STATUS_RUNNING) {
        stState = osThreadRunning;
    } else if (taskStatus & OS_TASK_STATUS_READY) {
        stState = osThreadReady;
    } else if (taskStatus &
        (OS_TASK_STATUS_DELAY | OS_TASK_STATUS_PEND | OS_TASK_STATUS_SUSPEND)) {
        stState = osThreadBlocked;
    } else if (taskStatus & OS_TASK_STATUS_UNUSED) {
        stState = osThreadInactive;
    } else {
        stState = osThreadError;
    }

    return stState;
}

OSStatus rtos_thread_join(beken_thread_t *thread)
{
	ASSERT(thread);
    while ( _thread_get_status( *thread ) != osThreadInactive )
    {
        rtos_delay_milliseconds(10);
    }
    
    return kNoErr;
}

BOOL rtos_is_current_thread( beken_thread_t *thread )
{
    uint32_t selfTid = LOS_CurTaskIDGet();
    LosTaskCB *tcb = OS_TCB_FROM_TID(selfTid);
	
    if ( tcb == (LosTaskCB *)*thread )
    {
        return true;
    }
    else
    {
        return false;
    }
}

beken_thread_t *rtos_get_current_thread(void)
{
    uint32_t selfTid = LOS_CurTaskIDGet();
    LosTaskCB *tcb = OS_TCB_FROM_TID(selfTid);
	
    return (beken_thread_t *)tcb;
}

/* Re-write vTaskList to add a buffer size parameter */
OSStatus rtos_print_threads_status( char* pcWriteBuffer, int xWriteBufferLen )
{
    return kNoErr;
}

OSStatus rtos_check_stack(void)
{
    //  TODO: Add stack checking here.
    return kNoErr;
}

OSStatus rtos_thread_force_awake( beken_thread_t *thread )
{
    return kNoErr;
}

void rtos_thread_sleep(uint32_t seconds)
{
    UINT32 uwRet = LOS_OK;
	
	uwRet = LOS_TaskDelay(seconds * LOS_TICKS_PER_SECOND);
	(void)uwRet;
}

void rtos_suspend_thread(beken_thread_t *thread)
{
    UINT32 uwRet;
    LosTaskCB *pstTaskCB = NULL;
	beken_thread_t bk_thread;

    if (OS_INT_ACTIVE) {
        return;
    }

	bk_thread = *thread;
    if (bk_thread == NULL) {
        bk_thread = rtos_get_current_thread();
    }

    pstTaskCB = (LosTaskCB *)bk_thread;
    uwRet = LOS_TaskSuspend(pstTaskCB->taskID);
    switch (uwRet) {
        case LOS_ERRNO_TSK_OPERATE_IDLE:
        case LOS_ERRNO_TSK_SUSPEND_SWTMR_NOT_ALLOWED:
        case LOS_ERRNO_TSK_ID_INVALID:
            return;

        case LOS_ERRNO_TSK_NOT_CREATED:
        case LOS_ERRNO_TSK_ALREADY_SUSPENDED:
        case LOS_ERRNO_TSK_SUSPEND_LOCKED:
            return;

        default:
            return;
    }
}

void rtos_resume_thread(beken_thread_t *thread)
{
    UINT32 uwRet;
    LosTaskCB *pstTaskCB = NULL;
	beken_thread_t bk_thread;

    if (OS_INT_ACTIVE) {
        return;
    }

	bk_thread = *thread;
    if (bk_thread == NULL) {
        bk_thread = rtos_get_current_thread();
    }


    pstTaskCB = (LosTaskCB *)bk_thread;
    uwRet = LOS_TaskResume(pstTaskCB->taskID);

    switch (uwRet) {
        case LOS_ERRNO_TSK_ID_INVALID:
            return;

        case LOS_ERRNO_TSK_NOT_CREATED:
        case LOS_ERRNO_TSK_NOT_SUSPENDED:
            return;

        default:
            return;
    }
}

uint32_t rtos_get_tick_count(void)
{
	return (uint32_t)LOS_TickCountGet();
}

uint32_t beken_tick_ms(void)
{
	return ms_to_tick_ratio;
}

OSStatus beken_time_get_time(beken_time_t* time_ptr)
{
    *time_ptr = (beken_time_t) ( LOS_TickCountGet() * ms_to_tick_ratio ) + beken_time_offset;
    return kNoErr;
}

OSStatus beken_time_set_time(beken_time_t* time_ptr)
{
    beken_time_offset = *time_ptr - (beken_time_t) ( LOS_TickCountGet() * ms_to_tick_ratio );
    return kNoErr;
}

OSStatus rtos_init_semaphore(beken_semaphore_t *semaphore, int max_count )
{
    return rtos_init_semaphore_adv(semaphore, max_count, 0);
}

OSStatus rtos_init_semaphore_adv(beken_semaphore_t *semaphore, int max_count, int init_count)
{
    UINT32 uwRet;
    UINT32 uwSemId;

    if (OS_INT_ACTIVE) {
        *semaphore = (beken_semaphore_t)NULL;
		goto init_aexit;
    }

    if (1 == max_count) {
        uwRet = LOS_BinarySemCreate((UINT16)init_count, &uwSemId);
    } else {
        uwRet = LOS_SemCreate((UINT16)init_count, &uwSemId);
    }
	ASSERT(LOS_OK == uwRet);

    if (uwRet == LOS_OK) {
        *semaphore = (beken_semaphore_t)(GET_SEM(uwSemId));
    } else {
        *semaphore = (beken_semaphore_t)NULL;
    }

init_aexit:
    return ( *semaphore != NULL ) ? kNoErr : kGeneralErr;
}

OSStatus rtos_get_semaphore(beken_semaphore_t *semaphore, uint32_t timeout_ms )
{
    UINT32 uwRet;
    uint32_t timeout;

	ASSERT(semaphore);
    if(timeout_ms == BEKEN_WAIT_FOREVER)
        timeout = LOS_WAIT_FOREVER;
    else
        timeout = timeout_ms / ms_to_tick_ratio;     

    if (*semaphore == NULL) {
        return kParamErr;
    }

    if (OS_INT_ACTIVE && (timeout != LOS_NO_WAIT)) {
        return kUnknownErr;
    }

    uwRet = LOS_SemPend(((LosSemCB *)*semaphore)->semID, timeout);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_SEM_TIMEOUT) {
        return kTimeoutErr;
    } else if (uwRet == LOS_ERRNO_SEM_INVALID) {
        return kParamErr;
    } else if (uwRet == LOS_ERRNO_SEM_PEND_INTERR) {
        return kInProgressErr;
    } else {
        return kGeneralErr;
    } 
}

int rtos_get_sema_count(beken_semaphore_t *semaphore )
{
    uint32_t uwIntSave;
    uint32_t uwCount;

    if (OS_INT_ACTIVE) {
        return 0;
    }

	ASSERT(semaphore);

    if (*semaphore == NULL) {
        return 0;
    }

    uwIntSave = LOS_IntLock();
    uwCount = ((LosSemCB *)*semaphore)->semCount;
    LOS_IntRestore(uwIntSave);

    return uwCount;
}

int rtos_set_semaphore(beken_semaphore_t *semaphore )
{
    UINT32 uwRet;

	ASSERT(semaphore);

    if (*semaphore == NULL) {
        return kParamErr;
    }

    uwRet = LOS_SemPost(((LosSemCB *)*semaphore)->semID);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_SEM_INVALID) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_deinit_semaphore(beken_semaphore_t *semaphore )
{
    UINT32 uwRet;

    if (OS_INT_ACTIVE) {
        return kGeneralErr;
    }

	ASSERT(semaphore);

    if (*semaphore == NULL) {
        return kParamErr;
    }

    uwRet = LOS_SemDelete(((LosSemCB *)*semaphore)->semID);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_SEM_INVALID) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

void rtos_enter_critical(void)
{
}

void rtos_exit_critical(void)
{
}

OSStatus rtos_init_mutex(beken_mutex_t *mutex)
{
    UINT32 uwRet;
    UINT32 uwMuxId;

    if (OS_INT_ACTIVE) {
        *mutex = NULL;
		goto init_exit;
    }

	ASSERT(mutex);

    uwRet = LOS_MuxCreate(&uwMuxId);
    if (uwRet == LOS_OK) {
        *mutex = (beken_mutex_t)(GET_MUX(uwMuxId));
    } else {
        *mutex = (beken_mutex_t)NULL;
    }
	ASSERT(LOS_OK == uwRet);

init_exit:
    if ( *mutex == NULL )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

OSStatus rtos_trylock_mutex(beken_mutex_t *mutex)
{
    UINT32 uwRet;

    if (*mutex == NULL) {
        return kParamErr;
    }

	ASSERT(mutex);

    uwRet = LOS_MuxPend(((LosMuxCB *)*mutex)->muxID, 0);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_MUX_TIMEOUT) {
        return kTimeoutErr;
    } else if (uwRet == LOS_ERRNO_MUX_INVALID) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_lock_mutex(beken_mutex_t *mutex)
{
    UINT32 uwRet;

	ASSERT(mutex);

    if (*mutex == NULL) {
        return kParamErr;
    }

    uwRet = LOS_MuxPend(((LosMuxCB *)*mutex)->muxID, LOS_WAIT_FOREVER);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_MUX_TIMEOUT) {
        return kTimeoutErr;
    } else if (uwRet == LOS_ERRNO_MUX_INVALID) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_unlock_mutex(beken_mutex_t *mutex)
{
    UINT32 uwRet;

    if (*mutex == NULL) {
        return kParamErr;
    }

	ASSERT(mutex);

    uwRet = LOS_MuxPost(((LosMuxCB *)*mutex)->muxID);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_deinit_mutex(beken_mutex_t *mutex)
{
    UINT32 uwRet;

    if (OS_INT_ACTIVE) {
        return kStateErr;
    }

	ASSERT(mutex);

    if (*mutex == NULL) {
        return kParamErr;
    }

    uwRet = LOS_MuxDelete(((LosMuxCB *)*mutex)->muxID);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_MUX_INVALID) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_init_queue( beken_queue_t *queue, const char* name, uint32_t msg_size, uint32_t msg_count )
{
    UINT32 uwRet;
    UINT32 uwQueueID;
	OSStatus ret = kNoErr;

	ASSERT(queue);

    if (0 == msg_count || 0 == msg_size || OS_INT_ACTIVE) {
        *queue = (beken_queue_t)NULL;
		ret = kParamErr;
		goto qinit_exit;
    }

    uwRet = LOS_QueueCreate((char *)name, (UINT16)msg_count, &uwQueueID, 0, (UINT16)msg_size);
	bk_printf("[wzl_debug]LOS_QueueCreate:%s:%d\r\n", name, uwRet);
    if (uwRet == LOS_OK) {
        *queue = (beken_queue_t)(GET_QUEUE_HANDLE(uwQueueID));
    } else {
        *queue = (beken_queue_t)NULL;
		ret = kNoResourcesErr;
    }
	ASSERT(LOS_OK == uwRet);

qinit_exit:
    return ret;
}

OSStatus rtos_push_to_queue( beken_queue_t *queue, void* msg_ptr, uint32_t timeout_ms )
{
    UINT32 uwRet;
    uint32_t timeout;
    uint32_t uwBufferSize;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

	ASSERT(queue);

    if(timeout_ms == BEKEN_WAIT_FOREVER)
        timeout = LOS_WAIT_FOREVER;
    else
        timeout = timeout_ms / ms_to_tick_ratio;    

    if (pstQueue == NULL || msg_ptr == NULL || ((OS_INT_ACTIVE) && (0 != timeout))) {
        return kParamErr;
    }
    if (pstQueue->queueSize < sizeof(uint32_t)) {
        return kParamErr;
    }
    uwBufferSize = (uint32_t)(pstQueue->queueSize - sizeof(uint32_t));
    uwRet = LOS_QueueWriteCopy((uint32_t)pstQueue->queueID, (void *)msg_ptr, uwBufferSize, timeout);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE) {
        return kParamErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT) {
        return kTimeoutErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_push_to_queue_front( beken_queue_t *queue, void *msg_ptr, uint32_t timeout_ms )
{
    UINT32 uwRet;
    uint32_t timeout;
    uint32_t uwBufferSize;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

	ASSERT(queue);

    if(timeout_ms == BEKEN_WAIT_FOREVER)
        timeout = LOS_WAIT_FOREVER;
    else
        timeout = timeout_ms / ms_to_tick_ratio;    

    if (pstQueue == NULL || msg_ptr == NULL || ((OS_INT_ACTIVE) && (0 != timeout))) {
        return kParamErr;
    }
    if (pstQueue->queueSize < sizeof(uint32_t)) {
        return kParamErr;
    }
    uwBufferSize = (uint32_t)(pstQueue->queueSize - sizeof(uint32_t));
    uwRet = LOS_QueueWriteHead((uint32_t)pstQueue->queueID, (void *)msg_ptr, uwBufferSize, timeout);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE) {
        return kParamErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT) {
        return kTimeoutErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_pop_from_queue( beken_queue_t *queue, void *msg_ptr, uint32_t timeout_ms )
{
    UINT32 uwRet;
    uint32_t timeout;
    UINT32 uwBufferSize;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

	ASSERT(queue);

    if(timeout_ms == BEKEN_WAIT_FOREVER)
        timeout = LOS_WAIT_FOREVER;
    else
        timeout = timeout_ms / ms_to_tick_ratio;    

    if (pstQueue == NULL || msg_ptr == NULL || ((OS_INT_ACTIVE) && (0 != timeout))) {
        return kParamErr;
    }

    uwBufferSize = (uint32_t)(pstQueue->queueSize - sizeof(uint32_t));
    uwRet = LOS_QueueReadCopy((uint32_t)pstQueue->queueID, msg_ptr, &uwBufferSize, timeout);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_INVALID || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE) {
        return kParamErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_TIMEOUT) {
        return kTimeoutErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_deinit_queue(beken_queue_t *queue)
{
    UINT32 uwRet;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

	ASSERT(queue);

    if (pstQueue == NULL) {
        return kParamErr;
    }

    if (OS_INT_ACTIVE) {
        return kGeneralErr;
    }

    uwRet = LOS_QueueDelete((uint32_t)pstQueue->queueID);
    if (uwRet == LOS_OK) {
        return kNoErr;
    } else if (uwRet == LOS_ERRNO_QUEUE_NOT_FOUND || uwRet == LOS_ERRNO_QUEUE_NOT_CREATE) {
        return kParamErr;
    } else {
        return kUnknownErr;
    }
}

uint32_t _queue_get_capacity(beken_queue_t *queue)
{
    uint32_t capacity;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

    if (pstQueue == NULL) {
        capacity = 0U;
    } else {
        capacity = pstQueue->queueLen;
    }

    return (capacity);
}

uint32_t _queue_get_count(beken_queue_t *queue)
{
    uint32_t count;
    UINTPTR uwIntSave;
    LosQueueCB *pstQueue = (LosQueueCB *)*queue;

	ASSERT(queue);

    if (pstQueue == NULL) {
        count = 0U;
    } else {
        uwIntSave = LOS_IntLock();
        count = (uint32_t)(pstQueue->readWriteableCnt[OS_QUEUE_READ]);
        LOS_IntRestore(uwIntSave);
    }
    return count;
}

BOOL rtos_is_queue_full(beken_queue_t *queue)
{
	ASSERT(queue);
    return ( _queue_get_capacity(queue) == _queue_get_count(queue) ) ? true : false;
}

BOOL rtos_is_queue_empty(beken_queue_t *queue)
{
	ASSERT(queue);
    return ( 0 == _queue_get_count(queue) ) ? true : false;
}

static void timer_callback2( void *handle )
{
    beken2_timer_t *timer = (beken2_timer_t *) handle;

	if(BEKEN_MAGIC_WORD != timer->beken_magic)
	{
		return;
	}
    if ( timer->function )
    {
        timer->function( timer->left_arg, timer->right_arg );
    }
}

static void timer_callback1( void *handle )
{
    beken_timer_t *timer = (beken_timer_t*) handle;

    if ( timer->function )
    {
        timer->function( timer->arg);
    }
}

OSStatus rtos_init_oneshot_timer( beken2_timer_t *timer, 
									uint32_t time_ms, 
									timer_2handler_t func,
									void* larg, 
									void* rarg )
{
    SWTMR_CTRL_S *pstSwtmr;
	OSStatus ret = kNoErr;
    UINT32 usSwTmrID;
    UINT8 mode;

	timer->handle = NULL;
    if (NULL == func) {
		ret = kParamErr;
		goto tinit_exit;
    }

    mode = LOS_SWTMR_MODE_NO_SELFDELETE;
	timer->beken_magic = BEKEN_MAGIC_WORD;
	timer->function = func;
	timer->left_arg = larg;
	timer->right_arg = rarg;
	
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == 1)
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)timer_callback2, &usSwTmrID, (uint32_t)(UINTPTR)timer,
        osTimerRousesAllow, osTimerAlignIgnore)) {
        ret = kGeneralErr;
		goto tinit_exit;
    }
#else
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)func, &usSwTmrID, (uint32_t)(UINTPTR)larg)) {
        ret = kGeneralErr;
		goto tinit_exit;
    }
#endif

	pstSwtmr = (SWTMR_CTRL_S *)OS_SWT_FROM_SID(usSwTmrID);
    timer->handle = (void *)pstSwtmr;

	if(pstSwtmr){
		pstSwtmr->uwInterval = time_ms / ms_to_tick_ratio;
	}

tinit_exit:
    return ret;
}

OSStatus rtos_start_oneshot_timer(beken2_timer_t *timer)
{
	return rtos_start_timer((beken_timer_t *)timer);
}

OSStatus rtos_deinit_oneshot_timer(beken2_timer_t *timer)
{
	timer->beken_magic = 0;
	timer->left_arg = 0;
	timer->right_arg = 0;
	timer->function = NULL;
	
	return rtos_deinit_timer((beken_timer_t *)timer);
}

OSStatus rtos_stop_oneshot_timer(beken2_timer_t *timer)
{
    return rtos_stop_timer((beken_timer_t *)timer);
}

BOOL rtos_is_oneshot_timer_init(beken2_timer_t *timer)
{
    return (NULL != timer->handle) ? true : false;
}

BOOL rtos_is_oneshot_timer_running(beken2_timer_t *timer)
{
    return rtos_is_timer_running((beken_timer_t *)timer);
}

OSStatus rtos_oneshot_reload_timer(beken2_timer_t *timer)
{
    return rtos_start_timer((beken_timer_t *)timer);
}

OSStatus rtos_oneshot_reload_timer_ex(beken2_timer_t *timer,
										uint32_t time_ms,
										timer_2handler_t function,
										void *larg,
										void *rarg)
{
	return kUnsupportedErr;
}

OSStatus rtos_init_timer(beken_timer_t *timer, 
									uint32_t time_ms, 
									timer_handler_t func, 
									void* argument )
{
    SWTMR_CTRL_S *pstSwtmr;
	OSStatus ret = kNoErr;
    UINT32 usSwTmrID;
    UINT8 mode;

	timer->handle = NULL;
	
    if (NULL == func) {
		ret = kParamErr;
		goto tinit_exit;
    }

    mode = LOS_SWTMR_MODE_PERIOD;
	timer->function = func;
	timer->arg = argument;
	
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == 1)
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)timer_callback1, &usSwTmrID, (uint32_t)(UINTPTR)timer,
        osTimerRousesAllow, osTimerAlignIgnore)) {
		ret = kGeneralErr;
		goto tinit_exit;
    }
#else
    if (LOS_OK != LOS_SwtmrCreate(1, mode, (SWTMR_PROC_FUNC)timer_callback1, &usSwTmrID, (uint32_t)(UINTPTR)timer)) {
		ret = kGeneralErr;
		goto tinit_exit;
    }
#endif

    timer->handle = (void *)OS_SWT_FROM_SID(usSwTmrID);
	pstSwtmr = (SWTMR_CTRL_S *)timer->handle;


	if(pstSwtmr){
		pstSwtmr->uwInterval = time_ms / ms_to_tick_ratio;
	}

tinit_exit:
    return ret;
}

OSStatus rtos_start_timer(beken_timer_t *timer)
{
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr;

    if (NULL == timer) {
        return kParamErr;
    }

    UINTPTR intSave = LOS_IntLock();
    pstSwtmr = (SWTMR_CTRL_S *)timer;
    uwRet = LOS_SwtmrStart(pstSwtmr->usTimerID);
    LOS_IntRestore(intSave);
    if (LOS_OK == uwRet) {
        return kNoErr;
    } else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet) {
        return kParamErr;
    } else {
        return kTimeoutErr;
    }
}

OSStatus rtos_stop_timer(beken_timer_t *timer)
{
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr;

    if (NULL == timer) {
        return kParamErr;
    }

	pstSwtmr = (SWTMR_CTRL_S *)timer->handle;
    uwRet = LOS_SwtmrStop(pstSwtmr->usTimerID);
    if (LOS_OK == uwRet) {
        return kNoErr;
    } else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_reload_timer(beken_timer_t *timer)
{
    return rtos_start_timer(timer);
}

OSStatus rtos_change_period(beken_timer_t *timer, uint32_t time_ms)
{
    UINT32 uwRet;
	UINTPTR intSave;
    SWTMR_CTRL_S *pstSwtmr;
	
    if (NULL == timer) {
        return kParamErr;
    }
	rtos_stop_timer(timer);

    intSave = LOS_IntLock();
    pstSwtmr = (SWTMR_CTRL_S *)timer->handle;
    pstSwtmr->uwInterval = time_ms / ms_to_tick_ratio;
    uwRet = LOS_SwtmrStart(pstSwtmr->usTimerID);
    LOS_IntRestore(intSave);
    if (LOS_OK == uwRet) {
        return kNoErr;
    } else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

OSStatus rtos_deinit_timer(beken_timer_t *timer)
{
    UINT32 uwRet;
    SWTMR_CTRL_S *pstSwtmr;

	pstSwtmr = (SWTMR_CTRL_S *)timer->handle;
    if (NULL == pstSwtmr) {
        return kParamErr;
    }

    uwRet = LOS_SwtmrDelete(pstSwtmr->usTimerID);
    if (LOS_OK == uwRet) {
		timer->handle = NULL;
		timer->arg = 0;
		timer->function = NULL;
        return kNoErr;
    } else if (LOS_ERRNO_SWTMR_ID_INVALID == uwRet) {
        return kParamErr;
    } else {
        return kGeneralErr;
    }
}

uint32_t rtos_get_timer_expiry_time(beken_timer_t *timer)
{
	uint32_t val;
    UINTPTR intSave;
    SWTMR_CTRL_S *pstSwtmr;

	intSave = LOS_IntLock();
    pstSwtmr = (SWTMR_CTRL_S *)timer->handle;
    val = pstSwtmr->uwInterval;
    LOS_IntRestore(intSave);
	
    return val;
}

uint32_t rtos_get_next_expire_time(void)
{
    return kGeneralErr;
}

uint32_t rtos_get_current_timer_count(void)
{
    return kGeneralErr;
}

BOOL rtos_is_timer_init(beken_timer_t *timer)
{
    return (NULL != timer->handle) ? true : false;
}

BOOL rtos_is_timer_running(beken_timer_t *timer)
{
    if (NULL == timer->handle) {
        return 0;
    }

    return (OS_SWTMR_STATUS_TICKING == ((SWTMR_CTRL_S *)timer->handle)->ucState);
}

OSStatus rtos_init_event_flags( beken_event_flags_t* event_flags )
{
    UNUSED_PARAMETER( event_flags );
	
    return kUnsupportedErr;
}

OSStatus rtos_wait_for_event_flags( beken_event_flags_t* event_flags, uint32_t flags_to_wait_for, uint32_t* flags_set, beken_bool_t clear_set_flags, beken_event_flags_wait_option_t wait_option, uint32_t timeout_ms )
{
    UNUSED_PARAMETER( event_flags );
    UNUSED_PARAMETER( flags_to_wait_for );
    UNUSED_PARAMETER( flags_set );
    UNUSED_PARAMETER( clear_set_flags );
    UNUSED_PARAMETER( wait_option );
    UNUSED_PARAMETER( timeout_ms );

    return kUnsupportedErr;
}

OSStatus rtos_set_event_flags( beken_event_flags_t* event_flags, uint32_t flags_to_set )
{
    UNUSED_PARAMETER( event_flags );
    UNUSED_PARAMETER( flags_to_set );
	
    return kUnsupportedErr;
}

OSStatus rtos_deinit_event_flags( beken_event_flags_t* event_flags )
{
    UNUSED_PARAMETER( event_flags );
	
    return kUnsupportedErr;
}

/**
 * Gets time in milliseconds since RTOS start
 *
 * @Note: since this is only 32 bits, it will roll over every 49 days, 17 hours.
 *
 * @returns Time in milliseconds since RTOS started.
 */
beken_time_t rtos_get_time(void)
{
    return (beken_time_t) ( LOS_TickCountGet() * ms_to_tick_ratio );
}

/**
 * Delay for a number of milliseconds
 *
 * Processing of this function depends on the minimum sleep
 * time resolution of the RTOS.
 * The current thread sleeps for the longest period possible which
 * is less than the delay required, then makes up the difference
 * with a tight loop
 *
 * @return OSStatus : kNoErr if delay was successful
 *
 */
OSStatus rtos_delay_milliseconds( uint32_t num_ms )
{
    uint32_t ticks;

    ticks = num_ms / ms_to_tick_ratio;
    if (ticks == 0)
        ticks = 1;

    LOS_TaskDelay( ticks );

    return kNoErr;
}

/*-----------------------------------------------------------*/
void *beken_malloc( size_t xWantedSize )
{
	return (void *)LOS_MemAlloc(m_aucSysMem0, xWantedSize);;
}

/*-----------------------------------------------------------*/
void beken_free( void *pv )
{
	LOS_MemFree(m_aucSysMem0, pv);
}

/*-----------------------------------------------------------*/
void *beken_realloc( void *pv, size_t xWantedSize )
{
	return LOS_MemRealloc(m_aucSysMem0, pv, xWantedSize);
}

void rtos_dump_all_thread(void)
{
	//To be completed
}

void rtos_dump_stack(beken_thread_t *task)
{
}

// eof

