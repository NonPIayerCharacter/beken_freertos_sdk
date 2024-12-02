#include "mem_pub.h"
#include "rtos_pub.h"

extern char _itcmcode_ram_begin , _itcmcode_ram_end ;
extern char fiq_handler;
extern char __flash_txt_end;
extern size_t strlen(const char *s);
extern void bk_printf(const char *fmt, ...);

extern TaskHandle_t xTaskGetCurrentTaskHandle( void );
extern char *pcTaskGetName( TaskHandle_t xTaskToQuery ); 

bool addr_is_in_itcm(uint32_t addr);
bool addr_is_in_flash_txt(uint32_t addr);
bool code_addr_is_valid(uint32_t addr);

BaseType_t xTaskGetStackInfo(xTaskHandle xTask, uint32_t *stack_top, uint32_t *stack_bottom, uint32_t *stack_size);
void rtos_dump_task_backtrace(beken_thread_t *task);
void vTaskDumpThreadListStack(List_t *list);
void arch_parse_stack_backtrace(const char *str_type, uint32_t stack_top, uint32_t stack_bottom,
						   uint32_t stack_size, bool thumb_mode); 
void rtos_dump_task_list()
{
	int num_of_tasks = 0;
	int buf_len = 0;// info_len = 0;
	char *buf = NULL;

	num_of_tasks = uxTaskGetNumberOfTasks();
	if(num_of_tasks == 0) {
		bk_printf("no task running.\r\n");
		return;
	}
	buf_len = (num_of_tasks + 5) * 100;
	buf = (char*)os_malloc(buf_len);
	if (!buf) {
		bk_printf("dump task list oom.\r\n");
		return;
	}
	bk_printf("%-12s   %-5s   %-5s   %-5s   %-5s\r\n",
		"task", "state", "pri", "water", "no");
	vTaskList(buf);

	buf[buf_len - 1] = '\0';
	os_free(buf);

	bk_printf("\r\n");
}

void vTaskDumpAllThreadStack()
{
	for (int i = 0; i < configMAX_PRIORITIES; i++) {
		vTaskDumpThreadListStack(&pxReadyTasksLists[i]);
	}
	vTaskDumpThreadListStack(&xDelayedTaskList1);
	vTaskDumpThreadListStack(&xDelayedTaskList2);
	vTaskDumpThreadListStack(&xPendingReadyList);
#if ( INCLUDE_vTaskSuspend == 1 )
	vTaskDumpThreadListStack(&xSuspendedTaskList);
#endif
}

void rtos_dump_task_backtrace(beken_thread_t *task)
{
	beken_thread_t t = NULL;
	uint32_t stack_bottom = 0;
	uint32_t stack_top = 0;
	uint32_t stack_size = 0;

	if (!task || !(*task)) {
		t = (beken_thread_t )xTaskGetCurrentTaskHandle();
	} else {
		t = *task;
	}

	xTaskGetStackInfo(t, &stack_top, &stack_bottom, &stack_size);
	arch_parse_stack_backtrace(pcTaskGetName(t), stack_top, stack_bottom, stack_size, true);
}

#define STACK_CALLBACK_BUF_SIZE 32

void arch_parse_stack_backtrace(const char *str_type, uint32_t stack_top, uint32_t stack_bottom,
						   uint32_t stack_size, bool thumb_mode)
{
	uint32_t call_stack_buf[STACK_CALLBACK_BUF_SIZE] = {0};
	uint32_t stack_minimum = stack_bottom - stack_size;
	uint32_t pc;
	int call_stack_index = 0;
	uint32_t init_stack_top = stack_top;

	bk_printf("itcm_begin=%x end=%x flash begin=%x end=%x\n", (uint32_t)&_itcmcode_ram_begin, (uint32_t)&_itcmcode_ram_end, (uint32_t)fiq_handler, (uint32_t)&__flash_txt_end);

	bk_printf("top=%x bottom=%x size=%d mode=%d\n", stack_top, stack_bottom, stack_size, thumb_mode);

	for (; stack_top < stack_bottom && (call_stack_index < STACK_CALLBACK_BUF_SIZE); stack_top += sizeof(size_t)) {
		pc = *((uint32_t *) stack_top);
		/* ARM9 using thumb instruction, so the pc must be an odd number */
		if (thumb_mode && (pc & 1) == 0)
			continue;

		if (code_addr_is_valid(pc)) {
				if (pc & 1)
				pc = pc - 1;

			call_stack_buf[call_stack_index] = pc;
			call_stack_index++;
		}
	}

	if (call_stack_index > 0) {
		int index;
		bk_printf("%-16s   [0x%-6x ~ 0x%-6x]   0x%-6x   %-4d   %-8d   ",
				  str_type, stack_minimum, stack_bottom, init_stack_top, stack_size, init_stack_top < stack_minimum);
		for (index = 0; index < call_stack_index; index++)
			bk_printf("%lx ", call_stack_buf[index]);
		bk_printf("\n");
	} else if (init_stack_top < stack_minimum) {
		bk_printf("%-16s   [0x%-6x ~ 0x%-6x]   0x%-6x   %-4d   %-8d   ",
				  str_type, stack_minimum, stack_bottom, init_stack_top, stack_size, init_stack_top < stack_minimum);
	}
}

void rtos_dump_current_task_name()
{
	beken_thread_t t;

	t = (beken_thread_t)xTaskGetCurrentTaskHandle();
	bk_printf("Current task: %s\n", pcTaskGetName(t));
}
void rtos_dump_backtrace()
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	rtos_dump_current_task_name();
 
	bk_printf("%-16s   %-21s   %-8s   %-4s   %-8s   %-s\n",
		"task", "stack_addr", "top", "size", "overflow", "backtrace");
	vTaskDumpAllThreadStack();
	GLOBAL_INT_RESTORE();
}

#if !((CONFIG_SOC_BK7231U) || (CONFIG_SOC_BK7251))
bool addr_is_in_itcm(uint32_t addr)
{
	return ((addr > (uint32_t)&_itcmcode_ram_begin) && (addr < (uint32_t)&_itcmcode_ram_end));
}
#endif

bool addr_is_in_flash_txt(uint32_t addr)
{

	return ((addr > (uint32_t)fiq_handler) && (addr < (uint32_t)&__flash_txt_end));
}

bool code_addr_is_valid(uint32_t addr)
{
#if ((CONFIG_SOC_BK7231U) || (CONFIG_SOC_BK7251))
	return (addr_is_in_flash_txt(addr));
#else
	return (addr_is_in_itcm(addr) || addr_is_in_flash_txt(addr));
#endif
}

