#include "include.h"
#include "arm_arch.h"

#include "target_util_pub.h"
#include "sys_ctrl_pub.h"
#include "fake_clock_pub.h"
#include "drv_model_pub.h"

/*******************************************************************************
* Function Implemantation
*******************************************************************************/
/*
	MCLK:26MHz, delay(1): about 25us
				delay(10):about 125us
				delay(100):about 850us
 */
void delay(INT32 num)
{
    volatile INT32 i, j;

    for(i = 0; i < num; i ++)
    {
        for(j = 0; j < 100; j ++)
            ;
    }
}

/*delay according to basic_frequency */
extern UINT32 basic_frequency_for_delay;
void delay_us(UINT32 us_count)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    volatile UINT32 i;
    for(i=0;i<us_count*basic_frequency_for_delay;++i)
        ;
    GLOBAL_INT_RESTORE();
}

void delay_ms(UINT32 ms_count)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    volatile UINT32 i;
    for(i=0;i<ms_count*basic_frequency_for_delay*1000;++i)
        ;
    GLOBAL_INT_RESTORE();
}



/*
	when parameter is 1, the return result is approximately 1 ms;
 */

/*
	[delay offset]worst case: delay about 1 second;
 */
void delay_sec(UINT32 ms_count)
{
    UINT32 t0;
    UINT32 t1;

    t0 = fclk_get_second();
    while(1)
    {
        t1 = fclk_get_second();
        if(t1 - t0 >= 1)
        {
            break;
        }
    }
}

/*
	[delay offset]worst case: delay about 1 tick;
 */
void delay_tick(UINT32 tick_count)
{
    UINT32 t0;
    UINT32 t1;

    t0 = fclk_get_tick();
    while(1)
    {
        t1 = fclk_get_tick();
        if(t1 - t0 >= 1)
        {
            break;
        }
    }
}

// EOF

