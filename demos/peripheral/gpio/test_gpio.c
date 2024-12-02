#include "test_gpio.h"
#include "include.h"

#if TEST_GPIO_DEMO
#include "string.h"
#include "stdlib.h"
#include "mem_pub.h"
#include "uart_pub.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "rtos_pub.h"
#include "Error.h"

beken_timer_t *test_gpio_demo_timer = NULL;

void test_gpio_init(GPIO_INDEX gpio_idx, UINT32 mode)
{
    u32 param;

    param =  GPIO_CFG_PARAM(gpio_idx, mode);
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
}

void test_gpio_output_set(GPIO_INDEX gpio_idx)
{
    u32 param;

    param =	GPIO_CFG_PARAM(gpio_idx, 1);
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_OUTPUT, &param);
}

void test_gpio_output_reset(GPIO_INDEX gpio_idx)
{
    u32 param;

    param =	GPIO_CFG_PARAM(gpio_idx, 0);
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_OUTPUT, &param);
}

int test_gpio_input_status(GPIO_INDEX gpio_idx)
{
    u32 param;
    UINT32 status;

    param =	GPIO_CFG_PARAM(gpio_idx, 0);
    status = sddev_control(GPIO_DEV_NAME, CMD_GPIO_INPUT, &param);

    return status;
}

void test_gpio_demo_timer_alarm( void *arg )
{
    static char gpio4_output = 0;
    static int gpio14_input = 0;
    int status;

    if(gpio4_output & 0x01)
    {
        test_gpio_output_set(GPIO4);
    }
    else
    {
        test_gpio_output_reset(GPIO4);
    }
    gpio4_output++;

    status = test_gpio_input_status(GPIO14);
    if(gpio14_input != status)
    {
        gpio14_input = status;
        os_printf("gpio14_input:%x\n", gpio14_input);
    }
}

int create_test_gpio_demo_timer(void)
{
    OSStatus err;

    if(test_gpio_demo_timer == NULL)
    {
        test_gpio_demo_timer = (beken_timer_t *) os_malloc( sizeof(beken_timer_t) );
    }

    if(test_gpio_demo_timer == NULL)
    {
        os_printf("test_gpio_demo_timer error!\r\n");
        return kGeneralErr;
    }

    err = rtos_init_timer(test_gpio_demo_timer, 100, test_gpio_demo_timer_alarm, 0);
    if(kNoErr != err)
    {
        if(test_gpio_demo_timer)
        {
            os_free(test_gpio_demo_timer);
            test_gpio_demo_timer = NULL;
        }
        os_printf("test_gpio_demo_timer init failed!\r\n");
    }

    return err;
}

void destroy_test_gpio_demo_timer( void )
{
    rtos_deinit_timer( test_gpio_demo_timer );
    if(test_gpio_demo_timer)
    {
        os_free(test_gpio_demo_timer);
        test_gpio_demo_timer = NULL;
    }
}

int demo_start(void)
{
    test_gpio_init(GPIO4, GMODE_OUTPUT);
    test_gpio_init(GPIO14, GMODE_INPUT_PULLUP);

    if(create_test_gpio_demo_timer() == kNoErr)
    {
        if(kNoErr == rtos_start_timer(test_gpio_demo_timer))
        {
            return kNoErr;
        }
        destroy_test_gpio_demo_timer();
        os_printf("timer start failed!\r\n");
    }

    return kGeneralErr;
}

#endif /*TEST_GPIO_DEMO*/

