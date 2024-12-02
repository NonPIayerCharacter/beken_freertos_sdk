#include "test_pwm.h"
#include "include.h"

#if TEST_PWM_DEMO
#include "string.h"
#include "stdlib.h"
#include "mem_pub.h"
#include "uart_pub.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "rtos_pub.h"
#include "Error.h"
#include "pwm_pub.h"

beken_timer_t *test_pwm_demo_timer = NULL;
uint32 pwm3_counter = 0;

void test_pwm1_demo_isr_cb(uint8 para)
{

}

void test_pwm3_demo_isr_cb(uint8 para)
{
    pwm3_counter++;
}


void test_pwm_demo_init(void)
{
    UINT32 status;
    pwm_param_t pwm_para;

#ifdef TEST_PWM_DEMO_PWM_OUT_ENABLE
    /** config PWM1 PWM OUT T = 100uS duty = 50% (GPIO7 out signal) **/
    pwm_para.cfg.bits.en = 1;
    pwm_para.cfg.bits.int_en = 0;
    pwm_para.cfg.bits.mode = 0x00;  ///PWM
    pwm_para.cfg.bits.clk = 0x01;  ///26MHZ
    pwm_para.cfg.bits.rsv = 0;
    pwm_para.duty_cycle = 1300 - 1;
    pwm_para.end_value = 2600 - 1;
    pwm_para.channel = 1;
    pwm_para.p_Int_Handler = test_pwm1_demo_isr_cb;

    sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &pwm_para);
    if(status != 0)
    {
        os_printf("pwm1_open failed\n");
    }
#endif

    /** config PWM3 1mS/times interrupt **/
    pwm_para.cfg.bits.en = 1;   ///Enable PWMx
    pwm_para.cfg.bits.int_en = 1;  ///Enable Int
    pwm_para.cfg.bits.mode = 0x01;  ///TIMER
    pwm_para.cfg.bits.clk = 0x01;  ///26MHz
    pwm_para.cfg.bits.rsv = 0;
    pwm_para.duty_cycle = 0;
    pwm_para.end_value = 26000 - 1;
    pwm_para.channel = 3;
    pwm_para.p_Int_Handler = test_pwm3_demo_isr_cb;

    sddev_control(PWM_DEV_NAME, CMD_PWM_INIT_PARAM, &pwm_para);
    if(status != 0)
    {
        os_printf("pwm3_open failed\n");
    }
}

void test_pwm_demo_timer_alarm( void *arg )
{
    os_printf("PWM3_cnt:%d\n", pwm3_counter);
}

int create_test_pwm_demo_timer(void)
{
    OSStatus err;

    if(test_pwm_demo_timer == NULL)
    {
        test_pwm_demo_timer = (beken_timer_t *) os_malloc( sizeof(beken_timer_t) );
    }

    if(test_pwm_demo_timer == NULL)
    {
        os_printf("test_pwm_demo_timer error!\r\n");
        return kGeneralErr;
    }

    err = rtos_init_timer(test_pwm_demo_timer, 1500, test_pwm_demo_timer_alarm, 0);
    if(kNoErr != err)
    {
        if(test_pwm_demo_timer)
        {
            os_free(test_pwm_demo_timer);
            test_pwm_demo_timer = NULL;
        }
        os_printf("test_pwm_demo_timer init failed!\r\n");
    }

    return err;
}

void destroy_test_pwm_demo_timer( void )
{
    rtos_deinit_timer( test_pwm_demo_timer );
    if(test_pwm_demo_timer)
    {
        os_free(test_pwm_demo_timer);
        test_pwm_demo_timer = NULL;
    }
}

int demo_start(void)
{
    test_pwm_demo_init();

    os_printf("%s,%s:%d", __FILE__, __FUNCTION__, __LINE__);
    if(create_test_pwm_demo_timer() == kNoErr)
    {
        if(kNoErr == rtos_start_timer(test_pwm_demo_timer))
        {
            return kNoErr;
        }
        destroy_test_pwm_demo_timer();
        os_printf("pwm timer start failed!\r\n");
    }

    return kGeneralErr;
}

#endif /* TEST_PWM_DEMO */
// EOF

