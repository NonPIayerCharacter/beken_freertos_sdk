#include "test_adc.h"
#include "include.h"

#if TEST_ADC_DEMO
#include "string.h"
#include "stdlib.h"
#include "mem_pub.h"
#include "uart_pub.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "rtos_pub.h"
#include "error.h"
#include "saradc_pub.h"

beken_timer_t *test_adc_demo_timer = NULL;
saradc_desc_t test_adc_demo_adc1;
DD_HANDLE test_adc_demo_handle = -1;

void test_adc_demo_isr_cb(void)
{
    uint32_t mode = 0x00;

    if(test_adc_demo_adc1.data_buff_size <= test_adc_demo_adc1.current_sample_data_cnt)
    {
        ddev_close(test_adc_demo_handle);
        saradc_ensure_close();

        {
        UINT32 sum = 0, sum1, sum2;
        UINT16 *pData = test_adc_demo_adc1.pData;
        sum1 = pData[1] + pData[2];
        sum2 = pData[3] + pData[4];
        sum = sum1/ 2  + sum2 / 2;
        sum = sum / 2;
        sum = sum / 4;
        pData[0] = sum;
        }
    }
}

VOID test_adc_demo_init(void)
{
    uint32_t ret;
    UINT32 status;
    GLOBAL_INT_DECLARATION();

    os_memset(&test_adc_demo_adc1, 0x00, sizeof(saradc_desc_t));
    saradc_config_param_init(&test_adc_demo_adc1);

    test_adc_demo_adc1.channel = 1;
    test_adc_demo_adc1.data_buff_size = 20;
    test_adc_demo_adc1.mode = 3;
    test_adc_demo_adc1.current_read_data_cnt = 0;
    test_adc_demo_adc1.current_sample_data_cnt = 0;
    test_adc_demo_adc1.has_data = 0;
    test_adc_demo_adc1.p_Int_Handler = test_adc_demo_isr_cb;
    test_adc_demo_adc1.pData = os_malloc(sizeof(UINT16) * test_adc_demo_adc1.data_buff_size);
    if(!test_adc_demo_adc1.pData)
    {
        os_printf("malloc failed\n");
        return;
    }

    ret = 0;
    do {
        GLOBAL_INT_DISABLE();
        if(saradc_check_busy() == 0) {
            test_adc_demo_handle = ddev_open(SARADC_DEV_NAME, &status, (UINT32)&test_adc_demo_adc1);
            if(DD_HANDLE_UNVALID != test_adc_demo_handle)
            {
                GLOBAL_INT_RESTORE();
                break;
            }
        }
    GLOBAL_INT_RESTORE();

    rtos_delay_milliseconds(5);
    ret++;
    } while(ret<5);

    if(ret == 5) {
        os_free(test_adc_demo_adc1.pData);
        os_printf("adc_open failed\n");
        return;
    }
}

void test_adc_demo_timer_alarm( void *arg )
{
    uint32_t mode = 0x01;
    uint32_t i;

    if(test_adc_demo_adc1.data_buff_size <= test_adc_demo_adc1.current_sample_data_cnt)
    {
        os_printf("adc_result:%d\n", test_adc_demo_adc1.pData[0]);

        os_free(test_adc_demo_adc1.pData);
        // adc sample again
        test_adc_demo_init();
    }
}

int create_test_adc_demo_timer(void)
{
    OSStatus err;

    if(test_adc_demo_timer == NULL)
    {
        test_adc_demo_timer = (beken_timer_t *) os_malloc( sizeof(beken_timer_t) );
    }

    if(test_adc_demo_timer == NULL)
    {
        os_printf("test_adc_demo_timer error!\r\n");
        return kGeneralErr;
    }

    err = rtos_init_timer(test_adc_demo_timer, 500, test_adc_demo_timer_alarm, 0);
    if(kNoErr != err)
    {
        if(test_adc_demo_timer)
        {
            os_free(test_adc_demo_timer);
            test_adc_demo_timer = NULL;
        }
        os_printf("test_adc_demo_timer init failed!\r\n");
    }

    return err;
}

void destroy_test_adc_demo_timer( void )
{
    rtos_deinit_timer( test_adc_demo_timer );
    if(test_adc_demo_timer)
    {
        os_free(test_adc_demo_timer);
        test_adc_demo_timer = NULL;
    }
}

int demo_start(void)
{
    if(create_test_adc_demo_timer() == kNoErr)
    {
        if(kNoErr == rtos_start_timer(test_adc_demo_timer))
        {
            return kNoErr;
        }

        destroy_test_adc_demo_timer();
        os_printf("test_adc_timer start failed!\r\n");
    }

    return kGeneralErr;
}

#endif /*TEST_ADC_DEMO*/

// EOF

