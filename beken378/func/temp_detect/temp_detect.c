#include "include.h"
#include "arm_arch.h"
#include "target_util_pub.h"
#include "mem_pub.h"
#include "drv_model_pub.h"
#include "sys_ctrl_pub.h"
#include "saradc_pub.h"
#include "uart_pub.h"
#include "sys_rtos.h"
#include "rtos_pub.h"
#include "error.h"
#include "fake_clock_pub.h"
#include "bk7011_cal_pub.h"
#include "temp_detect_pub.h"
#include "temp_detect.h"
#include "power_save_pub.h"
#if (1 == CFG_USE_MCU_PS) && (0 == CFG_LOW_VOLTAGE_PS)
#include "mcu_ps_pub.h"
#endif

saradc_desc_t tmp_single_desc;
UINT16 tmp_single_buff[ADC_TEMP_BUFFER_SIZE];
volatile DD_HANDLE tmp_single_hdl = DD_HANDLE_UNVALID;
beken_semaphore_t tmp_single_semaphore = NULL;

extern void manual_cal_tmp_pwr_init(UINT16 init_temp, UINT16 init_thre, UINT16 init_dist);
static void temp_single_get_desc_init(UINT8 channel);
extern void ps_set_temp_prevent(void);
extern void ps_clear_temp_prevent(void);

#if CFG_USE_TEMPERATURE_DETECT || CFG_USE_VOLTAGE_DETECT
volatile DD_HANDLE tmp_detect_hdl = DD_HANDLE_UNVALID;
saradc_desc_t tmp_detect_desc;
UINT16 tmp_detect_buff[ADC_TEMP_BUFFER_SIZE];
TEMP_DETECT_CONFIG_ST g_temp_detect_config;
beken_thread_t  temp_detct_handle = NULL;

enum
{
	TMPD_PAUSE_TIMER          = 0,
    TMPD_RESTART_TIMER,
    TMPD_CHANGE_PARAM,
    TMPD_TIMER_POLL,
    TMPD_INT_POLL,
    VOLT_TIMER_POLL,
    VOLT_INT_POLL,
	TMPD_EXIT,
};

typedef struct temp_message
{
	u32 temp_msg;
}TEMP_MSG_T;

#if CFG_USE_TEMPERATURE_DETECT && CFG_USE_VOLTAGE_DETECT
#define TEMP_DET_QITEM_COUNT          (5+5)
#else
#define TEMP_DET_QITEM_COUNT          (5)
#endif

beken_queue_t tempd_msg_que = NULL;


static void temp_detect_handler(void);
static void temp_detect_main( beken_thread_arg_t data );

static void temp_detect_desc_init(UINT8 channel)
{
    tmp_detect_desc.channel                 = channel;
    tmp_detect_desc.has_data                = 0;
    tmp_detect_desc.current_sample_data_cnt = 0;
}

static void temp_detect_enable_config_sysctrl(void)
{
    UINT32 param;

    param = BLK_BIT_TEMPRATURE_SENSOR;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_ENABLE, &param);
}

static void temp_detect_disable_config_sysctrl(void)
{
    UINT32 param;
    param = BLK_BIT_TEMPRATURE_SENSOR;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_DISABLE, &param);
}

void temp_detect_send_msg(u32 new_msg)
{
	OSStatus ret;
	TEMP_MSG_T msg;

    if(tempd_msg_que) {
    	msg.temp_msg = new_msg;

    	ret = rtos_push_to_queue(&tempd_msg_que, &msg, BEKEN_NO_WAIT);
    	if(kNoErr != ret)
    	{
    		TMP_DETECT_PRT("temp_detect_send_msg failed\r\n");
    	}
    }
}

UINT32 temp_detect_init(UINT32 init_val)
{
    int ret;

    TMP_DETECT_FATAL("temp_detect_init %d\r\n", init_val);

    if((!temp_detct_handle) && (!tempd_msg_que))
    {

    	ret = rtos_init_queue(&tempd_msg_que,
    							"temp_det_queue",
    							sizeof(TEMP_MSG_T),
    							TEMP_DET_QITEM_COUNT);
    	if (kNoErr != ret)
    	{
    		TMP_DETECT_FATAL("temp detect ceate queue failed\r\n");
            return kGeneralErr;
    	}

        ret = rtos_create_thread(&temp_detct_handle,
                                      BEKEN_DEFAULT_WORKER_PRIORITY,
                                      "temp_detct",
                                      (beken_thread_function_t)temp_detect_main,
                                      1024,
                                      (beken_thread_arg_t)init_val);
        if (ret != kNoErr)
        {
            rtos_deinit_queue(&tempd_msg_que);
            tempd_msg_que = NULL;
            TMP_DETECT_FATAL("Error: Failed to create temp_detect: %d\r\n", ret);
            return kGeneralErr;
        }
    }

    return kNoErr;
}

UINT32 temp_detect_uninit(void)
{
    if((temp_detct_handle) && (tempd_msg_que))
    {
        //temp_detect_send_msg(TMPD_EXIT);

        // wait untill task exit
        //while(temp_detct_handle)
        //    rtos_delay_milliseconds(100);

        // set reg mod & pa to initial value, this must be happened in
        // txevm or rxsens to calibration txpwr or rxsens
        manual_cal_temp_pwr_unint();
    }

	return 0;
}

void temp_detect_pause_timer(void)
{
    if(g_temp_detect_config.detect_timer.function
        && rtos_is_timer_running(&g_temp_detect_config.detect_timer))
    {
        temp_detect_send_msg(TMPD_PAUSE_TIMER);
    }
}

void temp_detect_restart_detect(void)
{
    if(g_temp_detect_config.detect_timer.function &&
        !rtos_is_timer_running(&g_temp_detect_config.detect_timer))
    {
        temp_detect_send_msg(TMPD_RESTART_TIMER);
    }
}

UINT32 temp_detect_is_opened_saradc(void)
{
    // if saradc is opened, idle hook to do sleep may turn off saradc's icu clk and inter enbit
    // so before sleep, should check this bit

    return (DD_HANDLE_UNVALID != tmp_detect_hdl);
}

UINT32 temp_detect_is_init(void)
{
    return ((temp_detct_handle) && (tempd_msg_que));
}

static UINT32 temp_detect_open(void)
{
    GLOBAL_INT_DECLARATION();

#if (CFG_SOC_NAME == SOC_BK7231)
    turnoff_PA_in_temp_dect();
#endif // (CFG_SOC_NAME == SOC_BK7231)

    GLOBAL_INT_DISABLE();

    if(saradc_check_busy() == 1)
    {
        tmp_detect_hdl = DD_HANDLE_UNVALID;
        GLOBAL_INT_RESTORE();
        return SARADC_FAILURE;
    }
	
#if CFG_SUPPORT_SARADC
    UINT32 status;

    tmp_detect_hdl = ddev_open(SARADC_DEV_NAME, &status, (UINT32)&tmp_detect_desc);
    if ((DD_HANDLE_UNVALID == tmp_detect_hdl) || (SARADC_SUCCESS != status))
    {
        if (SARADC_SUCCESS != status)
        {
            ddev_close(tmp_detect_hdl);
        }
        tmp_detect_hdl = DD_HANDLE_UNVALID;
        GLOBAL_INT_RESTORE();
        return SARADC_FAILURE;
    }
#endif

    GLOBAL_INT_RESTORE();

    return SARADC_SUCCESS;
}

static UINT32 temp_detect_close(void)
{
    UINT32 status = DRV_SUCCESS;
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    status = ddev_close(tmp_detect_hdl);
    if(DRV_FAILURE == status )
    {
        //GLOBAL_INT_RESTORE();
        //return SARADC_FAILURE;
    }
    tmp_detect_hdl = DD_HANDLE_UNVALID;
    GLOBAL_INT_RESTORE();

    return SARADC_SUCCESS;
}

static UINT32 temp_detect_enable(UINT8 channel)
{
    UINT32 err = SARADC_SUCCESS;

    if(tmp_detect_hdl != DD_HANDLE_UNVALID)
	{
        //aready enable saradc, so return no err
        TMP_DETECT_PRT("aready open\r\n");
        return SARADC_SUCCESS;
    }

    while(tmp_single_hdl !=  DD_HANDLE_UNVALID)
    {
        rtos_delay_milliseconds(10);;
    }

    err = rtos_stop_timer(&g_temp_detect_config.detect_timer);
    ASSERT(kNoErr == err);
    TMP_DETECT_PRT("stop detect timer, start ADC\r\n");

    temp_detect_desc_init(channel);
    if (ADC_TEMP_SENSER_CHANNEL == channel)
    {
        temp_detect_enable_config_sysctrl();
    }

    err = temp_detect_open();
    if(err == SARADC_FAILURE)
    {
        if (ADC_TEMP_SENSER_CHANNEL == channel)
        {
            temp_detect_disable_config_sysctrl();
        }

        TMP_DETECT_FATAL("Can't open saradc, have you register this device?\r\n");
        return err;
    }
    TMP_DETECT_PRT("saradc_open is ok \r\n");

    return SARADC_SUCCESS;
}

static void temp_detect_disable(void)
{
    temp_detect_close();
    saradc_ensure_close();
    if (ADC_TEMP_SENSER_CHANNEL == tmp_detect_desc.channel)
    {
        temp_detect_disable_config_sysctrl();
    }
    TMP_DETECT_PRT("saradc_open is close \r\n");
}

static void temp_detect_timer_handler(void *data)
{
#if CFG_USE_TEMPERATURE_DETECT
    temp_detect_send_msg(TMPD_TIMER_POLL);
#elif CFG_USE_VOLTAGE_DETECT
    temp_detect_send_msg(VOLT_TIMER_POLL);
#endif
}
#endif

#if CFG_USE_TEMPERATURE_DETECT
static void temp_detect_timer_poll(void)
{
    if((temp_detect_enable(ADC_TEMP_SENSER_CHANNEL) != SARADC_SUCCESS))
    {
        rtos_reload_timer(&g_temp_detect_config.detect_timer);
        TMP_DETECT_PRT("temp_detect_enable failed, restart detect timer, \r\n");
    }
#if (1 == CFG_USE_MCU_PS) && (0 == CFG_LOW_VOLTAGE_PS)
    else
    {
        mcu_ps_cb_hold_on(CB_HOLD_BY_TEMP_DETECT);
    }
#endif
}

static void temp_detect_polling_handler(void)
{
    OSStatus err;
    UINT16 cur_val;

    #if (CFG_SOC_NAME != SOC_BK7231)
    cur_val = tmp_detect_desc.pData[0];
    #else
    cur_val = tmp_detect_desc.pData[ADC_TEMP_BUFFER_SIZE-1];
    #endif // (CFG_SOC_NAME != SOC_BK7231)

    g_temp_detect_config.detect_intval_change++;
#if (CFG_SOC_NAME == SOC_BK7238)
    if ((g_temp_detect_config.detect_intval_change > 1)
    && (g_temp_detect_config.detect_intval_change < ADC_TMEP_DETECT_INTVAL_CHANGE)) {
        cur_val = (UINT16)((float)cur_val * 0.7 + (float)g_temp_detect_config.last_detect_val * 0.3);
        //os_printf("temp_code %d&%d->%d\r\n", tmp_detect_desc.pData[0], g_temp_detect_config.last_detect_val, cur_val);
    }
#endif

    TMP_DETECT_PRT("%d:%d seconds: last:%d, cur:%d, thr:%d\r\n",
                    g_temp_detect_config.detect_intval,
					g_temp_detect_config.detect_intval_change,
                    g_temp_detect_config.last_detect_val,
                    cur_val,
                    g_temp_detect_config.detect_thre);

    UINT16 thre = g_temp_detect_config.detect_thre;
#if CFG_USE_STA_PS
    ps_set_temp_prevent();
    power_save_rf_hold_bit_set(RF_HOLD_BY_TEMP_BIT);
    rwnx_cal_do_temp_detect(cur_val, thre, &g_temp_detect_config.last_detect_val);
    ps_clear_temp_prevent();
    power_save_rf_hold_bit_clear(RF_HOLD_BY_TEMP_BIT);
#else
    rwnx_cal_do_temp_detect(cur_val, thre, &g_temp_detect_config.last_detect_val);
#endif

    if(g_temp_detect_config.detect_intval_change == ADC_TMEP_DETECT_INTVAL_CHANGE)
    {
        temp_detect_send_msg(TMPD_CHANGE_PARAM);
    }
    else
    {
#if CFG_USE_VOLTAGE_DETECT
        if (flash_support_wide_voltage()) {
            temp_detect_send_msg(VOLT_TIMER_POLL);
        } else {
            rtos_reload_timer(&g_temp_detect_config.detect_timer);
        }
        (void)err;
#else
        err = rtos_reload_timer(&g_temp_detect_config.detect_timer);
        ASSERT(kNoErr == err);
#endif
    }

#if (1 == CFG_USE_MCU_PS) && (0 == CFG_LOW_VOLTAGE_PS)
    mcu_ps_cb_release();
#endif
}
#endif

#if CFG_USE_VOLTAGE_DETECT
static void volt_detect_timer_poll(void)
{
    UINT32 err = temp_detect_enable(ADC_VOLT_SENSER_CHANNEL);

    if (err != SARADC_SUCCESS)
    {
        err = rtos_reload_timer(&g_temp_detect_config.detect_timer);
        TMP_DETECT_PRT("volt_detect_enable failed, restart detect timer\r\n");
    }
#if (1 == CFG_USE_MCU_PS) && (0 == CFG_LOW_VOLTAGE_PS)
    else
    {
        mcu_ps_cb_hold_on(CB_HOLD_BY_VOLTAGE_DETECT);
    }
#endif
}

static void volt_detect_polling_handler(void)
{
    OSStatus err;
    UINT16 cur_val;

    #if (CFG_SOC_NAME != SOC_BK7231)
    cur_val = tmp_detect_desc.pData[0] * 4;
    #else
    cur_val = tmp_detect_desc.pData[ADC_TEMP_BUFFER_SIZE-1];
    #endif // (CFG_SOC_NAME != SOC_BK7231)

    ps_set_temp_prevent();

    #if CFG_USE_STA_PS
    bk_wlan_dtim_rf_ps_mode_do_wakeup();
    #endif

    rwnx_cal_do_volt_detect(cur_val);
    ps_clear_temp_prevent();

    err = rtos_reload_timer(&g_temp_detect_config.detect_timer);
    if (kNoErr != err)
    {
        TMP_DETECT_FATAL("volt_detect_polling_handler, restart detect timer failed\r\n");
    }

#if (1 == CFG_USE_MCU_PS) && (0 == CFG_LOW_VOLTAGE_PS)
    mcu_ps_cb_release();
#endif
}
#endif

#if CFG_USE_TEMPERATURE_DETECT || CFG_USE_VOLTAGE_DETECT
static void temp_detect_main( beken_thread_arg_t data )
{
    OSStatus err;

    os_memset(&tmp_detect_buff[0], 0, sizeof(UINT16)*ADC_TEMP_BUFFER_SIZE);

    saradc_config_param_init(&tmp_detect_desc);
    tmp_detect_desc.channel = ADC_TEMP_SENSER_CHANNEL;
    tmp_detect_desc.pData = &tmp_detect_buff[0];
    tmp_detect_desc.data_buff_size = ADC_TEMP_BUFFER_SIZE;
    tmp_detect_desc.p_Int_Handler = temp_detect_handler;

    g_temp_detect_config.last_detect_val = (UINT32)(data);
    g_temp_detect_config.inital_data = (UINT32)(data) + ADC_TMEP_DIST_INTIAL_VAL;
    g_temp_detect_config.detect_thre = ADC_TMEP_LSB_PER_10DEGREE * ADC_TMEP_10DEGREE_PER_DBPWR;
    g_temp_detect_config.detect_intval = ADC_TMEP_DETECT_INTVAL_INIT;
    g_temp_detect_config.detect_intval_change = 0;
    g_temp_detect_config.dist_inital = ADC_TMEP_DIST_INTIAL_VAL;

    g_temp_detect_config.last_xtal_val = (UINT32)(data);
    #if (CFG_SOC_NAME != SOC_BK7231)
    g_temp_detect_config.xtal_thre_val = ADC_XTAL_DIST_INTIAL_VAL;
    g_temp_detect_config.xtal_init_val = sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_GET_XTALH_CTUNE, NULL);
    TMP_DETECT_PRT("xtal inital:%d, %d, %d\r\n", g_temp_detect_config.last_xtal_val,
        g_temp_detect_config.xtal_thre_val, g_temp_detect_config.xtal_init_val);
    #endif // (CFG_SOC_NAME != SOC_BK7231)

	err = rtos_init_timer(&g_temp_detect_config.detect_timer,
							g_temp_detect_config.detect_intval * 1000,
							temp_detect_timer_handler,
							(void *)0);
    ASSERT(kNoErr == err);
	err = rtos_start_timer(&g_temp_detect_config.detect_timer);

	ASSERT(kNoErr == err);

    while(1)
    {
        TEMP_MSG_T msg;
        err = rtos_pop_from_queue(&tempd_msg_que, &msg, BEKEN_WAIT_FOREVER);
        if(kNoErr == err)
        {
        	switch(msg.temp_msg)
            {
                case TMPD_PAUSE_TIMER:
                    {
                        TMP_DETECT_PRT("pause_detect\r\n");
                        err = rtos_stop_timer(&g_temp_detect_config.detect_timer);
                        ASSERT(kNoErr == err);
                    }
                    break;
                case TMPD_RESTART_TIMER:
                    {
                        TMP_DETECT_PRT(" restart detect timer\r\n");
                        err = rtos_reload_timer(&g_temp_detect_config.detect_timer);
                        ASSERT(kNoErr == err);
                    }
                    break;
                case TMPD_CHANGE_PARAM:
                    {
                        temp_detect_change_configuration(ADC_TMEP_DETECT_INTVAL,
                            g_temp_detect_config.detect_thre, ADC_TMEP_DIST_INTIAL_VAL);
                    }
                    break;
#if CFG_USE_TEMPERATURE_DETECT
                case TMPD_TIMER_POLL:
                    {
                        temp_detect_timer_poll();
                    }
                    break;
                case TMPD_INT_POLL:
                    temp_detect_polling_handler();
                    break;
#endif
#if CFG_USE_VOLTAGE_DETECT
                case VOLT_TIMER_POLL:
                    volt_detect_timer_poll();
                    break;
                case VOLT_INT_POLL:
                    volt_detect_polling_handler();
                    break;
#endif
                case TMPD_EXIT:
                    goto tempd_exit;
                    break;
                default:
                    break;
            }
        }
    }

tempd_exit:
    err = rtos_deinit_timer(&g_temp_detect_config.detect_timer);
    ASSERT(kNoErr == err);

    rtos_deinit_queue(&tempd_msg_que);
    tempd_msg_que = NULL;

    temp_detct_handle = NULL;
    rtos_delete_thread(NULL);
}

static void temp_detect_handler(void)
{
    UINT32 sum = 0, count = 0;
    UINT32 index = 0;  /* start index to calculate average */
    UINT32 target = 0; /* targe index to storage average */

    if(tmp_detect_hdl == DD_HANDLE_UNVALID)
    {
        //TMP_DETECT_PRT("u_err\r\n");
        return;
    }

    if(tmp_detect_desc.current_sample_data_cnt < tmp_detect_desc.data_buff_size)
    {
        return;
    }

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236)
    index = 5; /* drop first 5 items */
#elif (CFG_SOC_NAME == SOC_BK7231)
    index = ADC_TEMP_BUFFER_SIZE - 1;
    target = ADC_TEMP_BUFFER_SIZE - 1;
    turnon_PA_in_temp_dect();
#else
    index = 1;
#endif

    temp_detect_disable();
    TMP_DETECT_PRT("buff:%p,%d,%d,%d,%d,%d\r\n", tmp_detect_desc.pData,
                   tmp_detect_desc.pData[index + 0], tmp_detect_desc.pData[index + 1],
                   tmp_detect_desc.pData[index + 2], tmp_detect_desc.pData[index + 3],
                   tmp_detect_desc.pData[index + 4]);
    for (; index < ADC_TEMP_BUFFER_SIZE; index++)
    {
        /* 0 is invalid, but saradc may return 0 in power save mode */
#if (CFG_SOC_NAME == SOC_BK7238)
        if ((0 != tmp_detect_desc.pData[index]) && (1023 != tmp_detect_desc.pData[index]))
#else
        if ((0 != tmp_detect_desc.pData[index]) && (2048 != tmp_detect_desc.pData[index]))
#endif
        {
            sum += tmp_detect_desc.pData[index];
            count++;
        }
    }
    if (count == 0)
    {
        tmp_detect_desc.pData[target] = 0;
        return ;
    }
    else
    {
        sum = sum / count;
#if (CFG_SOC_NAME == SOC_BK7238)
        sum = sum / 2;
#elif (CFG_SOC_NAME != SOC_BK7231)
        sum = sum / 4;
#endif
        tmp_detect_desc.pData[target] = sum;
    }

	if (ADC_VOLT_SENSER_CHANNEL == tmp_detect_desc.channel)
	{
		temp_detect_send_msg(VOLT_INT_POLL);
	}
	else
	{
		temp_detect_send_msg(TMPD_INT_POLL);
	}
}

void temp_detect_change_configuration(UINT32 intval, UINT32 thre, UINT32 dist)
{
    OSStatus err;

    if(intval == 0)
        intval = ADC_TMEP_DETECT_INTVAL;
    if(thre == 0)
        thre = ADC_TMEP_LSB_PER_10DEGREE * ADC_TMEP_10DEGREE_PER_DBPWR;
    if(dist == 0)
        dist = ADC_TMEP_DIST_INTIAL_VAL;

    TMP_DETECT_WARN("config: intval:%d, thre:%d, dist:%d\r\n", intval, thre, dist);

    if((g_temp_detect_config.detect_thre != thre)
        || (g_temp_detect_config.dist_inital != dist))
    {
        if(g_temp_detect_config.detect_thre != thre)
            g_temp_detect_config.detect_thre = thre;

        if(g_temp_detect_config.dist_inital != dist)
            g_temp_detect_config.dist_inital = dist;

        manual_cal_tmp_pwr_init(g_temp_detect_config.inital_data,
            g_temp_detect_config.detect_thre, g_temp_detect_config.dist_inital);
    }

    if(g_temp_detect_config.detect_intval != intval)
    {
        g_temp_detect_config.detect_intval = intval;

        if(g_temp_detect_config.detect_timer.function) {
            err = rtos_deinit_timer(&g_temp_detect_config.detect_timer);
            ASSERT(kNoErr == err);
        }

    	err = rtos_init_timer(&g_temp_detect_config.detect_timer,
    							g_temp_detect_config.detect_intval * 1000,
    							temp_detect_timer_handler,
    							(void *)0);
        ASSERT(kNoErr == err);

    	err = rtos_start_timer(&g_temp_detect_config.detect_timer);
    	ASSERT(kNoErr == err);
    }
}

UINT32 temp_get_detect_time(void)
{
    return rtos_get_timer_expiry_time(&g_temp_detect_config.detect_timer);
}

////////////////////////////////////////////////////////////////////////
#endif  // CFG_USE_TEMPERATURE_DETECT

static UINT32 temp_single_get_enable(UINT8 channel)
{
    UINT32 status;

#if CFG_USE_TEMPERATURE_DETECT
    while(tmp_detect_hdl !=  DD_HANDLE_UNVALID)
    {
        rtos_delay_milliseconds(10);
    }
#endif
    temp_single_get_desc_init(channel);

    status = BLK_BIT_TEMPRATURE_SENSOR;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_ENABLE, &status);

#if (CFG_SOC_NAME == SOC_BK7231)
    turnoff_PA_in_temp_dect();
#endif // (CFG_SOC_NAME == SOC_BK7231)

#if CFG_SUPPORT_SARADC
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    tmp_single_hdl = ddev_open(SARADC_DEV_NAME, &status, (UINT32)&tmp_single_desc);
    if(DD_HANDLE_UNVALID == tmp_single_hdl)
    {
        GLOBAL_INT_RESTORE();
        TMP_DETECT_FATAL("Can't open saradc, have you register this device?\r\n");
        return SARADC_FAILURE;
    }
    GLOBAL_INT_RESTORE();
#endif

    return SARADC_SUCCESS;
}

static void temp_single_get_disable(void)
{
    UINT32 status = DRV_SUCCESS;

    status = ddev_close(tmp_single_hdl);
    if(DRV_FAILURE == status )
    {
        //TMP_DETECT_PRT("saradc disable failed\r\n");
        //return;
    }
    saradc_ensure_close();
    tmp_single_hdl = DD_HANDLE_UNVALID;

    status = BLK_BIT_TEMPRATURE_SENSOR;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLK_DISABLE, &status);

    TMP_DETECT_PRT("saradc_open is close \r\n");
}

static void temp_single_detect_handler(void)
{
    if(tmp_single_desc.current_sample_data_cnt >= tmp_single_desc.data_buff_size)
    {
        #if (CFG_SOC_NAME != SOC_BK7231)
        UINT32 sum = 0, sum1, sum2;
        //turnon_PA_in_temp_dect();
        temp_single_get_disable();
        TMP_DETECT_PRT("buff:%p,%d,%d,%d,%d,%d\r\n", tmp_single_desc.pData,
                       tmp_single_desc.pData[0], tmp_single_desc.pData[1],
                       tmp_single_desc.pData[2], tmp_single_desc.pData[3],
                       tmp_single_desc.pData[4]);

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236)
        sum1 = tmp_single_desc.pData[6] + tmp_single_desc.pData[7];
        sum2 = tmp_single_desc.pData[8] + tmp_single_desc.pData[9];
        sum = sum1 / 2 + sum2 / 2;
        sum = sum / 2;
        sum = sum / 4;
#elif (CFG_SOC_NAME == SOC_BK7238)
        sum1 = tmp_single_desc.pData[1] + tmp_single_desc.pData[2];
        sum2 = tmp_single_desc.pData[3] + tmp_single_desc.pData[4];
        sum = sum1 / 2 + sum2 / 2;
        sum = sum / 4;
#else
        sum1 = tmp_single_desc.pData[1] + tmp_single_desc.pData[2];
        sum2 = tmp_single_desc.pData[3] + tmp_single_desc.pData[4];
        sum = sum1 / 2 + sum2 / 2;
        sum = sum / 2;
        sum = sum / 4;
#endif

        tmp_single_desc.pData[0] = sum;
        #else
        turnon_PA_in_temp_dect();
        temp_single_get_disable();
        TMP_DETECT_PRT("buff:%p,%d,%d,%d,%d,%d\r\n", tmp_single_desc.pData,
                       tmp_single_desc.pData[0], tmp_single_desc.pData[1],
                       tmp_single_desc.pData[2], tmp_single_desc.pData[3],
                       tmp_single_desc.pData[4]);
        #endif // (CFG_SOC_NAME != SOC_BK7231)

        rtos_set_semaphore(&tmp_single_semaphore);
    }
}

static void temp_single_get_desc_init(UINT8 channel)
{
    os_memset(&tmp_single_buff[0], 0, sizeof(UINT16)*ADC_TEMP_BUFFER_SIZE);

    saradc_config_param_init(&tmp_single_desc);
    tmp_single_desc.channel = channel;
    tmp_single_desc.pData = &tmp_single_buff[0];
    tmp_single_desc.data_buff_size = ADC_TEMP_BUFFER_SIZE;
    tmp_single_desc.p_Int_Handler = temp_single_detect_handler;
}

UINT32 temp_single_get_current_temperature(UINT32 *temp_value)
{
    UINT32 ret;
    int result;
    int retry_count = 3;

    *temp_value = 0;

    for (; retry_count > 0; retry_count--) {
        if(tmp_single_semaphore == NULL) {
            result = rtos_init_semaphore(&tmp_single_semaphore, 1);
            ASSERT(kNoErr == result);
        }

        ret = temp_single_get_enable(ADC_TEMP_SENSER_CHANNEL);
        if (SARADC_SUCCESS != ret)
        {
            continue;
        }

        ret = 1000; // 1s
        result = rtos_get_semaphore(&tmp_single_semaphore, ret);
        if(result == kNoErr) {
            #if (CFG_SOC_NAME != SOC_BK7231)
            *temp_value = tmp_single_desc.pData[0];
            #else
            *temp_value = tmp_single_desc.pData[4];
            #endif
            ret = 0;
        }else {
            TMP_DETECT_FATAL("temp_single timeout\r\n");
            ret = 1;
        }

        if ((ADC_TEMP_VAL_MIN < *temp_value) && (*temp_value < ADC_TEMP_VAL_MAX)) {
            break;
        }
    }

    return ret;
}

#if (CFG_SOC_NAME == SOC_BK7231N)
UINT32 volt_single_get_current_voltage(UINT32 *volt_value)
{
    UINT32 ret;
    int result;
    int retry_count = 3;

    *volt_value = 0;

    for (; retry_count > 0; retry_count--) {
        if(tmp_single_semaphore == NULL) {
            result = rtos_init_semaphore(&tmp_single_semaphore, 1);
            ASSERT(kNoErr == result);
        }

        ret = temp_single_get_enable(ADC_VOLT_SENSER_CHANNEL);
        if (SARADC_SUCCESS != ret)
        {
            continue;
        }

        ret = 1000; // 1s
        result = rtos_get_semaphore(&tmp_single_semaphore, ret);
        if(result == kNoErr) {
            #if (CFG_SOC_NAME != SOC_BK7231)
            *volt_value = tmp_single_desc.pData[0] * 4;
            #else
            *volt_value = tmp_single_desc.pData[4];
            #endif
            ret = 0;
        }else {
            TMP_DETECT_FATAL("volt_single timeout\r\n");
            ret = 1;
        }

        if ((ADC_TEMP_VAL_MIN < *volt_value) && (*volt_value < ADC_TEMP_VAL_MAX)) {
            break;
        }
    }

    return ret;
}
#endif

