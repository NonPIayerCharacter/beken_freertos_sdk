/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
#include "rwip_config.h" // RW SW configuration
#include "architect.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include <string.h>   // boolean definition
#include "rwip.h"      // RW SW initialization
#include "prf.h"      // RW SW initialization
#include "rwble.h"
#include "rwble_config.h"
#include "uart_pub.h"
#include "rtos_pub.h"
#include "ble.h"
#include "ble_pub.h"
#include "ble_api_5_x.h"
#include "sys_ctrl_pub.h"
#include "icu_pub.h"
#include "intc_pub.h"
#include "drv_model_pub.h"
#include "uart_ble.h"
#include "ble_reg_blecore.h"
#include "BK3633_RegList.h"
#include "param_config.h"
#include "common_utils.h"
#include "ate_app.h"
#include "power_save_pub.h"
#include "rf.h"
#if CFG_LOW_VOLTAGE_PS
#include "low_voltage_ps.h"
#endif

beken_queue_t ble_msg_que = NULL;
beken_thread_t ble_thread_handle = NULL;
uint8_t ble_system_mode;
uint8_t tx_pwr_idx;
static uint32_t ble_sleep_enable = 1;
__attribute__((aligned(0x10))) uint8_t ex_mem[EM_BLE_END] = { 0 };

extern void intc_service_change_handler(UINT8 int_num, FUNCPTR isr);
extern void xvr_reg_tx_pwr_set(uint32_t tx_pwr);
extern uint32_t ble_cal_get_txpwr(uint8_t idx);
extern void rwnx_cal_set_txif_2rd(uint8_t txif_2rd_b, uint8_t txif_2rd_g);
extern void bk7011_set_rx_hpf_bypass(UINT8 bypass);
extern UINT32 sctrl_ctrl(UINT32 cmd, void *param);

enum {
	DUT_IDLE,
	DUT_RUNNING,
};

uint8_t ble_dut_status = DUT_IDLE;

const struct rwip_eif_api uart_api =
{
    ble_uart_read,
    ble_uart_write,
    ble_uart_flow_on,
    ble_uart_flow_off,
};

static SDD_OPERATIONS ble_op =
{
    ble_ctrl
};

#if (BK_BLE_ASSERT)
void assert_err(const char *condition, const char * file, int line)
{
	bk_printf("%s,condition %s,file %s,line = %d\r\n",__func__,condition,file,line);
}

void assert_param(int param0, int param1, const char * file, int line)
{
	bk_printf("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);
}

void assert_warn(int param0, int param1, const char * file, int line)
{
	bk_printf("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);
}
#endif

void platform_reset(uint32_t error)
{
	bk_printf("reset error = %x\r\n", error);

	//watch dog reset
	extern void bk_reboot(void);
	bk_reboot();
}

void ble_set_power_up(uint32 up)
{
	if (up) {
		sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLE_POWERUP, NULL);
	} else {
		sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_BLE_POWERDOWN, NULL);
	}
}

void bdaddr_env_init(uint8_t addr_type, struct bd_addr *addr)
{
	uint8_t sta_mac[BD_ADDR_LEN];
	uint8_t *ble_mac = &common_default_bdaddr.addr[0];

	wifi_get_mac_address((char *)sta_mac, CONFIG_ROLE_STA);
	sta_mac[5] += 1; // add 1, diff from wifi's mac

	for (int i = 0; i < BD_ADDR_LEN; i++) {
		ble_mac[i] = sta_mac[BD_ADDR_LEN - 1 - i];
	}
	bk_printf("ble mac:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
		ble_mac[5], ble_mac[4], ble_mac[3], ble_mac[2], ble_mac[1], ble_mac[0]);

	if (addr_type == ADDR_RAND) {
		ble_mac = &common_static_addr.addr[0];
		memcpy(ble_mac, &addr->addr[0], BD_ADDR_LEN);
		bk_printf("ble static addr:%02x-%02x-%02x-%02x-%02x-%02x\r\n",
			ble_mac[5], ble_mac[4], ble_mac[3], ble_mac[2], ble_mac[1], ble_mac[0]);
	}
}

void ble_sys_mode_init(uint8_t mode)
{
	ble_system_mode = mode;
}

uint8_t ble_get_sys_mode(void)
{
	return ble_system_mode;
}

void ble_clk_power_up(uint32 up)
{
	UINT32 param;
	param = PWD_BLE_CLK_BIT;
	
	if (up) {
		sddev_control(ICU_DEV_NAME, CMD_TL410_CLK_PWR_UP, &param);
	} else {
		sddev_control(ICU_DEV_NAME, CMD_TL410_CLK_PWR_DOWN, &param);	
	}
}

void ble_intc_enable(uint32_t enable)
{
    if (enable) {
		intc_enable(FIQ_BTDM);
        intc_enable(FIQ_BLE);
    } else {
		intc_disable(FIQ_BTDM);
		intc_disable(FIQ_BLE);
    }
}

void ble_btdm_isr(void)
{
	rwip_isr();
	ble_send_msg(BLE_MSG_POLL);
	return;
}

void ble_ble_isr(void)
{
	rwble_isr();
	ble_send_msg(BLE_MSG_POLL);
	return;
}

void ble_init(void)
{
	intc_service_register( FIQ_BLE, PRI_FIQ_BLE, ble_ble_isr );
	intc_service_register( FIQ_BTDM, PRI_FIQ_BTDM, ble_btdm_isr );

	sddev_register_dev( BLE_DEV_NAME, &ble_op );

	return;
}

void ble_exit(void)
{
	sddev_unregister_dev( BLE_DEV_NAME );

	return;

}

void ble_send_msg(UINT32 data)
{
	BLE_MSG_T msg;

	if (ble_msg_que) {
		msg.data = data;
		rtos_push_to_queue(&ble_msg_que, &msg, BEKEN_NO_WAIT);
	}
}

extern int bk7011_reduce_vdddig_for_rx(int reduce);
extern void intc_service_change_handler(UINT8 int_num, FUNCPTR isr);
#define		BLE_DUT_DIVISION			(8)

void enter_dut_fcc_mode(void)
{
#if CFG_TX_EVM_TEST || CFG_RX_SENSITIVITY_TEST
	bk_printf("enter dut mode\r\n");
	uint32_t tx_pwr;

#ifdef FOR_TEST
	ble_dut_status = DUT_RUNNING;
	tx_pwr = ble_cal_get_txpwr(tx_pwr_idx);
	xvr_reg_tx_pwr_set(tx_pwr);
	if (uart_print_port == UART2_PORT) {
		intc_service_change_handler(IRQ_UART2, ble_uart_isr);
	} else {
		intc_service_change_handler(IRQ_UART1, ble_uart_isr);
	}
#endif
	while (1) {
		OSStatus err;
		BLE_MSG_T msg;

		err = rtos_pop_from_queue(&ble_msg_que, &msg, BEKEN_WAIT_FOREVER);
		if (kNoErr == err) {
			switch (msg.data) {
				case BLE_MSG_POLL:
					//schedule all pending events      
					rwip_schedule();
					break;
				case BLE_MSG_DUT:
					//ble test code for debug pin
					//ble_diagcntl_set(0x00008383);
					uart_h4tl_data_switch();
					break;
				case BLE_DUT_EXIT:
					bk_printf("exit ble dut\r\n");
					rwnx_cal_set_txif_2rd(1, 1);
					ble_dut_status = DUT_IDLE;
					if (uart_print_port == UART2_PORT) {
						intc_service_change_handler(IRQ_UART2, uart2_isr);
					} else {
						intc_service_change_handler(IRQ_UART1, uart1_isr);
					}
					sctrl_ctrl(CMD_BLE_RF_PTA_EN, NULL);
					sctrl_ctrl(CMD_BLE_RF_BIT_CLR, NULL);
					break;
				case BLE_DUT_START:
					bk_printf("enter ble dut\r\n");
					ble_dut_status = DUT_RUNNING;

					sctrl_ctrl(CMD_BLE_RF_PTA_DIS, NULL);
					sctrl_ctrl(CMD_BLE_RF_BIT_SET, NULL);
					tx_pwr = ble_cal_get_txpwr(tx_pwr_idx);
					xvr_reg_tx_pwr_set(tx_pwr);
					rwnx_cal_set_txif_2rd(0, 0);

					if (uart_print_port == UART2_PORT) {
						intc_service_change_handler(IRQ_UART2, ble_uart_isr);
					} else {
						intc_service_change_handler(IRQ_UART1, ble_uart_isr);
					}
					break;
				default:
					break;
			}
		}
	}
#endif
}

void enter_normal_app_mode(void)
{
	bk_printf("enter normal mode\r\n");
	#if (CFG_USE_PTA)
	sctrl_ctrl(CMD_BLE_RF_PTA_EN,NULL);
	#else
	sctrl_ctrl(CMD_BLE_RF_PTA_DIS,NULL);
	#endif
	while (1) {
		OSStatus err;
		BLE_MSG_T msg;

		err = rtos_pop_from_queue(&ble_msg_que, &msg, BEKEN_WAIT_FOREVER);
		if (kNoErr == err) {
			switch (msg.data) {
				case BLE_MSG_POLL:
					#if CFG_BLE_DIAGNOSTIC_PORT
					//ble test code for debug pin
					ble_diagcntl_pack(1,0x03,1,0x03,0,0,0,0);
					#endif
					//schedule all pending events
					rwip_schedule();
					break;
					case BLE_THREAD_EXIT:
						bk_printf("ble thread exit\r\n");
						goto exit_normal_loop;
						break;
					default:
						break;
			}
		}

		if (ble_ps_enabled())
		{
			GLOBAL_INT_DIS();
			rwip_sleep();
			GLOBAL_INT_RES();
		}
	}
exit_normal_loop:
{
	extern void ble_switch_rf_to_wifi(void);
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	ble_set_power_up(0);
	ble_clk_power_up(0);
	GLOBAL_INT_RESTORE();
	ble_switch_rf_to_wifi();
}
	return;
}

#if CFG_BLE_DIAGNOSTIC_PORT

#define BLE_XVR_SIG		1
#define BLE_IP_SIG		0

static void ble_diagnostic_init(void)
{
	*(volatile unsigned int *)(0x802800 + 0x2A * 4) = 0x05;
	*(volatile unsigned int *)(0x802800 + 0x2B * 4) |= 0xFF00;

	#if BLE_XVR_SIG
	//xvr signal map:
	//GPIO16:ble_lr
	//GPIO17:lrbit_data
	//GPIO24:lrbit_valid
	//GPIO26:lr_sync_find
	//GPIO20:sync_find
	//GPIO21:bb_tx_bit
	//GPIO22:bb_rx_bit
	//GPIO23:ref_clk
	*(volatile unsigned int *)(0x800000 + 0x0D * 4) &= ~(0x07000000);
	#endif
	#if BLE_IP_SIG
	*(volatile unsigned int *)(0x800000 + 0x0D * 4) |= 0x02000000;
	#endif

	*(volatile unsigned int *)(0x802800 + 0x10 * 4) = 0x40; //GPIO16
	*(volatile unsigned int *)(0x802800 + 0x11 * 4) = 0x40; //GPIO17
	*(volatile unsigned int *)(0x802800 + 0x18 * 4) = 0x40; //GPIO24
	*(volatile unsigned int *)(0x802800 + 0x1A * 4) = 0x40; //GPIO26
	*(volatile unsigned int *)(0x802800 + 0x14 * 4) = 0x40; //GPIO20
	*(volatile unsigned int *)(0x802800 + 0x15 * 4) = 0x40; //GPIO21
	*(volatile unsigned int *)(0x802800 + 0x16 * 4) = 0x40; //GPIO22
	*(volatile unsigned int *)(0x802800 + 0x17 * 4) = 0x40; //GPIO23
}
#endif

void ble_thread_main(void *arg)
{
	ble_set_power_up(1);

	ble_clk_power_up(1);

	ble_intc_enable(1);

	if (get_ate_mode_state()) {
		ble_sys_mode_init(DUT_FCC_MODE);
	} else {
		ble_sys_mode_init(NORMAL_MODE);
	}

	bdaddr_env_init(ADDR_PUBLIC, NULL);

	ble_uart_init();

	memset(ex_mem, 0, sizeof(ex_mem));

#if CFG_BLE_DIAGNOSTIC_PORT
	ble_diagnostic_init();
#endif

#if (NVDS_SUPPORT)
	nvds_init();
#endif

	rwip_init(0);

#if (CFG_SUPPORT_MANUAL_CALI)
	extern uint8_t manual_cal_get_ble_pwr_idx(uint8_t channel);
	tx_pwr_idx = manual_cal_get_ble_pwr_idx(19);
#else
	tx_pwr_idx = 50;
#endif

	bk_printf("tx_pwr_idx:%d\r\n", tx_pwr_idx);

	if (ble_get_sys_mode() == DUT_FCC_MODE) {
		enter_dut_fcc_mode();
	} else {
		enter_normal_app_mode();
	}

	rtos_deinit_queue(&ble_msg_que);
	ble_msg_que = NULL;
	ble_thread_handle = NULL;
	#if CFG_BLE_USE_DYN_RAM
	rwip_dyn_heap_free();
	#endif
	rtos_delete_thread(NULL);
}

void ble_thread_exit(void)
{
    if (ble_thread_handle || ble_msg_que) {
        if(if_ble_sleep()) {
            rwip_prevent_sleep_set(RW_BLE_ACTIVE_MODE);
            #if CFG_LOW_VOLTAGE_PS
            lv_ps_element_bt_del();
            lv_ps_bt_wakeup();
            #else
            ble_set_ext_wkup(1);
            #endif
            while(if_ble_sleep()) {
                rtos_delay_milliseconds(4);
            }
        }
        if(ble_thread_handle)
        {
            bk_printf("send msg to ble thread exit\r\n");
            ble_send_msg(BLE_THREAD_EXIT);
            while(ble_thread_handle)
            {
                rtos_delay_milliseconds(4);
            }
        }
        if( rwip_active_check() ) {
            rwip_prevent_sleep_clear(RW_BLE_ACTIVE_MODE);
        }
        if(ble_msg_que)
        {
            rtos_deinit_queue(&ble_msg_que);
            ble_msg_que = NULL;
        }
    }
}

bool ble_thread_is_up(void)
{
	return (ble_thread_handle) ? true : false;
}

void ble_entry(void)
{
    OSStatus ret;

    if (!ble_thread_handle && !ble_msg_que) {
    	ret = rtos_init_queue(&ble_msg_que, 
    							"ble_msg_queue",
    							sizeof(BLE_MSG_T),
    							BLE_MSG_QUEUE_COUNT);
        ASSERT(0 == ret);
        
    	ret = rtos_create_thread(&ble_thread_handle, 
    			4,
    			"ble", 
    			(beken_thread_function_t)ble_thread_main, 
    			BLE_STACK_SIZE, 
    			(beken_thread_arg_t)0);
    	
        ASSERT(0 == ret);
    }
}

UINT32 ble_ctrl( UINT32 cmd, void *param )
{
	UINT32 reg;
	UINT32 ret = ERR_SUCCESS;

	switch (cmd) {
	case CMD_BLE_REG_INIT:
		break;

	case CMD_BLE_REG_DEINIT:
		break;

	case CMD_BLE_SET_CHANNEL:
		reg = REG_READ(BLE_XVR_REG24);
		reg &= ~(BLE_XVR_CHAN_MASK << BLE_XVR_CHAN_POST);
		reg |= (*(UINT32 *)param) << BLE_XVR_CHAN_POST;
		REG_WRITE(BLE_XVR_REG24, reg);
		break;

	case CMD_BLE_START_TX:
		reg = REG_READ(BLE_XVR_REG24);
		reg &= ~(BLE_XVR_AUTO_CHAN_MASK << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		if ((*(UINT8 *)param) == 0x00) {
			reg = 0x3800;
		} else if ((*(UINT8 *)param) == 0x01) {
			reg = 0x3100;
		} else {
			bk_printf("unknow ble test mode\r\n");
		}
		REG_WRITE(BLE_XVR_REG25, reg);
		break;

	case CMD_BLE_START_RX:
		reg = REG_READ(BLE_XVR_REG24);
		reg &= ~(BLE_XVR_AUTO_CHAN_MASK << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		reg = 0x2400;
		REG_WRITE(BLE_XVR_REG25, reg);
		break;

	case CMD_BLE_STOP_TX:
		reg = REG_READ(BLE_XVR_REG24);
		reg |= (BLE_XVR_AUTO_CHAN_MASK << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		reg = 0;
		REG_WRITE(BLE_XVR_REG25, reg);
		break;

	case CMD_BLE_STOP_RX:
		reg = REG_READ(BLE_XVR_REG24);
		reg |= (BLE_XVR_AUTO_CHAN_MASK << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		reg = 0;
		REG_WRITE(BLE_XVR_REG25, reg);
		break;

	case CMD_BLE_HOLD_PN9_ESTIMATE:
		reg = REG_READ(BLE_XVR_REG25);
		reg |= (BLE_XVR_PN9_HOLD_MASK << BLE_XVR_PN9_HOLD_POST);
		REG_WRITE(BLE_XVR_REG25, reg);

	case CMD_BLE_TRIG_RFPLL:
		reg = REG_READ(BLE_XVR_REG24);
		reg &= ~(1 << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		Delay_us(100);
		reg &= ~(BLE_XVR_CHAN_MASK << BLE_XVR_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		Delay_us(100);
		reg |= (1 << BLE_XVR_AUTO_CHAN_POST);
		REG_WRITE(BLE_XVR_REG24, reg);
		Delay_us(100);
		break;

	default:
		ret = ERR_CMD_NOT_SUPPORT;
		break;
	}

	return ret;
}

void ble_dut_start(void)
{
	ble_send_msg(BLE_DUT_START);
}

#define SYS_CTRL_BASE_ADDRESS		0x800000

void ble_set_ext_wkup(uint8_t enable)
{
	if (enable) {
		*(volatile unsigned int *)(SYS_CTRL_BASE_ADDRESS + 0xD * 4) |= 0x1 << 27;
	} else {
		*(volatile unsigned int *)(SYS_CTRL_BASE_ADDRESS + 0xD * 4) &= ~(0x1 << 27);
	}
}

UINT32 ble_in_dut_mode(void)
{
	return (ble_dut_status == DUT_IDLE) ? 0 : 1;
}

void bk_ble_request_rf(void)
{
    if (ble_coex_pta_is_on()) {
        power_save_rf_hold_bit_set(RF_HOLD_BY_BLE_BIT);
        bk7011_set_rx_hpf_bypass(1);
        #if (CFG_LOW_VOLTAGE_PS)
        extern void power_save_ble_lv_cb(void);
        power_save_ble_lv_cb();
        #endif
    }
}

void bk_ble_release_rf(void)
{
    if (ble_coex_pta_is_on()) {
        bk7011_set_rx_hpf_bypass(0);
        power_save_rf_hold_bit_clear(RF_HOLD_BY_BLE_BIT);
    }
}

const struct rwip_eif_api* rwip_eif_get(uint8_t idx)
{
    return &uart_api;
}

void ble_ps_enable_set(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    ble_sleep_enable = 1;
    GLOBAL_INT_RESTORE();
}

void ble_ps_enable_clear(void)
{
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    ble_sleep_enable = 0;
    GLOBAL_INT_RESTORE();
}

UINT32 ble_ps_enabled(void )
{
    uint32_t value = 0;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    value =  ble_sleep_enable;
    GLOBAL_INT_RESTORE();
    return value;
}

void ble_coex_set_pta(bool enable)
{
	if (enable) {
		sctrl_ctrl(CMD_BLE_RF_PTA_EN,NULL);
	} else {
		sctrl_ctrl(CMD_BLE_RF_PTA_DIS,NULL);
	}
}

bool ble_coex_pta_is_on(void)
{
    uint32_t value;
    sctrl_ctrl(CMD_BLE_RF_PTA_GET,&value);

    return value ? true : false;
}

