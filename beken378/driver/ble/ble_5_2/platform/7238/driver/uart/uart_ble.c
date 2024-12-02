#include "include.h"
#include <stddef.h>     // standard definition
#include <stdarg.h>
#include <stdint.h>        // standard integer definition
#include <string.h>        // string manipulation
#include <stdio.h>
#include "rwip.h"
#include "uart.h"
#include "uart_ble.h"
#include "cmd_evm.h"
#include "ble_pub.h"
#include "BK3633_RegList.h"
#include "bk7011_cal_pub.h"

volatile static struct uart_env_tag uart_env;
volatile static uint8_t  uart_rx_done = 0;
volatile static uint32_t uart_rx_index = 0;
uint8_t uart_rx_buf[UART_FIFO_MAX_COUNT];
extern uint8_t tx_pwr_idx;
uint8_t ble_test_mode = IDLE_MODE;
uint8_t tx_mode;
static beken_timer_t ble_sens_rx_tmr;

extern void xvr_reg_tx_pwr_set(uint32_t tx_pwr);
extern uint32_t ble_cal_get_txpwr(uint8_t idx);

void ble_uart_init(void)
{
	uart_rx_done = 0;
	uart_rx_index = 0;

	//// Initialize RX and TX transfer callbacks
	uart_env.rx.callback = NULL;
	uart_env.tx.callback = NULL;
	uart_env.uart_tx_buf = NULL;
	uart_env.uart_rx_buf = NULL;
	uart_env.uart_tx_length = 0;
	uart_env.uart_rx_length = 0;
	uart_env.uart_tx_enable = 0;
	uart_env.uart_rx_enable = 0;

	if (ble_get_sys_mode() == DUT_FCC_MODE) {
		hci_data_init((HCI_DATA_TYPE_CMD | HCI_DATA_TYPE_EVENT));
		host_get_event_cbReg(ble_uart_send);
	}
}

void ble_uart_read(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	BLE_ASSERT_ERR(bufptr != NULL);
	BLE_ASSERT_ERR(size != 0);
	BLE_ASSERT_ERR(callback != NULL);
	uart_env.rx.callback = callback;
	uart_env.rx.dummy    = dummy;

	uart_env.uart_rx_buf = bufptr;	////	uart_rx_ptr_setf((uint32_t) bufptr);
	uart_env.uart_rx_length = size; ////	uart_rx_size_setf(size);
	uart_env.uart_rx_enable = 1;	////	uart_rx_start_setf(1);
}

void ble_uart_write(uint8_t *bufptr, uint32_t size, void (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	BLE_ASSERT_ERR(bufptr != NULL);
	BLE_ASSERT_ERR(size != 0);
	BLE_ASSERT_ERR(callback != NULL);
	uart_env.tx.callback = callback;
	uart_env.tx.dummy    = dummy;

	uart_env.uart_tx_buf = bufptr;
	uart_env.uart_tx_length = size;
	uart_env.uart_tx_enable = 1;

	ble_send_msg(BLE_MSG_DUT);
}

void ble_uart_flow_on(void)
{

}
bool ble_uart_flow_off(void)
{
	return true;
}

static void ble_uart_send_byte(uint8_t data)
{
	data = (data & UART_TX_FIFO_DIN_MASK) << UART_TX_FIFO_DIN_POSI;
	if (uart_print_port == UART1_PORT) {
		REG_WRITE(REG_UART1_FIFO_PORT, data);
	} else {
		REG_WRITE(REG_UART2_FIFO_PORT, data);
	}
}

void ble_uart_send(void *buff, uint16_t len)
{
	uint8_t *tmpbuf = (uint8_t *)buff;
	while (len--)
		ble_uart_send_byte(*tmpbuf++);
}

void ble_rx_ct_hdl(void *param)
{
	uint32_t rx_ble = 0;
	uint32_t rx_ble_old = 0;
	uint32_t rx_ble_err = 0;
	uint32_t rx_ble_err_old = 0;

	ble_ctrl(CMD_BLE_HOLD_PN9_ESTIMATE, NULL);

	rx_ble = REG_READ(BLE_XVR_REG15);
	while (rx_ble_old != rx_ble) {
		rx_ble_old = rx_ble;
		rx_ble = REG_READ(BLE_XVR_REG15);
	}

	rx_ble_err = REG_READ(BLE_XVR_REG16);
	while (rx_ble_err_old != rx_ble_err) {
		rx_ble_err_old = rx_ble_err;
		rx_ble_err = REG_READ(BLE_XVR_REG16);
	}

	bk_printf("Total:%d\r\n", rx_ble);
	bk_printf("Error:%d\r\n", rx_ble_err);

	if (rx_ble_err && rx_ble) {
		if (rx_ble_err > 400000)
			bk_printf("BER: > 40 %%\r\n");
		else
			bk_printf("BER:%d.%02d %%\r\n", (10000 * rx_ble_err / rx_ble) / 100,
				(10000 * rx_ble_err / rx_ble) % 100);
	} else {
		bk_printf("BER:%d.%02d %%\r\n", 0, 0);
	}

	bk_printf("\r\n");

	ble_ctrl(CMD_BLE_STOP_RX, NULL);
	ble_ctrl(CMD_BLE_START_RX, NULL);
}

uint8_t uart_rx_cmd_handler(uint8_t *buff, uint8_t len)
{
	// status for check ble cmd or bkreg cmd
	uint8_t status = 0xFF;
	uint8_t length;
	uint32_t tx_pwr;
	length = len;
	switch (buff[0]) {
		case EXIT_DUT_CMD:
			if ((length == 2) && (buff[1] == EXIT_DUT_ACT)) {
				ble_send_msg(BLE_DUT_EXIT);
				status = 0;
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case TX_PWR_SET_CMD:
			if (length == 2) {
				if (buff[1] > 128) {
					tx_pwr_idx = 128;
				} else {
					tx_pwr_idx = buff[1];
				}

				tx_pwr = ble_cal_get_txpwr(tx_pwr_idx);
				xvr_reg_tx_pwr_set(tx_pwr);

				bk_printf("idx:%d\r\n", tx_pwr_idx);

				if (ble_test_mode == USER_TX_MODE)
				{
					ble_ctrl(CMD_BLE_STOP_TX, NULL);
					ble_ctrl(CMD_BLE_START_TX, &tx_mode);
				}

				status = 0;
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case TX_PWR_SAVE_CMD:
			if (length == 3) {
				uint8_t channel = buff[1];
				uint8_t pwr_idx = buff[2];

				if (channel > 39)
					channel = 39;
				if (pwr_idx > 128)
					pwr_idx = 128;
				extern void manual_cal_save_txpwr(UINT32 rate, UINT32 channel, UINT32 pwr_gain);
				manual_cal_save_txpwr(EVM_DEFUALT_BLE_RATE, channel, pwr_idx);

				status = 0;
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case USER_SEND_CMD:
			if (length == 3) {
				if (ble_test_mode == IDLE_MODE) {
					uint8_t channel = buff[1];
					tx_mode = buff[2];
					if (channel > 39)
						channel = 39;

					uint32_t freq = (uint32_t)((channel + 1) * 2);

					ble_ctrl(CMD_BLE_SET_CHANNEL, &freq);
					ble_ctrl(CMD_BLE_START_TX, &tx_mode);

					ble_test_mode = USER_TX_MODE;
				} else {
					bk_printf("ble_test_mode:%d\r\n", ble_test_mode);
				}

				status = 0;
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case USER_RECV_CMD:
			if (length == 2) {
				if (ble_test_mode == IDLE_MODE) {
					uint8_t channel = buff[1];
					if (channel > 39)
						channel = 39;

					uint32_t freq = (uint32_t)((channel + 1) * 2);

					ble_ctrl(CMD_BLE_SET_CHANNEL, &freq);
					ble_ctrl(CMD_BLE_START_RX, NULL);

					if (!rtos_is_timer_init(&ble_sens_rx_tmr)) {
						rtos_init_timer(&ble_sens_rx_tmr, 1000, ble_rx_ct_hdl, (void *)0);
						rtos_start_timer(&ble_sens_rx_tmr);
					}

					ble_test_mode = USER_RX_MODE;
				} else {
					bk_printf("ble_test_mode:%d\r\n", ble_test_mode);
				}

				status = 0;
			}
		break;
		case USER_STOP_CMD:
			if (length == 2) {
				if (buff[1] == USER_SEND_CMD) {
					if (ble_test_mode == USER_TX_MODE) {
						ble_ctrl(CMD_BLE_STOP_TX, NULL);
						ble_test_mode = IDLE_MODE;
					}
					status = 0;
				} else if (buff[1] == USER_RECV_CMD) {
					if (ble_test_mode == USER_RX_MODE) {
						ble_ctrl(CMD_BLE_STOP_RX, NULL);
						if (rtos_is_timer_init(&ble_sens_rx_tmr)) {
							if (rtos_is_timer_running(&ble_sens_rx_tmr))
								rtos_stop_timer(&ble_sens_rx_tmr);
							rtos_deinit_timer(&ble_sens_rx_tmr);
						}
						ble_test_mode = IDLE_MODE;
					}
					status = 0;
				} else {
					bk_printf("unknow dut cmd");
				}
			} else {
				bk_printf("unknow dut cmd");
			}
		break;
		case XTAL_SET_CMD:
		{
			uint8_t xtalh = buff[1];
			manual_cal_set_xtal(xtalh);
		}break;
		default:
		break;
	}

	return status;
}

extern int bkreg_run_command(const char *content, int cnt);
void rwnx_cal_set_anabuf(int tx_mode);
void  ble_uart_isr(void)
{
	UINT32 status;
	UINT32 intr_en;
	UINT32 intr_status;
	UINT32 fifo_status_reg;
	UINT32 tx_pwr;

	if (uart_print_port == UART1_PORT) {
		intr_en = REG_READ(REG_UART1_INTR_ENABLE);
		intr_status = REG_READ(REG_UART1_INTR_STATUS);
		REG_WRITE(REG_UART1_INTR_STATUS, intr_status);
        fifo_status_reg = REG_UART1_FIFO_STATUS;
	} else {
		intr_en = REG_READ(REG_UART2_INTR_ENABLE);
		intr_status = REG_READ(REG_UART2_INTR_STATUS);
		REG_WRITE(REG_UART2_INTR_STATUS, intr_status);
        fifo_status_reg = REG_UART2_FIFO_STATUS;
	}
	status = intr_status & intr_en;

	if (status & (RX_FIFO_NEED_READ_STA | UART_RX_STOP_END_STA)) {
		while(REG_READ(fifo_status_reg) & FIFO_RD_READY)
		{
			if (uart_print_port == UART1_PORT) {
				uart_rx_buf[uart_rx_index] = ((REG_READ(REG_UART1_FIFO_PORT) >> UART_RX_FIFO_DOUT_POSI) & UART_RX_FIFO_DOUT_MASK);
			} else {
				uart_rx_buf[uart_rx_index] = ((REG_READ(REG_UART2_FIFO_PORT) >> UART_RX_FIFO_DOUT_POSI) & UART_RX_FIFO_DOUT_MASK);
			}
			uart_rx_index++;
			if (uart_rx_index == UART_FIFO_MAX_COUNT) {
				uart_rx_index = 0;
			}
		}

		if ((uart_rx_buf[0] == 0x01) && (uart_rx_buf[1] == 0xe0) && (uart_rx_buf[2] == 0xfc)) {
			if (uart_rx_buf[3] == (uart_rx_index - 4)) {
					uart_rx_cmd_handler((uint8_t *)&uart_rx_buf[4], uart_rx_buf[3]);
					bkreg_run_command((char*)&uart_rx_buf[0], uart_rx_index);
			}
		}

		if ((uart_rx_buf[0] == 0x01) && ((uart_rx_buf[3] + 4) == uart_rx_index)
			&& (uart_rx_buf[1] != 0xe0) && (uart_rx_buf[2] != 0xfc)) {
			if (ble_test_mode != IDLE_MODE) {
				bk_printf("user test is running\r\n");
			} else {
				host_send_cmd(uart_rx_buf, uart_rx_index);
			}
		}

		// work around:ble dut have own print show command success, wo should not add print
		if ((uart_rx_buf[0] == 0x01) && ((uart_rx_buf[1] == 0x1e) || (uart_rx_buf[1] == 0x34)) && (uart_rx_buf[2] == 0x20)) {
			// TODO TX
#if (CFG_SUPPORT_MANUAL_CALI)
			extern uint8_t manual_cal_get_ble_pwr_idx(uint8_t channel);
			tx_pwr_idx = manual_cal_get_ble_pwr_idx(uart_rx_buf[4]);
#endif
			tx_pwr = ble_cal_get_txpwr(tx_pwr_idx);
			xvr_reg_tx_pwr_set(tx_pwr);
			bk_printf("c:%d\r\n", uart_rx_buf[4]);
		}

		if ((uart_rx_buf[0] == 0x01) && (uart_rx_buf[1] == 0x1d) && (uart_rx_buf[2] == 0x20)) {
			//TODO RX START
			rwnx_cal_set_anabuf(0);
		}
		if ((uart_rx_buf[0] == 0x01) && (uart_rx_buf[1] == 0x1f) && (uart_rx_buf[2] == 0x20)) {
			//TODO RX STOP
			rwnx_cal_set_anabuf(1);
		}

		uart_rx_index=0;
		uart_rx_done = 1;
	}
}

volatile struct hci_cmd_event_data host_cmd_data;
volatile struct hci_cmd_event_data host_event_data;

void hci_data_init(uint8_t type)
{	////type: 0x01-Clear host cmd data; 0x02-Clear host event data

	if(type & HCI_DATA_TYPE_CMD)
	{
		host_cmd_data.callback = NULL;
		memset((void *)&host_cmd_data.data_buf[0], 0, HCI_DATA_BUF_SIZE);
		host_cmd_data.data_len = 0;
	}
	if(type & HCI_DATA_TYPE_EVENT)
	{
		////	host_event_data.callback = NULL;		////Will clear callback func
		memset((void *)&host_event_data.data_buf[0], 0, HCI_DATA_BUF_SIZE);
		host_event_data.data_len = 0;
	}
}

void host_send_cmd(uint8_t *bufptr, uint16_t length)
{
	//uint16_t tmpCnt = 0;
	BLE_ASSERT_ERR(length <= HCI_DATA_BUF_SIZE);
	host_cmd_data.callback = NULL;		////Test Only
	memcpy((void *)&host_cmd_data.data_buf[0], bufptr, length);
	host_cmd_data.data_len = length;
	ble_send_msg(BLE_MSG_DUT);
}

////void host_get_event(uint8 *bufptr, uint8 length)
void host_get_event(void)
{

	if(host_event_data.callback != NULL)
	{
		host_event_data.callback((void *)host_event_data.data_buf, host_event_data.data_len);
	}
	hci_data_init(HCI_DATA_TYPE_EVENT);
}

void host_get_event_cbReg(void (*callback) (void*, uint16_t))
{
	host_event_data.callback = callback;
}

void uart_h4tl_data_switch(void)
{   
	void (*callback) (void*, uint8_t) = NULL;
	void* data =NULL;
	uint16_t data_len = 0;

	while(uart_env.uart_tx_enable == 1)
	{
		//uart_printf("uart_h4tl_data_switch tx_enable\r\n");
		// Retrieve callback pointer
		callback = uart_env.tx.callback;
		data     = uart_env.tx.dummy;

		uart_env.uart_tx_enable = 0;
		memcpy((void *)&host_event_data.data_buf[data_len], uart_env.uart_tx_buf, uart_env.uart_tx_length);
		data_len += uart_env.uart_tx_length;
		host_event_data.data_len += uart_env.uart_tx_length;
		if(callback != NULL)
		{
			// Clear callback pointer
			uart_env.tx.callback = NULL;
			uart_env.tx.dummy    = NULL;

			// Call handler
			callback(data, RWIP_EIF_STATUS_OK);
		}
    }
    
	if(host_event_data.data_len != 0)
	{	////New Event
		host_get_event();
	}

	data_len = 0;

	if(host_cmd_data.data_len > 0)
	{
		while(uart_env.uart_rx_enable == 1)
		{
			// Retrieve callback pointer
			callback = uart_env.rx.callback;
			data     = uart_env.rx.dummy;
			uart_env.uart_rx_enable = 0;

			memcpy((void *)uart_env.uart_rx_buf, (void *)&host_cmd_data.data_buf[data_len], uart_env.uart_rx_length);

			data_len += uart_env.uart_rx_length;
			if(callback != NULL)
			{
				// Clear callback pointer
				uart_env.rx.callback = NULL;
				uart_env.rx.dummy    = NULL;

				// Call handler
				callback(data, RWIP_EIF_STATUS_OK);
			}
			if(data_len >= host_cmd_data.data_len)
				break;
		}////while(uart_env.uart_rx_enable == 1)
     
		////Clear HCI Cmd data
		hci_data_init(HCI_DATA_TYPE_CMD);        
    }
}

