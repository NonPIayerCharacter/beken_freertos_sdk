/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief UART driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup UART
 * @{
 ****************************************************************************************
 */ 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include "ble_uart.h"       // uart definition
#include "BK3435_reg.h"
#include "reg_uart.h"   // uart register
#include "rwip.h"       // SW interface
#include "h4tl.h"
#if (PLF_NVDS)
#include "nvds.h"       // NVDS
#endif // (PLF_NVDS)

#include "dbg.h"
 
/*
 * DEFINES
 *****************************************************************************************
 */

/// Max baudrate supported by this UART (in bps)
#define UART_BAUD_MAX      		  3500000
/// Min baudrate supported by this UART (in bps)
#define UART_BAUD_MIN      		  9600

/// Duration of 1 byte transfer over UART (10 bits) in us (for 921600 default baudrate)
#define UART_CHAR_DURATION        11

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
 
 /*
 * common.c
 *
 *  Created on: 2013-5-7
 *  Author: pujie
 */

#include  <stdarg.h>
#include  <stdio.h>
#include "BK3435_reg.h"

#include "ea.h"

#include "ble_pub.h"
#include "cmd_evm.h"

#define Uart_Write_Byte(v)               (REG_APB3_UART_PORT=v)

#define UART_TX_FIFO_COUNT               (REG_APB3_UART_FIFO_STAT&0xff)
#define UART_RX_FIFO_COUNT               ((REG_APB3_UART_FIFO_STAT>>8)&0xff)
#define UART_TX_FIFO_FULL                (REG_APB3_UART_FIFO_STAT&0x00010000)
#define UART_TX_FIFO_EMPTY               (REG_APB3_UART_FIFO_STAT&0x00020000)
#define UART_RX_FIFO_FULL                (REG_APB3_UART_FIFO_STAT&0x00040000)
#define UART_RX_FIFO_EMPTY               (REG_APB3_UART_FIFO_STAT&0x00080000) 
#define UART_TX_WRITE_READY              (REG_APB3_UART_FIFO_STAT&0x00100000) 
#define UART_RX_READ_READY               (REG_APB3_UART_FIFO_STAT&0x00200000) 
#define bit_UART_TXFIFO_NEED_WRITE        0x01
#define bit_UART_RXFIFO_NEED_READ         0x02
#define bit_UART_RXFIFO_OVER_FLOW         0x04
#define bit_UART_RX_PARITY_ERROR          0x08
#define bit_UART_RX_STOP_ERROR            0x10
#define bit_UART_TX_PACKET_END            0x20
#define bit_UART_RX_PACKET_END            0x40
#define bit_UART_RXD_WAKEUP_DETECT        0x80


#define uart_tx_en    0x1      // 0: Disable TX, 1: Enable TX 
#define uart_rx_en    0x1      // 0: Disable RX, 1: Enable RX
#define irda_mode     0x0      // 0: UART  MODE, 1: IRDA MODE
#define data_len      0x3      // 0: 5 bits, 1: 6 bits, 2: 7 bits, 3: 8 bits
#define parity_en     0x0      // 0: NO Parity, 1: Enable Parity
#define parity_mode   0x1      // 0: Odd Check, 1: Even Check 
#define stop_bits     0x0      // 0: 1 stop-bit, 1: 2 stop-bit 
#define uart_clks     16000000 // UART's Main-Freq, 26M 
#define baud_rate     115200   // UART's Baud-Rate,  1M 


unsigned char uart_rx_buf[UART0_RX_FIFO_MAX_COUNT];
unsigned char uart_tx_buf[UART0_TX_FIFO_MAX_COUNT];
volatile bool uart_rx_done = 0;
volatile unsigned long uart_rx_index = 0;
uint8_t cur_read_buf_idx = 0;
uint8_t ble_test_mode = IDLE_MODE;
uint8_t tx_mode;

#define UART_READ_BYTE()                 (REG_APB3_UART_PORT&0xff)


///UART Character format
enum UART_CHARFORMAT
{
    UART_CHARFORMAT_8 = 0,
    UART_CHARFORMAT_7 = 1
};

///UART Stop bit
enum UART_STOPBITS
{
    UART_STOPBITS_1 = 0,
    UART_STOPBITS_2 = 1  /* Note: The number of stop bits is 1.5 if a character format
                            with 5 bit is chosen*/
};

///UART Parity enable
enum UART_PARITY
{
    UART_PARITY_DISABLED = 0,
    UART_PARITY_ENABLED  = 1
};

///UART Parity type
enum UART_PARITYBIT
{
    UART_PARITYBIT_EVEN  = 0,
    UART_PARITYBIT_ODD   = 1,
    UART_PARITYBIT_SPACE = 2, // The parity bit is always 0.
    UART_PARITYBIT_MARK  = 3  // The parity bit is always 1.
};

///UART HW flow control
enum UART_HW_FLOW_CNTL
{
    UART_HW_FLOW_CNTL_DISABLED = 0,
    UART_HW_FLOW_CNTL_ENABLED = 1
};

///UART Input clock select
enum UART_INPUT_CLK_SEL
{
    UART_INPUT_CLK_SEL_0 = 0,
    UART_INPUT_CLK_SEL_1 = 1,
    UART_INPUT_CLK_SEL_2 = 2,
    UART_INPUT_CLK_SEL_3 = 3
};

///UART Interrupt enable/disable
enum UART_INT
{
    UART_INT_DISABLE = 0,
    UART_INT_ENABLE = 1
};

///UART Error detection
enum UART_ERROR_DETECT
{
    UART_ERROR_DETECT_DISABLED = 0,
    UART_ERROR_DETECT_ENABLED  = 1
};

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
struct uart_txrxchannel
{
    /// call back function pointer
    uint8_t (*callback) (void*, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void* dummy;
};

/// UART environment structure
struct uart_env_tag
{
    /// tx channel
    struct uart_txrxchannel tx;
    /// rx channel
    struct uart_txrxchannel rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// uart environment structure
struct uart_env_tag uart_env;

char uart_buff[128];


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */	
uint16_t Read_Uart_Buf(uint8_t *buf, uint16_t len)
{
	memcpy(buf, &(uart_rx_buf[cur_read_buf_idx]), len);
	cur_read_buf_idx += len;
	return len;
}

uint8_t Uart_Read_Byte(void)
{
	return (REG_APB3_UART_PORT & 0xff);
}

int uart_putchar(char * st)
{
	uint8_t num = 0;
#if UART_PRINTF_EN
	while (*st)
	{
		if(UART_TX_WRITE_READY)
		{
			REG_APB3_UART_PORT = *st;
			st++;
			num++;
		}
	} 
#endif 		
	return num;
}

void ble_uart_flow_on(void)
{
	// Configure modem (HW flow control enable)
	// uart_flow_en_setf(0);
}

void uart_clear_rxfifo()
{
	while(uart_rx_fifo_rd_ready_getf())
	{
		Uart_Read_Byte();
	}
	memset(uart_rx_buf,0,UART0_RX_FIFO_MAX_COUNT);
	
}
bool ble_uart_flow_off(void)
{
	return true;
}

void uart_finish_transfers(void)
{
	uart_flow_en_setf(1);

	// Wait TX FIFO empty
	while(!uart_tx_fifo_empty_getf());
}


void ble_uart_read(uint8_t *bufptr, uint32_t size, uint8_t (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	BLE_ASSERT_ERR(bufptr != NULL);
	BLE_ASSERT_ERR(size != 0);
	BLE_ASSERT_ERR(callback != NULL);
	uart_env.rx.callback = callback;

	uart_env.rx.dummy    = dummy;
}



void ble_uart_write(uint8_t *bufptr, uint32_t size, uint8_t (*callback) (void*, uint8_t), void* dummy)
{
	// Sanity check
	BLE_ASSERT_ERR(bufptr != NULL);
	BLE_ASSERT_ERR(size != 0);
	BLE_ASSERT_ERR(callback != NULL);
	
	uint8_t len;
	len = size;
		
	uart_env.tx.callback = callback;
	uart_env.tx.dummy    = dummy;
	
	//Delay_ms(100);
	while(len--)
	{
		//cpu_delay(10);
		if(UART_TX_WRITE_READY)
		{
			REG_APB3_UART_PORT = *bufptr;
        	bufptr++;
    	}
	}
		
	if(callback != NULL)
	{
		// Clear callback pointer
		uart_env.tx.callback = NULL;
		uart_env.tx.dummy    = NULL;
		// Call handler
		callback(dummy, RWIP_EIF_STATUS_OK);
	}
}

static void uart_send_byte(unsigned char data)
{
	while (!uart_tx_fifo_empty_getf());
    
	REG_APB3_UART_PORT = data ;
}

void uart_send(unsigned char *buff, int len)
{
	while (len--)
		uart_send_byte(*buff++);
}

void uart_rx_cmd_handler(uint8_t *buff, uint8_t len)
{
	uint8_t length;
	length = len;
	switch (buff[0]) {
		case EXIT_DUT_CMD:
			if ((length == 2) && (buff[1] == EXIT_DUT_ACT)) {
				ble_send_msg(BLE_MSG_EXIT);
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case TX_PWR_SET_CMD:
			if (length == 2) {
				uint8_t pwr_idx = buff[1];
				if (pwr_idx > 31)
					pwr_idx = 31;
				extern void rwnx_cal_set_txpwr(UINT32 pwr_gain, UINT32 grate);
				rwnx_cal_set_txpwr(pwr_idx, EVM_DEFUALT_BLE_RATE);
				bk_printf("idx:%d\r\n", pwr_idx);

				if (ble_test_mode == USER_TX_MODE)
				{
					ble_ctrl(CMD_BLE_STOP_TX, NULL);
					ble_ctrl(CMD_BLE_START_TX, &tx_mode);
				}
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
				if (pwr_idx > 31)
					pwr_idx = 31;
				extern void manual_cal_save_txpwr(UINT32 rate, UINT32 channel, UINT32 pwr_gain);
				manual_cal_save_txpwr(EVM_DEFUALT_BLE_RATE, channel, pwr_idx);

			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case USER_SEND_CMD:
			if (length == 3) {
				uint8_t channel = buff[1];
				tx_mode = buff[2];
				if (channel > 39)
					channel = 39;

				void rwnx_cal_set_txpwr_by_channel(UINT32 channel);
				rwnx_cal_set_txpwr_by_channel(channel);

				uint32_t freq = (uint32_t)((channel + 1) * 2);

				ble_ctrl(CMD_BLE_SET_CHANNEL, &freq);
				ble_ctrl(CMD_BLE_START_TX, &tx_mode);

				ble_test_mode = USER_TX_MODE;
			} else {
				bk_printf("unknow dut cmd\r\n");
			}
		break;
		case USER_STOP_CMD:
			if (length == 2) {
				if (buff[1] == USER_SEND_CMD) {
					ble_ctrl(CMD_BLE_STOP_TX, NULL);
					ble_test_mode = IDLE_MODE;
				} else {
					bk_printf("unknow dut cmd");
				}
			} else {
				bk_printf("unknow dut cmd");
			}
		break;
		default:
		break;
	}
}

extern int bkreg_run_command(const char *content, int cnt);

void uart_isr(void)
{		
	uint32_t IntStat;

#if BLE_TESTER
	uint8_t (*callback) (void*, uint8_t) = NULL;
	void* data =NULL;
#endif

	IntStat = uart_isr_stat_get();
	if(uart_rx_fifo_need_rd_isr_getf() || uart_rx_end_isr_getf()|| uart_rxd_wakeup_isr_getf())
	{
		while((REG_APB3_UART_FIFO_STAT & (0x01 << 21)))
		{
			uart_rx_buf[uart_rx_index++] = UART_READ_BYTE();
			if( UART0_RX_FIFO_MAX_COUNT == uart_rx_index )
			{
				uart_rx_index = 0;
			}
		}

		if ((uart_rx_buf[0] == 0x01) && (uart_rx_buf[1] == 0xe0) && (uart_rx_buf[2] == 0xfc)) {
			if (uart_rx_buf[3] == (uart_rx_index - 4)) {
				uart_rx_cmd_handler((uint8_t *)&uart_rx_buf[4], uart_rx_buf[3]);
				bkreg_run_command((char*)&uart_rx_buf[0], uart_rx_index);
			}
		}

		uart_rx_done = 1;
		uart_rx_index = 0;
	}
#if BLE_TESTER
	{
		callback = uart_env.rx.callback;
		data = uart_env.rx.dummy;
		if(callback != NULL)
		{
			// Clear callback pointer
			uart_env.rx.callback = NULL;
			uart_env.rx.dummy    = NULL;

			// Call handler
			//UART_PRINTF("UART callback 0x%08x\r\n",callback);
	  	 	callback(data, RWIP_EIF_STATUS_OK);
		}
	}
#endif // BLE_TESTER

	uart_isr_stat_set(IntStat);
}

uint8_t check_uart_stop(void)
{
	return uart_tx_fifo_empty_getf();
}

/// @} UART
