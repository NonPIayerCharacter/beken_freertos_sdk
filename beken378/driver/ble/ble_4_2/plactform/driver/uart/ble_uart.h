/**
 ****************************************************************************************
 *
 * @file ble_uart.h
 *
 * @brief UART Driver for HCI over UART operation.
 *
 * Copyright (C) Beken 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _BLE_UART_H_
#define _BLE_UART_H_

/**
 ****************************************************************************************
 * @defgroup UART UART
 * @ingroup DRIVERS
 * @brief UART driver
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdbool.h>          // standard boolean definitions
#include <stdint.h>           // standard integer functions
#include "rwip_config.h"
#include "user_config.h"

#if UART_PRINTF_EN
#define UART_PRINTF bk_printf //uart_printf 
#else
#define UART_PRINTF(...)
#endif // #if UART_PRINTF_EN

enum
{
	EXIT_DUT_CMD			= 0x0E,
	EXIT_DUT_ACT			= 0xA0,
	TX_PWR_SET_CMD			= 0x10,
	TX_PWR_SAVE_CMD			= 0x11,
	XTAL_SET_CMD			= 0x12,
	USER_SEND_CMD			= 0x21,
	USER_STOP_CMD			= 0x2F,
};

enum
{
	IDLE_MODE			= 0,
	USER_TX_MODE		= 1,
	DUT_MODE			= 2,
};

#define uart_printf bk_printf

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the UART to default values.
 *****************************************************************************************
 */
void uart_init(uint32_t baudrate);

void dbg_initial(void);

void uart_clear_rxfifo(void);

uint8_t Uart_Read_Byte(void);
uint16_t Read_Uart_Buf(uint8_t *buf, uint16_t len);

int dbg_putchar(char * st);
int uart_putchar(char * st);
int dbg_printf(const char *fmt,...);
void uart_print_int(unsigned int num);
uint8_t check_uart_stop(void);

void cpu_delay( volatile unsigned int times);


/****** REG  ****************/
void uart_send(unsigned char *buff, int len);
void TRAhcit_UART_Rx(void);

#define UART0_RX_FIFO_MAX_COUNT  128

#define UART0_TX_FIFO_MAX_COUNT  128

extern unsigned char uart_rx_buf[UART0_RX_FIFO_MAX_COUNT];
extern unsigned char uart_tx_buf[UART0_TX_FIFO_MAX_COUNT];
extern volatile bool uart_rx_done ;
extern volatile unsigned long uart_rx_index ;
/****** REG  ****************/



#ifndef CFG_ROM
/**
 ****************************************************************************************
 * @brief Enable UART flow.
 *****************************************************************************************
 */
void uart_flow_on(void);

/**
 ****************************************************************************************
 * @brief Disable UART flow.
 *****************************************************************************************
 */
bool uart_flow_off(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Finish current UART transfers
 *****************************************************************************************
 */
void uart_finish_transfers(void);

/**
 ****************************************************************************************
 * @brief Starts a data reception.
 *
 * @param[out] bufptr   Pointer to the RX buffer
 * @param[in]  size     Size of the expected reception
 * @param[in]  callback Pointer to the function called back when transfer finished
 * @param[in]  dummy    Dummy data pointer returned to callback when reception is finished
 *****************************************************************************************
 */
void uart_read(uint8_t *bufptr, uint32_t size, uint8_t (*callback) (void*, uint8_t), void* dummy);

/**
 ****************************************************************************************
 * @brief Starts a data transmission.
 *
 * @param[in] bufptr   Pointer to the TX buffer
 * @param[in] size     Size of the transmission
 * @param[in] callback Pointer to the function called back when transfer finished
 * @param[in] dummy    Dummy data pointer returned to callback when transmission is finished
 *****************************************************************************************
 */
void uart_write(uint8_t *bufptr, uint32_t size, uint8_t (*callback) (void*, uint8_t), void* dummy);

#if defined(CFG_ROM)
/**
 ****************************************************************************************
 * @brief Poll UART on reception and transmission.
 *
 * This function is used to poll UART for reception and transmission.
 * It is used when IRQ are not used to detect incoming bytes.
 *****************************************************************************************
 */
void uart_poll(void);
#endif //CFG_ROM

/**
 ****************************************************************************************
 * @brief Serves the data transfer interrupt requests.
 *
 * It clears the requests and executes the appropriate callback function.
 *****************************************************************************************
 */
void uart_isr(void);


/// @} UART
#endif /* _BLE_UART_H_ */
