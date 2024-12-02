#ifndef _TEST_PWM_H_
#define _TEST_PWM_H_

#ifdef TUART_DEBUG
#define TUART_PRT  os_printf
#else
#define TUART_PRT  null_prf
#endif

#define UART_TEST_POART1		0
#define UART_TEST_POART2		1
#define UART_TX_BUFFER_SIZE		1024
#define UART_RX_BUFFER_SIZE		1024*2
#define UART_RX_DMA_CHANNEL     GDMA_CHANNEL_1
#define UART_TX_DMA_CHANNEL     GDMA_CHANNEL_3

extern void uart_test_send(void);
extern void uart_dma_test_send(void);
extern void uart_dma_test_recv(void);

#define IN
#define OUT
#define CONST const
#endif /*_TEST_PWM_H_*/
// eof

