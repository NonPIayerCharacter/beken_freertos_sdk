#include "include.h"
#include "arm_arch.h"

#define TEST_UART_DMA_DEMO
#ifdef 	TEST_UART_DMA_DEMO
#include "string.h"
#include "stdlib.h"
#include "mem_pub.h"
#include "uart_pub.h"
#include "uart.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "rtos_pub.h"
#include "error.h"
#include "pwm_pub.h"
#include "test_uart.h"
#include "general_dma_pub.h"
#include "general_dma.h"
#include "BkDriverUart.h"

static volatile int dma_trans_flag = 0;
struct uart_message
{	
	UINT32 send_len;
	UINT32 recv_len;
	UINT8 *send_buf;	
	UINT16 *recv_buf;
};

const bk_uart_config_t uart1_config[] =
{
    [0] =
    {
        .baud_rate     = 115200,
        .data_width    = BK_DATA_WIDTH_8BIT,
        .parity  = BK_PARITY_NO,
        .stop_bits = BK_STOP_BITS_1,
        .flow_control = FLOW_CTRL_DISABLED,
        .flags   = 0,
    },
    
    [1] =
    {
        .baud_rate     = 19200,
        .data_width    = BK_DATA_WIDTH_8BIT,
        .parity  = BK_PARITY_NO,
        .stop_bits = BK_STOP_BITS_1,
        .flow_control = FLOW_CTRL_DISABLED,
        .flags   = 0,
    },
    
    [2] =
    {
        .baud_rate     = 115200,
        .data_width    = BK_DATA_WIDTH_8BIT,
        .parity  = BK_PARITY_EVEN,
        .stop_bits = BK_STOP_BITS_1,
        .flow_control = FLOW_CTRL_DISABLED,
        .flags   = 0,
    },
};

ring_buffer_t *ring_buf;

struct uart_message uart_msg;

void uart_test_send(void)
{
	struct uart_message msg;
	int i,ret = 0;

	msg.recv_len = UART_TX_BUFFER_SIZE;
	msg.send_len = UART_TX_BUFFER_SIZE;

	msg.recv_buf =os_malloc(UART_TX_BUFFER_SIZE * sizeof(msg.recv_buf[0]));
	if(msg.recv_buf == 0)
	{
		os_printf("msg.recv_buf malloc failed\r\n");
		return;
	}

	msg.send_buf = os_malloc(UART_TX_BUFFER_SIZE * sizeof(msg.send_buf[0]));
	if(msg.send_buf == 0)
	{
		os_printf("msg.send_buf malloc failed\r\n");
		return;
	}
	ring_buf->buffer = msg.send_buf;

	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		msg.send_buf[i] = 0x01 + i;
	}

	ret =  bk_uart_initialize(UART_TEST_POART1, &uart1_config[0], ring_buf);
	if (ret != kNoErr)
	{
		os_printf("init failed\r\n");
	}

	bk_uart_send(UART_TEST_POART1, msg.send_buf, UART_TX_BUFFER_SIZE);

	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		os_printf("send_buf[%d] =0x%x\r\n",i,msg.send_buf[i]);
	}	
}

void uart_test_recv(void)
{
	struct uart_message msg;
	int i,ret = 0;

	msg.recv_len = UART_TX_BUFFER_SIZE;
	msg.send_len = UART_TX_BUFFER_SIZE;

	msg.recv_buf =os_malloc(UART_TX_BUFFER_SIZE * sizeof(msg.recv_buf[0]));
	if(msg.recv_buf == 0)
	{
		os_printf("msg.recv_buf malloc failed\r\n");
		return;
	}

	msg.send_buf = os_malloc(UART_TX_BUFFER_SIZE * sizeof(msg.send_buf[0]));
	if(msg.send_buf == 0)
	{
		os_printf("msg.send_buf malloc failed\r\n");
		return;
	}

	ring_buf->buffer = msg.send_buf;

	ret =  bk_uart_initialize(UART_TEST_POART1, &uart1_config[0], ring_buf);
	if (ret != kNoErr)
	{
		os_printf("init failed\r\n");
	}

	for(i=0; i<UART_RX_BUFFER_SIZE; i++)
	{
		os_printf("send_buf[%d] =0x%x\r\n",i,msg.recv_buf[i]);
	}	
}

#if 1
static void uart_dma_tx_half_handler(UINT32 param)
{
	os_printf("uart_dma half hander\r\n");
}

static void uart_dma_tx_finish_handler(UINT32 param)
{
	os_printf("uart_dma finish hander\r\n");
}
static void uart_dma_rx_half_handler(UINT32 param)
{
	dma_trans_flag |= 1;
	os_printf("uart_dma rx half hander\r\n");
}

static void uart_dma_rx_finish_handler(UINT32 param)
{
	dma_trans_flag |=2;
	os_printf("uart_dma rx finish hander\r\n");
}

void uart_debug_prt(void)
{
	int reg_addr= 0;

	// wf debug
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x08)* 4));
	TUART_PRT("reg08:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x09)* 4));
	TUART_PRT("reg09:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x0a)* 4));
	TUART_PRT("reg0a:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x1b)* 4));
	TUART_PRT("reg0b:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x0c)* 4));
	TUART_PRT("reg0c:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x0d)* 4));
	TUART_PRT("reg0d:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x0e)* 4));
	TUART_PRT("reg0e:0x%lx\r\n", reg_addr);
	reg_addr = REG_READ(GENER_DMA0_REG0_CONF + ((0x0f)* 4));
	TUART_PRT("reg0f:0x%lx\r\n", reg_addr);

	(void)reg_addr;
}

int uart_dma_tx_init(void)
{	
    GDMACFG_TPYES_ST init_cfg;
    GDMA_CFG_ST en_cfg;

	os_printf("dma init\r\n");
    os_memset(&init_cfg, 0, sizeof(GDMACFG_TPYES_ST));
    os_memset(&en_cfg, 0, sizeof(GDMA_CFG_ST));

    init_cfg.dstdat_width = 8;
    init_cfg.srcdat_width = 32;
    init_cfg.dstptr_incr =  0;
    init_cfg.srcptr_incr =  1;

    init_cfg.src_start_addr = uart_msg.send_buf;
    init_cfg.dst_start_addr = (void *)REG_UART1_FIFO_PORT;

    init_cfg.channel = UART_TX_DMA_CHANNEL;
    init_cfg.prio = 0;
    init_cfg.u.type4.src_loop_start_addr = uart_msg.send_buf;
    init_cfg.u.type4.src_loop_end_addr = uart_msg.send_buf + UART_TX_BUFFER_SIZE;

    init_cfg.half_fin_handler = uart_dma_tx_half_handler;
    init_cfg.fin_handler = uart_dma_tx_finish_handler;

    init_cfg.src_module = GDMA_X_SRC_DTCM_RD_REQ;
    init_cfg.dst_module = GDMA_X_DST_UART1_TX_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE4, (void *)&init_cfg);

    en_cfg.channel = UART_TX_DMA_CHANNEL;
    en_cfg.param = UART_TX_BUFFER_SIZE; 		// dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);
    en_cfg.channel = UART_TX_DMA_CHANNEL;
    en_cfg.param = 0; 							// 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);

	return 0;
}

int uart_dma_rx_init(void)
{
    GDMACFG_TPYES_ST init_cfg;
    GDMA_CFG_ST en_cfg;

	os_printf("dma rx init\r\n");
    os_memset(&init_cfg, 0, sizeof(GDMACFG_TPYES_ST));
    os_memset(&en_cfg, 0, sizeof(GDMA_CFG_ST));

    init_cfg.dstdat_width = 32;
    init_cfg.srcdat_width = 16;
    init_cfg.dstptr_incr =  1;
    init_cfg.srcptr_incr =  0;

    init_cfg.src_start_addr =  (void *)REG_UART1_FIFO_PORT;
    init_cfg.dst_start_addr = uart_msg.recv_buf;

    init_cfg.channel = UART_RX_DMA_CHANNEL;
    init_cfg.prio = 0;
    init_cfg.u.type4.src_loop_start_addr = uart_msg.recv_buf;
    init_cfg.u.type4.src_loop_end_addr = uart_msg.recv_buf + UART_RX_BUFFER_SIZE;

    init_cfg.half_fin_handler = uart_dma_rx_half_handler;
    init_cfg.fin_handler = uart_dma_rx_finish_handler;

    init_cfg.src_module = GDMA_X_SRC_UART1_RX_REQ;
    init_cfg.dst_module = GDMA_X_DST_DTCM_WR_REQ;

    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_TYPE5, (void *)&init_cfg);

    en_cfg.channel = UART_RX_DMA_CHANNEL;
    en_cfg.param   = UART_RX_BUFFER_SIZE; 		// dma translen
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_TRANS_LENGTH, (void *)&en_cfg);

    en_cfg.channel = UART_RX_DMA_CHANNEL;
    en_cfg.param = 1; 							// 0:not repeat 1:repeat
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_CFG_WORK_MODE, (void *)&en_cfg);

	return 0;
}

static void dma_tx_enable(UINT8 enable)
{
    GDMA_CFG_ST en_cfg;

	os_printf("dma enable\r\n");
	en_cfg.channel = UART_TX_DMA_CHANNEL;

    if (enable)
        en_cfg.param = 1;
    else
        en_cfg.param = 0;
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
}

static void dma_rx_enable(UINT8 enable)
{
    GDMA_CFG_ST en_cfg;

	en_cfg.channel = UART_RX_DMA_CHANNEL;

    if (enable)
        en_cfg.param = 1;
    else
        en_cfg.param = 0;
    sddev_control(GDMA_DEV_NAME, CMD_GDMA_SET_DMA_ENABLE, &en_cfg);
}

void uart_dma_test_send(void)
{
	int i,ret = 0;

	os_printf("uart dma test\r\n");

	uart_msg.recv_len = UART_TX_BUFFER_SIZE;
	uart_msg.send_len = UART_TX_BUFFER_SIZE;

	ring_buf->buffer = uart_msg.send_buf;

	uart_msg.recv_buf =os_malloc(UART_TX_BUFFER_SIZE * sizeof(uart_msg.recv_buf[0]));
	if(uart_msg.recv_buf == 0)
	{
		os_printf("uart_msg.recv_buf malloc failed\r\n");
	}

	uart_msg.send_buf = os_malloc(UART_TX_BUFFER_SIZE * sizeof(uart_msg.send_buf[0]));
	if(uart_msg.send_buf == 0)
	{
		os_printf("uart_msg.send_buf malloc failed\r\n");
	}

	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		uart_msg.send_buf[i] = 0x00 + i;
	}

	ret =  bk_uart_initialize(UART_TEST_POART1, &uart1_config[0], ring_buf);
	if (ret != kNoErr)
	{
		os_printf("init failed\r\n");
	}

	uart_dma_tx_init();
	dma_tx_enable(1);


	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		//os_printf("send_buf[%d] =0x%x\r\n",i,uart_msg.send_buf[i]);
	}
}
/**********uart fifo 为16 bit：0~7 发送fifo 8~15 ：接收fifo, 
 **********所以DMA接收宽度的时候必须设置为16bit,发送宽度为8bit 
 **********相应的接收数据长度为send 长度的2倍, recv数据也取8~15bit 
***********/
void uart_dma_test_recv(void)
{
	int i,ret = 0;
	dma_trans_flag = 0;

	os_printf("uart dma test\r\n");

	uart_msg.recv_len = UART_RX_BUFFER_SIZE;

	ring_buf->buffer = (uint8_t*)(uart_msg.recv_buf);

	uart_msg.recv_buf =os_malloc(UART_TX_BUFFER_SIZE * sizeof(uart_msg.recv_buf[0]));
	if(uart_msg.recv_buf == 0)
	{
		os_printf("uart_msg.recv_buf malloc failed\r\n");
	}

	uart_msg.send_buf = os_malloc(UART_TX_BUFFER_SIZE * sizeof(uart_msg.send_buf[0]));
	if(uart_msg.send_buf == 0)
	{
		os_printf("uart_msg.send_buf malloc failed\r\n");
	}

	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		uart_msg.send_buf[i] = 0x00 + i;
	}

	ret =  bk_uart_initialize(UART_TEST_POART1, &uart1_config[0], ring_buf);
	if (ret != kNoErr)
	{
		os_printf("init failed\r\n");
	}

	uart_dma_rx_init();
	dma_rx_enable(1);
	while(dma_trans_flag !=3)
	{
	}

	dma_rx_enable(0);

	for(i=0; i<UART_TX_BUFFER_SIZE; i++)
	{
		os_printf("recv_buf[%d] =0x%x\r\n",i,uart_msg.recv_buf[i]>>8);
	}


	if((uart_msg.recv_buf != NULL)|| (uart_msg.send_buf != NULL)   )
	{
		os_free(uart_msg.recv_buf);
		os_free(uart_msg.send_buf);
	}
}

#endif

#endif /* TEST_UART_DMA_DEMO */
// EOF

