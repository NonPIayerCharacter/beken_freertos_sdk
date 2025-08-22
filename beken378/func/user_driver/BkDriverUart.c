// Copyright 2015-2024 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "include.h"
#include "rtos_pub.h"
#include "BkDriverUart.h"
#include "drv_model_pub.h"
#include "error.h"

typedef struct
{
    UINT8 user_buffer_flag;
    ring_buffer_t *rx_buffer;
} UART_UI_ST;

#if !(CFG_SOC_NAME == SOC_BK7252N)
static UART_UI_ST uart_st[2] =
#else
static UART_UI_ST uart_st[3] =
#endif
{
    [0]  = {.user_buffer_flag = 0, .rx_buffer = NULL},
    [1]  = {.user_buffer_flag = 0, .rx_buffer = NULL},
    #if ((CFG_SOC_NAME == SOC_BK7252N))
    [2]  = {.user_buffer_flag = 0, .rx_buffer = NULL},
    #endif
};

const bk_uart_config_t test_uart_config[] =
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

    #if ((CFG_SARADC_VERIFY) && (CFG_SOC_NAME == SOC_BK7252N))
    [3] =
    {
        .baud_rate     = 921600,
        .data_width    = BK_DATA_WIDTH_8BIT,
        .parity  = BK_PARITY_NO,
        .stop_bits = BK_STOP_BITS_1,
        .flow_control = FLOW_CTRL_DISABLED,
        .flags   = 0,
    },
    #endif
};

OSStatus bk_uart_initialize_test( bk_uart_t uart, uint8_t config, ring_buffer_t *optional_rx_buffer )
{
    return bk_uart_initialize(uart, &test_uart_config[config], optional_rx_buffer);
}

OSStatus bk_uart_initialize( bk_uart_t uart, const bk_uart_config_t *config, ring_buffer_t *optional_rx_buffer )
{
    UINT32 ret;
    UINT32 status;
    DD_HANDLE uart_hdl;

    if(BK_UART_1 == uart)
        uart_hdl = ddev_open(UART1_DEV_NAME, &status, 0);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    else if(BK_UART_3 == uart)
        uart_hdl = ddev_open(UART3_DEV_NAME, &status, 0);
    #endif
    else
        uart_hdl = ddev_open(UART2_DEV_NAME, &status, 0);

    ASSERT(DRV_FAILURE != uart_hdl);
    ASSERT(uart < BK_UART_MAX);

    if(optional_rx_buffer == NULL)
    {
        uart_st[uart].rx_buffer = NULL;
        uart_st[uart].user_buffer_flag = 0;
    }
    else
    {
        uart_st[uart].rx_buffer = optional_rx_buffer;
        uart_st[uart].user_buffer_flag = 1;
    }
    ret = ddev_control(uart_hdl, CMD_UART_INIT, (void *)config);
    return ret;
}

OSStatus bk_uart_finalize( bk_uart_t uart )
{
    return kNoErr;
}

OSStatus bk_uart_send( bk_uart_t uart, const void *data, uint32_t size )
{
    bk_send_string(uart, data);

    return kNoErr;
}

OSStatus bk_uart_recv( bk_uart_t uart, void *data, uint32_t size, uint32_t timeout )
{
    UINT32 ret;
    UINT32 status;
    DD_HANDLE uart_hdl;

    if(BK_UART_1 == uart)
        uart_hdl = ddev_open(UART1_DEV_NAME, &status, 0);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    else if(BK_UART_3 == uart)
        uart_hdl = ddev_open(UART3_DEV_NAME, &status, 0);
    #endif
    else
        uart_hdl = ddev_open(UART2_DEV_NAME, &status, 0);

    ASSERT(DRV_FAILURE != uart_hdl);
    ret = ddev_control(uart_hdl, CMD_RX_COUNT, 0);
    if(ret < size)
    {
        return kGeneralErr;
    }

    ret = ddev_read(uart_hdl, data, size, 0);
    ASSERT(size == ret);

    return kNoErr;
}

OSStatus bk_uart_recv_prefetch( bk_uart_t uart, void *data, uint32_t size, uint32_t timeout )
{
    UINT32 ret;
    UINT32 status;
    DD_HANDLE uart_hdl;
    UART_PEEK_RX_T peek;

    if(BK_UART_1 == uart)
        uart_hdl = ddev_open(UART1_DEV_NAME, &status, 0);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    else if(BK_UART_3 == uart)
        uart_hdl = ddev_open(UART3_DEV_NAME, &status, 0);
    #endif
    else
        uart_hdl = ddev_open(UART2_DEV_NAME, &status, 0);

    ASSERT(DRV_FAILURE != uart_hdl);

    peek.sig = URX_PEEK_SIG;
    peek.ptr = data;
    peek.len = size;
    ret = ddev_control(uart_hdl, CMD_RX_PEEK, &peek);
    ASSERT(size == ret);

    return kNoErr;
}

uint32_t bk_uart_get_length_in_buffer( bk_uart_t uart )
{
    uint32_t length;
    UINT32 status;
    DD_HANDLE uart_hdl;

    if(BK_UART_1 == uart)
        uart_hdl = ddev_open(UART1_DEV_NAME, &status, 0);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    else if(BK_UART_3 == uart)
        uart_hdl = ddev_open(UART3_DEV_NAME, &status, 0);
    #endif
    else
        uart_hdl = ddev_open(UART2_DEV_NAME, &status, 0);

    ASSERT(DRV_FAILURE != uart_hdl);
    length = (uint32_t)ddev_control(uart_hdl, CMD_RX_COUNT, 0);

    return length;
}

OSStatus bk_uart_set_rx_callback(bk_uart_t uart, uart_callback callback, void *param)
{
    UINT32 ret;
    UINT32 status;
    DD_HANDLE uart_hdl;
    UART_CALLBACK_RX_T uart_callback_rx;

    if(BK_UART_1 == uart)
        uart_hdl = ddev_open(UART1_DEV_NAME, &status, 0);
    #if (CFG_SOC_NAME == SOC_BK7252N)
    else if(BK_UART_3 == uart)
        uart_hdl = ddev_open(UART3_DEV_NAME, &status, 0);
    #endif
    else
        uart_hdl = ddev_open(UART2_DEV_NAME, &status, 0);

    ASSERT(DRV_FAILURE != uart_hdl);

    uart_callback_rx.callback = callback;
    uart_callback_rx.param = param;
    ret = ddev_control(uart_hdl, CMD_UART_SET_RX_CALLBACK, &uart_callback_rx);
    ASSERT(UART_SUCCESS == ret);

    return kNoErr;
}

// eof

