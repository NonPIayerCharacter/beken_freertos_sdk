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

#ifndef _TEST_PWM_H_
#define _TEST_PWM_H_

#ifdef TUART_DEBUG
#define TUART_PRT  os_printf
#else
#define TUART_PRT  null_prf
#endif

#define UART_TEST_POART1		0
#define UART_TEST_POART2		1
#define UART_TEST_POART3		2
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

