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

#ifndef _ATSVR_PORT_H_
#define _ATSVR_PORT_H_

#define ATSVR_QUEUE_MAX_NB              6

#define AT_UART_PORT_CFG                UART1_PORT
#define AT_CTS_RTS_SOFTWARE_CFG         1
#define AT_UART_SEND_DATA_INTTRRUPT_PROTECT   0


#define IN
#define OUT
#define CONST const
#define STRING

extern void set_at_uart_overflow(int enable);
extern int get_at_uart_overflow(void);

extern void atsvr_overflow_handler(void);

extern int IN atsvr_input_char(unsigned char *buf);
extern int atsvr_get_size_rxbuf();
extern void atsvr_copy_lenth_rxbuf(char *buf,int lenth);
extern void atsvr_clear_size_rxbuf();

#endif

