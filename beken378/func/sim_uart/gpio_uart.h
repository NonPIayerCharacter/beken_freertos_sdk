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

#ifndef _GPIO_UART_H_
#define _GPIO_UART_H_

#define CONFIG_GPIO_SIMU_UART_RX 1
#define CONFIG_GPIO_SIMU_UART_TX 1


#define SIMU_UART_GPIONUM             18
#define SIMU_UART_GPIO_RX             19
#define SIMU_UART_GPIO_TEST           15

extern void gpio_isr(void);
extern void gu_delay(uint32_t ) ;
#ifdef CONFIG_GPIO_SIMU_UART_TX
extern void gpio_uart_send_init(void);
extern int guart_fputc(int, FILE *);
#endif

#ifdef CONFIG_GPIO_SIMU_UART_RX
extern void gpio_uart_recv_init(void);
#endif

#endif // _GPIO_UART_H_
// eof

