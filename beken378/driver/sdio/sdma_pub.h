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

#ifndef _SDMA_PUB_H_
#define _SDMA_PUB_H_

#define SDMA_INTERACT_WITH_HOST

typedef void (*TX_FUNC)(void);
typedef void (*CMD_FUNC)(void *buf, UINT32 len);
typedef void (*RX_FUNC)(UINT32 count);

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void sdma_init(void);

extern void sdma_open(void);

extern void sdma_uninit(void);

extern void sdma_close(void);

#ifdef SDMA_INTERACT_WITH_HOST
extern UINT32 sdma_get_blk_len(void);
extern void sdma_set_tx_valid(void);
extern void sdma_clr_tx_valid(void);
extern void sdma_set_tx_dat_count(UINT32 val);
#endif // SDMA_INTERACT_WITH_HOST

extern void sdma_register_handler(TX_FUNC tx_callback,
                                  RX_FUNC rx_callback,
                                  CMD_FUNC cmd_callback);

extern UINT32 sdma_start_rx(UINT8 *buf, UINT32 len);
extern UINT32 sdma_start_tx(UINT8 *buf, UINT32 len);

extern UINT32 sdma_start_cmd(UINT8 *cmd, UINT32 len);
extern void sdma_fake_stop_dma(void);
extern void sdma_isr(void);
#endif // _SDMA_PUB_H_
