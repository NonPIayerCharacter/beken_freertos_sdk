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

#ifndef _SDIO_INTF_PUB_H_
#define _SDIO_INTF_PUB_H_

#include "sdio_pub.h"
#include "ke_msg.h"
#include "tx_swdesc.h"

#define SDIO_INTF_FAILURE        ((UINT32)-1)
#define SDIO_INTF_SUCCESS        (0)

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern UINT32 sdio_intf_init(void);
extern void sdio_emb_rxed_evt(int dummy);
extern UINT32 outbound_upload_data(UINT8 *buf_ptr, UINT32 len);
extern UINT32 sdio_get_free_node(UINT8 **buf_pptr, UINT32 buf_size);
extern void sdio_emb_rxed_evt(int dummy);
extern void inbound_cfm(void);
extern UINT32 sdio_emb_kmsg_fwd(struct ke_msg *msg);

extern UINT32 sdio_get_free_node_count(void);
extern UINT32 sdio_release_one_node(SDIO_NODE_PTR mem_node_ptr);

extern void sdio_trans_evt(int dummy);
extern SDIO_NODE_PTR sdio_get_rxed_node(void);
extern int sdio_trans_init(void);

#endif // _SDIO_INTF_PUB_H_


