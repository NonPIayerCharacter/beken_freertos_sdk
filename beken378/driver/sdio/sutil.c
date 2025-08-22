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
#include "arm_arch.h"

#include "uart_pub.h"
#include "sdio_pub.h"
#include "sdio.h"
#include "sutil.h"

#include "rwnx_config.h"

#if CFG_SDIO || CFG_SDIO_TRANS
void su_init(SDIO_PTR sdio_ptr)
{
    UINT32 i;
    SDIO_NODE_PTR node_ptr;

    INIT_LIST_HEAD(&sdio_ptr->free_nodes);

    for(i = 0; i < CELL_COUNT; i ++)
    {
        node_ptr = &sdio_ptr->snode[i];

        node_ptr->callback = NULLPTR;
        node_ptr->Lparam   = NULL;
        node_ptr->Rparam   = NULL;
        node_ptr->addr     = 0;
        node_ptr->node_list.next = NULLPTR;
        node_ptr->node_list.prev = NULLPTR;

        su_push_node(&sdio_ptr->free_nodes, node_ptr);
    }
}

void su_push_node(LIST_HEADER_T *head, SDIO_NODE_PTR node)
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    list_add_tail(&node->node_list, head);
    GLOBAL_INT_RESTORE();
}

SDIO_NODE_PTR su_pop_node(LIST_HEADER_T *head)
{
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;
    SDIO_NODE_PTR node;

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();

    node = NULLPTR;
    list_for_each_safe(pos, tmp, head)
    {
        list_del(pos);
        node = list_entry(pos, SDIO_NODE_T, node_list);

        break;
    }

    GLOBAL_INT_RESTORE();

    return node;
}

UINT32 su_get_node_count(LIST_HEADER_T *head)
{
    UINT32 count = 0;
    LIST_HEADER_T *tmp;
    LIST_HEADER_T *pos;

    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    list_for_each_safe(pos, tmp, head)
    {
        count ++;
    }


    GLOBAL_INT_RESTORE();
    return count;
}

UINT32 su_align_power2(UINT32 size)
{
    UINT32 i = 1;

    while(i < size)
    {
        i <<= 1;
    }

    return i;
}
#endif
// EOF

