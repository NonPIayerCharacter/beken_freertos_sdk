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

#ifndef _SDIO_UTIL_H_
#define _SDIO_UTIL_H_

/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void su_init(SDIO_PTR sdio_ptr);

extern void su_push_node(LIST_HEADER_T *head, SDIO_NODE_PTR node);

extern SDIO_NODE_PTR su_pop_node(LIST_HEADER_T *head);

extern UINT32 su_get_node_count(LIST_HEADER_T *head);

extern UINT32 su_align_power2(UINT32 size);

#endif // _SDIO_UTIL_H_
