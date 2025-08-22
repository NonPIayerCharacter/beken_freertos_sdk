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

#ifndef _TLS_RTOS_H_
#define _TLS_RTOS_H_



#include "rtos_pub.h"
#include "mem_pub.h"

#define os_calloc(nmemb,size)   ((size) && (nmemb) > (~( unsigned int) 0)/(size))?0:os_zalloc((nmemb)*(size))

#define TLS_EOK            0
#define TLS_ERROR          (-1)




#define TLS_NULL          0U

#endif

