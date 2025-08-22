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

#ifndef _TEST_PSRAM_H_
#define _TEST_PSRAM_H_
#include "uart_pub.h"

#define PSRAM_DEBUG        1

#ifdef PSRAM_DEBUG
#define PSRAM_PRT  os_printf
#else
#define PSRAM_PRT  null_prf
#endif

#define MODE_PSRAM_SINGLE              0
#define MODE_PSRAM_QUAD                3

#define QSPI_DCACHE_START_ADDR        (0x03000000)
#define PSRAM_TEST_START_ADDR         (QSPI_DCACHE_START_ADDR)
#define PSRAM_TEST_LEN                (2048)

#endif /*_TEST_PSRAM_H_*/
// eof

