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

#ifndef _UDEBUG_H_
#define _UDEBUG_H_

#define UD_DEBUG

#ifdef UD_DEBUG
#define UD_PRT       os_printf
#define UD_WPRT      warning_prf
#else
#define UD_PRT       os_null_printf
#define UD_WPRT      os_null_printf
#endif

#define BKREG_MIN_LEN                     3
#define BKREG_MAGIC_WORD0                 (0x01)
#define BKREG_MAGIC_WORD1                 (0xE0uc)
#define BKREG_MAGIC_WORD2                 (0xFCuc)

// largest one  01 e0 fc 09 03 xx xx xx xx xx xx xx xx
#define UART_PEEK_LEN                     13

//#define CONSOLE_NO_LOCAL_ECHO
#define UD_SUPPORT_UPKEY

#define UP_KEY_COUNT                      3
#define DEBUG_CHUNK_DEFAULT_COUNT         20*5
#define UDEBUG_PROMPT                     "\\BK7211\\UART_DEBUG>"

enum
{
    DEBUG_PRI_LEVEL_MIN,
    DEBUG_PRI_LEVEL_1,
    DEBUG_PRI_LEVEL_2,
    DEBUG_PRI_LEVEL_3,
    DEBUG_PRI_LEVEL_4,
    DEBUG_PRI_LEVEL_5,
    DEBUG_PRI_LEVEL_6,
    DEBUG_PRI_LEVEL_MAX
};

#endif // _UDEBUG_H_

// eof
