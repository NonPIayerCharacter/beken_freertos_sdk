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

#ifndef _BK_EASY_FLASH_H_
#define _BK_EASY_FLASH_H_

#include "easyflash.h"

#if CFG_EASY_FLASH
EfErrCode bk_save_env(void);
char *bk_get_env(const char *key);
EfErrCode bk_set_env(const char *key, const char *value);
EfErrCode bk_set_buf_env(const char *key, const char *buf, int len);
EfErrCode bk_get_buf_env(const char *key, const char *buf, int len);
#endif // CFG_EASY_FLASH

#endif // _BK_EASY_FLASH_H_
// eof

