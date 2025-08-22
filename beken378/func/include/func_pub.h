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

#ifndef _FUNC_PUB_H_
#define _FUNC_PUB_H_

#define FUNC_DEBUG
#ifdef FUNC_DEBUG
#define FUNC_PRT                 os_printf
#define FUNC_WPRT                warning_prf
#else
#define FUNC_PRT                 os_null_printf
#define FUNC_WPRT                os_null_printf
#endif

extern UINT32 func_init_extended(void);
extern UINT32 func_init_basic(void);
#endif // _FUNC_PUB_H_
// eof

