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

#ifndef _INCLUDES_H_
#define _INCLUDES_H_

#include "sys_config.h"
#include "typedef.h"
#include "generic.h"
#include "compiler.h"
#include "release.h"
#include "arch.h"
#include "bk_err.h"

#if CFG_ENABLE_DEMO_TEST
#include "demos_config.h"
#endif

#ifndef __maybe_unused
#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
#define __maybe_unused __attribute__((unused))
#else
#define __maybe_unused
#endif /* __GNUC__ */
#endif /* __maybe_unused */

#ifndef __maybe_unused_var
#define __maybe_unused_var(_var) do {\
	(void)(_var);\
} while(0)
#endif

#endif // _INCLUDES_H_
// eof
