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

#ifndef _HOSTAPD_INTF_H_
#define _HOSTAPD_INTF_H_

#define HINTF_SUCCESS           (0)
#define HINTF_FAILURE           (-1)

#define BCN_TIM_IE_LEN                   6

#define WPAS_DEBUG

#ifdef WPAS_DEBUG
#define WPAS_PRT       os_printf
#define WPAS_WPRT      warning_prf
#else
#define WPAS_PRT       os_null_printf
#define WPAS_WPRT      warning_prf
#endif

#endif
// eof

