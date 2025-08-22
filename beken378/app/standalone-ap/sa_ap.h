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

#ifndef _SA_AP_H_
#define _SA_AP_H_

#define SAAP_DEBUG

#ifdef SAAP_DEBUG
#define SAAP_PRT                 os_printf
#define SAAP_WPRT                warning_prf
#else
#define SAAP_PRT                 os_null_printf
#define SAAP_WPRT                os_null_printf
#endif

extern void sa_ap_init(void);
#endif // _SA_AP_H_
// eof

