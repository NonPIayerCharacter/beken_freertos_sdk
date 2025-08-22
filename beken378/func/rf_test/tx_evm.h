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

#ifndef _TX_EVM_H_
#define _TX_EVM_H_

#define EVM_DEBUG

#ifdef EVM_DEBUG
#define EVM_PRT       os_printf
#define EVM_WPRT      warning_prf
#else
#define EVM_PRT       os_null_printf
#define EVM_WPRT      os_null_printf
#endif

#define EVM_DEFAULT_CHANNEL         2437

/*******************************************************************************
* Function Declarations
*******************************************************************************/
void evm_bypass_mac(void);
uint32_t evm_via_mac_is_start(void);

#endif // _TX_EVM_H_
// eof

