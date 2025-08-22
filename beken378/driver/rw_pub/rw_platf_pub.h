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

#ifndef _RW_PLATF_PUB_H_
#define _RW_PLATF_PUB_H_

void rwxl_reset_patch(void);
void hal_machw_init_diagnostic_ports(void);
void hal_machw_before_reset_patch(void);
void hal_machw_after_reset_patch(void);

#endif
