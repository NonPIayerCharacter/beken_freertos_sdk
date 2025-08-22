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

#include "demos_start.h"
#include "demos_config.h"
#include "demos_case.h"

#if CFG_ENABLE_DEMO_TEST
void application_start(void)
{
    demo_start();
}
#endif // CFG_ENABLE_DEMO_TEST
// eof

