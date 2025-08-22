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

#include <stdlib.h>
#include <stdint.h>
#include <assert.h>

extern uint32_t platform_is_in_irq_context( void );
extern uint32_t platform_is_in_fiq_context( void );

uint32_t platform_is_in_interrupt_context( void )
{
    return ((platform_is_in_fiq_context())
            || (platform_is_in_irq_context()));
}
// eof

