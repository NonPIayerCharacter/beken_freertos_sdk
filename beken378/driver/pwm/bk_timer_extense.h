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


UINT32 bk_timer_disable(UINT32 channel);
UINT32 bk_timer_get_int_status(UINT32 channel, UINT32 *status);
UINT32 bk_timer_get_end_time(UINT32 channel, UINT32 *count_ms);
UINT32 bk_timer_enable(UINT32 channel, UINT32 delay_ms);
UINT32 bk_timer_measure(UINT32 channel, UINT32 *count_ms);
UINT32 bk_timer_pre_measure(UINT32 channel);


