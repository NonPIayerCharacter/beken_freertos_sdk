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

#ifndef _PWM_MUTEX_H_
#define _PWM_MUTEX_H_


#include "include.h"

#define TPWM_DEBUG
#undef TPWM_DEBUG

#ifdef TPWM_DEBUG
#define TPWM_PRT      os_printf
#define TPWM_WARN     warning_prf
#define TPWM_FATAL    fatal_prf
#else
#define TPWM_PRT      null_prf
#define TPWM_WARN     null_prf
#define TPWM_FATAL    null_prf
#endif

extern int bk_cw_pwm_reset_duty_cycle(uint8 channel_num_1, uint8 channel_num_2,
                                      uint32 duty_cycle_1, uint32 duty_cycle_2,
                                      uint32 end_value, uint32 dead_band_1);

#endif

///Eof

