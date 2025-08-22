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

#ifndef __AUDIO_INTF_H__
#define __AUDIO_INTF_H__

extern UINT32 audio_intf_init(void);
extern void audio_intf_dac_set_volume(void);
extern void audio_intf_adc_play(void);
extern void audio_intf_dac_play(void);
extern void audio_intf_dac_pause(void);
extern void audio_intf_adc_pause(void);
extern void audio_intf_set_sample_rate(void);

#endif
