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

#ifndef DRIVER_AUDIO_IF_PUB_H
#define DRIVER_AUDIO_IF_PUB_H

extern void aud_open(void);
extern void aud_close(void);
extern uint8_t aud_get_channel(void);
extern uint16_t aud_get_buffer_size(void);
extern void  aud_fill_buffer( uint8_t *buff, uint16_t len );
extern uint16_t aud_get_buffer_length(uint8_t *buff, uint16_t len);
extern void aud_initial(uint32_t freq, uint32_t channels, uint32_t bits_per_sample);
extern uint16_t aud_get_fill_size(void);
extern int32_t aud_hw_init(void);
extern uint8_t is_aud_opened(void);
#endif
