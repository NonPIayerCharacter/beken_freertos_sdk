// Copyright 2024-2025 Beken
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


#ifndef __PLUGIN_MANAGER_H__
#define __PLUGIN_MANAGER_H__
#include "player_osal.h"
#include "play_manager.h"
#include "linked_list.h"
#include "source_api.h"
#include "codec_api.h"
#include "sink_api.h"

int plugin_init(void);
void plugin_deinit(void);

int audio_source_register(audio_source_ops_t *ops);
list *audio_sources_get(void);

int audio_codec_register(audio_codec_ops_t *ops);
list *audio_codecs_get(void);

int audio_sink_register(audio_sink_ops_t *ops);
list *audio_sinks_get(void);

#endif
