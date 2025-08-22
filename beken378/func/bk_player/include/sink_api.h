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


#ifndef __SINK_API_H__
#define __SINK_API_H__

#include "bk_player_api.h"

enum AUDIO_SINK_TYPE
{
    AUDIO_SINK_DEVICE,
    AUDIO_SINK_FILE,
    //AUDIO_SINK_NET,
};

enum AUDIO_SINK_CONTROL
{
    AUDIO_SINK_PAUSE,
    AUDIO_SINK_RESUME,
    AUDIO_SINK_MUTE,
    AUDIO_SINK_UNMUTE,
    AUDIO_SINK_FRAME_INFO_CHANGE,
    AUDIO_SINK_SET_VOLUME,
};

typedef struct sink_info_s
{
    int nChans;
    int sampRate;
    int bitsPerSample;
    int volume;
} sink_info_t;

typedef struct audio_sink_s audio_sink_t;

typedef struct audio_sink_ops_s
{
    int (*open)(enum AUDIO_SINK_TYPE sink_type, void *param, audio_sink_t **sink_pp);
    int (*write)(audio_sink_t *sink, char *buffer, int len);
    int (*control)(audio_sink_t *sink, enum AUDIO_SINK_CONTROL control);    //opt
    int (*close)(audio_sink_t *sink);   //opt
} audio_sink_ops_t;

struct audio_sink_s
{
    audio_sink_ops_t *ops;

    sink_info_t info;

    void *sink_priv;
};

audio_sink_t *audio_sink_new(int priv_size);

audio_sink_t *audio_sink_open(enum AUDIO_SINK_TYPE sink_type, void *param);
int audio_sink_close(audio_sink_t *sink);

int audio_sink_write_data(audio_sink_t *sink, char *buffer, int len);
int audio_sink_control(audio_sink_t *sink, enum AUDIO_SINK_CONTROL control);
int audio_sink_set_info(audio_sink_t *sink, int rate, int bits, int ch);
int audio_sink_set_volume(audio_sink_t *sink, int volume);
#endif

