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


#ifndef __SOURCE_API_H__
#define __SOURCE_API_H__

#include "bk_player_api.h"

typedef struct audio_source_s audio_source_t;

typedef struct audio_source_ops_s
{
    int (*open)(char *url, audio_source_t **source_pp);
    int (*get_codec_type)(audio_source_t *source);
    uint32_t (*get_total_bytes)(audio_source_t *source);                     //opt
    int (*read)(audio_source_t *source, char *buffer, int len);
    int (*seek)(audio_source_t *source, int offset, uint32_t whence);   //opt
    int (*close)(audio_source_t *source);                               //opt
} audio_source_ops_t;

struct audio_source_s
{
    audio_source_ops_t *ops;

    void *source_priv;
};

audio_source_t *audio_source_new(int priv_size);

audio_source_t *audio_source_open_url(char *url);
int audio_source_close(audio_source_t *source);

int audio_source_get_codec_type(audio_source_t *source);
uint32_t audio_source_get_total_bytes(audio_source_t *source);
int audio_source_read_data(audio_source_t *source, char *buffer, int len);
int audio_source_seek(audio_source_t *source, int offset, uint32_t whence);

#endif
