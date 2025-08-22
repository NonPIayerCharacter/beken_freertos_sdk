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


#ifndef __CODEC_API_H__
#define __CODEC_API_H__

#include "bk_player_api.h"

typedef struct audio_codec_s audio_codec_t;

typedef enum
{
    BITERATE_TYPE_UNKNOW = 0,
    BITERATE_TYPE_CBR,
    BITERATE_TYPE_CBR_INFO,
    BITERATE_TYPE_VBR_XING,
    BITERATE_TYPE_VBR_VBRI,
} biterate_type_t;


enum AUDIO_CODEC_TYPE
{
    AUDIO_CODEC_UNKNOWN = 0,

    AUDIO_CODEC_WAV,
    AUDIO_CODEC_MP3,
    AUDIO_CODEC_OPUS,
    AUDIO_CODEC_AMR,
    AUDIO_CODEC_AAC,
    AUDIO_CODEC_TS,
    AUDIO_CODEC_FLAC,
    AUDIO_CODEC_OGG

    //AUDIO_CODEC_MAX
};

#define DEFAULT_CHUNK_SIZE 180

#define AUDIO_INFO_UNKNOWN      (-1)

typedef struct audio_info_s
{
    int channel_number;
    int sample_rate;
    int sample_bits;
    int frame_size;

    int bps;
    uint32_t total_bytes;
    uint32_t header_bytes;
    double duration;    //in milisecond (ms)
} audio_info_t;

typedef struct audio_codec_ops_s
{
    int (*open)(enum AUDIO_CODEC_TYPE codec_type, void *param, audio_codec_t **codec_pp);
    int (*get_info)(audio_codec_t *codec, audio_info_t *info);
    int (*get_chunk_size)(audio_codec_t *codec);    //opt
    int (*get_data)(audio_codec_t *codec, char *buffer, int len);
    int (*close)(audio_codec_t *codec); //opt
    int (*calc_position)(audio_codec_t *codec, int second);
} audio_codec_ops_t;

struct audio_codec_s
{
    audio_codec_ops_t *ops;
    audio_source_t *source;

    audio_info_t info;

    void *codec_priv;
};

int audio_codec_get_type(char *ext_name);
int audio_codec_get_mime_type(char *mime);

audio_codec_t *audio_codec_new(int priv_size);

audio_codec_t *audio_codec_open(enum AUDIO_CODEC_TYPE codec_type, void *param, audio_source_t *source);
int audio_codec_close(audio_codec_t *codec);

int audio_codec_get_info(audio_codec_t *codec, audio_info_t *info);
int audio_codec_get_chunk_size(audio_codec_t *codec);
int audio_codec_get_data(audio_codec_t *codec, char *buffer, int len);
int audio_codec_calc_position(audio_codec_t *codec, int second);
#endif
