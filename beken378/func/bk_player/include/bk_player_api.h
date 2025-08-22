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


#ifndef __BK_PLAYER_API_H__
#define __BK_PLAYER_API_H__

#ifndef PLATFORM_PC
#include <generic.h>
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <bk_player_config.h>

#define BK_PLAYER_API_VERSION   "1.0.0"

enum PLAYER_ERROR_CODE
{
    PLAYER_OK = 0,
    PLAYER_ERR = -100,
    PLAYER_NOT_INIT,
    PLAYER_NO_MEM,
    PLAYER_PROGRESS,
    PLAYER_INVALID,
    PLAYER_TIMEOUT = -200,
};

enum PLAY_MODE
{
    PLAY_ONE_SONG = 0,          //no loop
    PLAY_SEQUENCE,          //no loop
    PLAY_ONE_SONG_LOOP,
    PLAY_SEQUENCE_LOOP,
    PLAY_RANDOM
};

#define PLAY_MODE_DEFAULT   PLAY_ONE_SONG

enum EVENT_TYPE
{
    EVENT_SONG_START,
    EVENT_SONG_FINISH,
    EVENT_SONG_FAILURE,
    EVENT_SONG_PAUSE,
    EVENT_SONG_RESUME,

    EVENT_SONG_TICK,

    EVENT_LAST,
};

typedef void (*event_handler_func)(int event, void *extra_info);

int bk_player_init(event_handler_func event_cb);
void bk_player_deinit();

//mode : see enum PLAY_MODE
int bk_player_set_play_mode(int mode);
int bk_player_set_spk_gain(int gain);
int bk_player_get_spk_gain(void);


//default is output to device
int bk_player_output_device(void);
int bk_player_output_file(char *file);

//int bk_player_set_music_list(music_list_t *list);
int bk_player_clear_music_list();
int bk_player_add_music(char *name, char *url);
int bk_player_rm_music_by_name(char *name);
int bk_player_rm_music_by_url(char *url);
int bk_player_dump_music_list();

int bk_player_play();
int bk_player_play_position(int miliseconds);
int bk_player_stop();

int bk_player_pause();
int bk_player_resume();

int bk_player_prev();
int bk_player_next();
int bk_player_jumpto(int idx);

int bk_player_get_time_pos(void);
int bk_player_seek(int miliseconds);
double bk_player_get_total_time(void);

void bk_player_notify(int event, void *extra_info);

#endif
