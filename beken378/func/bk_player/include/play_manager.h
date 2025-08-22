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


#ifndef __PLAY_MANAGER_H__
#define __PLAY_MANAGER_H__

#include "bk_player_api.h"
#include "music_list.h"

#include "player_osal.h"

#define LOG_ERR     0
#define LOG_WARN    1
#define LOG_INFO    2
#define LOG_DEBUG   3
#define LOG_VERBOSE 4

void player_log(int level, char *format_str, ...);

//union player_control
typedef int (*cmd_handler_func)(int cmd, void *control);

enum PLAYER_STATE
{
    STATE_FIRST,

    STATE_STOPED,
    STATE_PLAYING,
    STATE_PAUSED,

    STATE_LAST,
};

typedef struct player_state_s
{
    int cur_state;

    music_list_t *music_list;
    int play_mode;
    int cur_music_index;
    char *cur_url;

    int running;
    osal_sema_t thread_sem;

    osal_mutex_t mutex;
    int cur_cmd;
    int cur_cmd_option;
    int cmd_result;
    osal_sema_t sem;
    osal_sema_t sem_2;

    cmd_handler_func cmd_handler;
    event_handler_func event_handler;
    event_handler_func event_handler_app;

    int output_to_file;
    char *output_file;

    int spk_gain;
    int seek_position;

    void *player_priv;
} player_state_t;

player_state_t *get_player(void);

enum PLAYER_COMMAND
{
    CMD_NULL,

    CMD_PLAY,
    CMD_STOP,
    CMD_PAUSE,
    CMD_RESUME,
    CMD_PREV,
    CMD_NEXT,
    CMD_JUMP,

    CMD_SEEK,

    CMD_EXIT,

    CMD_LIST_SET,
    CMD_LIST_CLEAR,
    CMD_LIST_ADD,
    CMD_LIST_RM_NAME,
    CMD_LIST_RM_URL,
};

#endif
