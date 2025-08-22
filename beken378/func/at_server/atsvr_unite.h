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

#ifndef _ATSVR_UNITE_H_
#define _ATSVR_UNITE_H_

#include "_at_svr_opts.h"

#if ATSVR_HANDLER_ENV
typedef void (*atsvr_handler)(void*env,int argc, char **argv);
#else
typedef void (*atsvr_handler)(int argc, char **argv);
#endif

/* Structure for registering at server commands */
struct _atsvr_command {
    const char *name;
    const char *help;

    atsvr_handler function;
    #if defined(ATSVR_OPTIM_FD_CMD) && ATSVR_OPTIM_FD_CMD
    unsigned char name_len;
    #endif
};

typedef struct _atsvr_command atsvr_command_t;
typedef struct _atsvr_command atsvr_command;
typedef struct _atsvr_command _atsvr_command_t;


typedef void (*output_func_t)(char *msg,unsigned int msg_len);
typedef int (*resources_protection)(int is_lock,unsigned int timeout);
typedef unsigned int (*input_msg_get_t)(char *data,unsigned int dat_len);

#endif
