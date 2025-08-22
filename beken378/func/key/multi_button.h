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

#ifndef _MULTI_BUTTON_H_
#define _MULTI_BUTTON_H_

#include "stdint.h"
#include "string.h"

//According to your need to modify the constants.
#define TICKS_INTERVAL    5	//ms
#define DEBOUNCE_TICKS    6	//MAX 8
#define SHORT_TICKS       (400 /TICKS_INTERVAL)
#define LONG_TICKS        (1000 /TICKS_INTERVAL)


typedef void (*btn_callback)(void*);

typedef enum {
    PRESS_DOWN = 0,
    PRESS_UP,
    PRESS_REPEAT,
    SINGLE_CLICK,
    DOUBLE_CLICK,
    LONG_PRESS_START,
    LONG_PRESS_HOLD,
    number_of_event,
    NONE_PRESS
} PRESS_EVT;

typedef struct _button_ {
    uint16_t ticks;
    uint8_t  repeat : 4;
    uint8_t  event : 4;
    uint8_t  state : 3;
    uint8_t  debounce_cnt : 3;
    uint8_t  active_level : 1;
    uint8_t  button_level : 1;

    void *user_data;
    uint8_t  (*hal_button_Level)(struct _button_ *);
    btn_callback  cb[number_of_event];
    struct _button_ *next;
} BUTTON_S;

#ifdef __cplusplus
extern "C" {
#endif

void button_init(BUTTON_S* handle, uint8_t(*pin_level)(struct _button_ *), uint8_t active_level, void *user_data);
void button_attach(BUTTON_S* handle, PRESS_EVT event, btn_callback cb);
PRESS_EVT button_get_event(BUTTON_S* handle);
int  button_start(BUTTON_S* handle);
void button_stop(BUTTON_S* handle);
void button_ticks(void *param);
BUTTON_S *button_find_with_user_data(void *user_data);

#ifdef __cplusplus
}
#endif

#endif
