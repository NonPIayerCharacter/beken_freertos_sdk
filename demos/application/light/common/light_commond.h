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

#ifndef _LIGHTCOMMOND_H_
#define _LIGHTCOMMOND_H_

#include "light_commun_protocol.h"

#ifdef LIGHT_COMMUNICATION_PROTOCOL

#define LIGHT_COMMOND
#endif

#ifdef LIGHT_COMMOND
typedef void *(*LightCommondFunction)(void *MSG, void *MSG2);

typedef struct
{
    unsigned int Cmd;
    LightCommondFunction pFun;
} light_commond_type_T;

prtcl_response_msg_T *analysis_request_message(prtcl_msg_body_T *MsgBdy);

#endif /*LIGHT_COMMOND*/
#endif /*_LIGHTCOMMOND_H_*/

