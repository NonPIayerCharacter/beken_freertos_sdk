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

#ifndef __DD_H_
#define __DD_H_

typedef struct _dd_init_s_
{
    char *dev_name;

    void (*init)(void);
    void (*exit)(void);
} DD_INIT_S;


/*******************************************************************************
* Function Declarations
*******************************************************************************/
extern void g_dd_init(void);
extern void g_dd_exit(void);

#endif // __DD_H_
