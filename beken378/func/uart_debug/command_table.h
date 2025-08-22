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

#ifndef _COMMAND_TABLE_H_
#define _COMMAND_TABLE_H_

#define COMMAND_MAX_COUNT            16

#define CONFIG_SYS_LONGHELP          1

/*
 * Monitor Command Table
 */
struct cmd_tbl_s
{
    char		*name;		/* Command Name			*/
    int		    maxargs;	/* maximum number of arguments	*/
    int		    repeatable;	/* autorepeat allowed?		*/
    /* Implementation function	*/
    int		    (*cmd)(struct cmd_tbl_s *, int, int, char *const []);
    char		*usage;		/* Usage message	(short)	*/

    #ifdef	CONFIG_SYS_LONGHELP
    char		*help;		/* Help  message	(long)	*/
    #endif

    #ifdef CONFIG_AUTO_COMPLETE
    /* do auto completion on the arguments */
    int		(*complete)(int argc, char *const argv[], char last_char, int maxv, char *cmdv[]);
    #endif
};

typedef struct cmd_tbl_s	cmd_tbl_t;

#if CFG_UART_DEBUG
extern cmd_tbl_t command_tbl[];

cmd_tbl_t *entry_get_start(void);
int entry_get_count(void);
#endif // CFG_UART_DEBUG

#endif // _COMMAND_TABLE_H_
