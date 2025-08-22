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

#ifndef _ENTRY_DECLARE_H_
#define _ENTRY_DECLARE_H_


#ifdef CONFIG_AUTO_COMPLETE
#define _CMD_COMPLETE(x) x,
#else
#define _CMD_COMPLETE(x)
#endif

#ifdef CONFIG_SYS_LONGHELP
#define _CMD_HELP(x) x,
#else
#define _CMD_HELP(x)
#endif

#define ENTRY_CMD_COMPLETE(_name, _maxargs, _rep, _cmd, _usage, _help, _comp) \
		{ #_name, _maxargs, _rep, _cmd, _usage,			\
			_CMD_HELP(_help) _CMD_COMPLETE(_comp) }

#define ENTRY_CMD(_name, _maxargs, _rep, _cmd, _usage, _help)		\
	ENTRY_CMD_COMPLETE(_name, _maxargs, _rep, _cmd, _usage, _help, NULL)


#endif // _ENTRY_DECLARE_H_
