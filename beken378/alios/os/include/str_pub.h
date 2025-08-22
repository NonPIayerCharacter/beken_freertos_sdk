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

#ifndef _STR_PUB_H_
#define _STR_PUB_H_

#include <stdarg.h>

UINT32 os_strlen(const char *str);
INT32 os_strcmp(const char *s1, const char *s2);
INT32 os_strncmp(const char *s1, const char *s2, const UINT32 n);
INT32 os_snprintf(char *buf, UINT32 size, const char *fmt, ...);
INT32 os_vsnprintf(char *buf, UINT32 size, const char *fmt, va_list ap);
char *os_strncpy(char *out, const char *in, const UINT32 n);
UINT32 os_strtoul(const char *nptr, char **endptr, int base);
char *os_strcpy(char *out, const char *in);
char *os_strchr(const char *s, int c);
char *os_strdup(const char *s);
int os_strcasecmp(const char *s1, const char *s2);
int os_strncasecmp(const char *s1, const char *s2, size_t n);
char *os_strrchr(const char *s, int c);
char *os_strstr(const char *haystack, const char *needle);
size_t os_strlcpy(char *dest, const char *src, size_t siz);
#endif // _STR_PUB_H_

// EOF
