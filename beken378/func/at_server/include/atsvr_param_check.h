/*
 * Tencent is pleased to support the open source community by making IoT Hub
 available.
 * Copyright (C) 2016 THL A29 Limited, a Tencent company. All rights reserved.

 * Licensed under the MIT License (the "License"); you may not use this file
 except in
 * compliance with the License. You may obtain a copy of the License at
 * http://opensource.org/licenses/MIT

 * Unless required by applicable law or agreed to in writing, software
 distributed under the License is
 * distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 KIND,
 * either express or implied. See the License for the specific language
 governing permissions and
 * limitations under the License.
 *
 */

#ifndef _ATSVR_PARAM_CHECK_H_
#define _ATSVR_PARAM_CHECK_H_

#if defined(__cplusplus)
extern "C" {
#endif

#include "include.h"

#define NUMBERIC_SANITY_CHECK(num, err)           \
    do {                                          \
        if (0 == (num)) {                         \
            bk_printf("Invalid argument, numeric 0"); \
            return (err);                         \
        }                                         \
    } while (0)

#define NUMBERIC_SANITY_CHECK_RTN(num)            \
    do {                                          \
        if (0 == (num)) {                         \
            bk_printf("Invalid argument, numeric 0"); \
            return;                               \
        }                                         \
    } while (0)

#define POINTER_SANITY_CHECK(ptr, err)                     \
    do {                                                   \
        if (NULL == (ptr)) {                               \
            bk_printf("Invalid argument, %s = %p", #ptr, ptr); \
            return (err);                                  \
        }                                                  \
    } while (0)

#define POINTER_SANITY_CHECK_RTN(ptr)                      \
    do {                                                   \
        if (NULL == (ptr)) {                               \
            bk_printf("Invalid argument, %s = %p", #ptr, ptr); \
			}                                                       \
    } while (0)

#define STRING_PTR_SANITY_CHECK(ptr, err)                      \
    do {                                                       \
        if (NULL == (ptr)) {                                   \
            bk_printf("Invalid argument, %s = %p", #ptr, (ptr));   \
            return (err);                                      \
        }                                                      \
        if (0 == strlen((ptr))) {                              \
            bk_printf("Invalid argument, %s = '%s'", #ptr, (ptr)); \
            return (err);                                      \
        }                                                      \
    } while (0)

#define STRING_PTR_SANITY_CHECK_RTN(ptr)                       \
    do {                                                       \
        if (NULL == (ptr)) {                                   \
            bk_printf("Invalid argument, %s = %p", #ptr, (ptr));   \
            return;                                            \
        }                                                      \
        if (0 == strlen((ptr))) {                              \
            bk_printf("Invalid argument, %s = '%s'", #ptr, (ptr)); \
            return;                                            \
        }                                                      \
    } while (0)



#define STRING_PTR_PRINT_SANITY_CHECK(ptr)    ((ptr)? (ptr) : "null")

#if defined(__cplusplus)
}
#endif

#endif /* _ATSVR_PARAM_CHECK_H_ */
