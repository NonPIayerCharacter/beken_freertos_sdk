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

/*************************************************************
 * @file        test_fatfs.h
 * @brief       Header file of test_fatfs.c
 * @author      GuWenFu
 * @version     V1.0
 * @date        2016-09-29
 * @par
 * @attention
 *
 * @history     2016-09-29 gwf    create this file
 */

#ifndef __TEST_FATFS_H__

#define __TEST_FATFS_H__


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */


#include "diskio.h"


#if CFG_USE_USB_HOST
extern void test_mount(DISK_NUMBER number);
extern void scan_file_system(DISK_NUMBER number);
extern void test_fatfs(DISK_NUMBER number);
extern void test_fatfs_format(DISK_NUMBER number);
#endif


#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif      /* __TEST_FATFS_H__ */
