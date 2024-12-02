/**
 ****************************************************************************************
 *
 * @file architect.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


#ifndef _ARCHITECT_H_
#define _ARCHITECT_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>        // standard integer definition
#include "compiler.h"      // inline functions
#include "lowlevel.h"
#include "typedef.h"

/*
 * CPU WORD SIZE
 ****************************************************************************************
 */
/// ARM is a 32-bit CPU
#define CPU_WORD_SIZE   4

/*
 * CPU Endianness
 ****************************************************************************************
 */
/// ARM is little endian
#define CPU_LE          1

/// Possible errors detected by FW
#define    RESET_NO_ERROR         0x00000000
#define    RESET_MEM_ALLOC_FAIL   0xF2F2F2F2

/// Reset platform and stay in ROM
#define    RESET_TO_ROM           0xA5A5A5A5
/// Reset platform and reload FW
#define    RESET_AND_LOAD_FW      0xC3C3C3C3

#define BK_BLE_ASSERT 0 
#define BK_BLE_DEBUG  0

#if (BK_BLE_ASSERT)
/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 *
 * @param condition C string containing the condition.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_err(const char *condition, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion error reason and loop forever.
 * The parameter value that is causing the assertion will also be disclosed.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_param(int param0, int param1, const char * file, int line);

/**
 ****************************************************************************************
 * @brief Print the assertion warning reason.
 *
 * @param param0 parameter value 0.
 * @param param1 parameter value 1.
 * @param file C string containing file where the assertion is located.
 * @param line Line number in the file where the assertion is located.
 ****************************************************************************************
 */
void assert_warn(int param0, int param1, const char * file, int line);


#define BLE_ASSERT_ERR(cond)                              \
    do {                                             \
        if (!(cond)) {                                \
            assert_err(#cond, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a critical error that could require a full system reset
#define BLE_ASSERT_INFO(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
             assert_param((int)param0, (int)param1, __MODULE__, __LINE__);  \
        }                                             \
    } while(0)

/// Assertions showing a non-critical problem that has to be fixed by the SW
#define BLE_ASSERT_WARN(cond, param0, param1)             \
    do {                                              \
        if (!(cond)) {                                \
             assert_warn((int)param0, (int)param1, __MODULE__, __LINE__); \
        }                                             \
    } while(0)
#else
#define BLE_ASSERT_ERR(cond) 
#define BLE_ASSERT_INFO(cond, param0, param1)
#define BLE_ASSERT_WARN(cond, param0, param1)
#endif

void platform_reset(uint32_t error);


#if (BK_BLE_DEBUG)
#else
/// Trace data into a VCD
#define DBG_DATA_TRACE(data, size)
/// Trace data allocation
#define DBG_DATA_ALLOC(data)
/// Trace data free
#define DBG_DATA_FREE(data)
/// Trace Function Enter
#define DBG_FUNC_ENTER(func)
/// Trace Function Exit
#define DBG_FUNC_EXIT(func)
/// Control memory access
#define DBG_MEM_GRANT_CTRL(mem_ptr, enable)
/// Set permission onto a specific memory block
#define DBG_MEM_PERM_SET(mem_ptr, size, write_en, read_en, init_clr)
/// Mark memory initialized
#define DBG_MEM_INIT(mem_ptr, size)
#endif

#endif // _ARCHITECT_H_
