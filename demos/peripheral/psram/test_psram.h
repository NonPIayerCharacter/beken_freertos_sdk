#ifndef _TEST_PSRAM_H_
#define _TEST_PSRAM_H_
#include "uart_pub.h"

#define PSRAM_DEBUG        1

#ifdef PSRAM_DEBUG
#define PSRAM_PRT  os_printf
#else
#define PSRAM_PRT  null_prf
#endif

#define MODE_PSRAM_SINGLE              0
#define MODE_PSRAM_QUAD                3

#define QSPI_DCACHE_START_ADDR        (0x03000000)
#define PSRAM_TEST_START_ADDR         (QSPI_DCACHE_START_ADDR)
#define PSRAM_TEST_LEN                (2048)

#endif /*_TEST_PSRAM_H_*/
// eof

