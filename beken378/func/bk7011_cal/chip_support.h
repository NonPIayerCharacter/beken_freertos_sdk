#ifndef __CHIP_SUPPORT_H__
#define __CHIP_SUPPORT_H__

#include <stdint.h>
#include <stdbool.h>
#include "sys_config.h"

typedef struct {
	uint32_t chip_id;
	uint32_t chip_id_mask;
	uint32_t dev_id;
	uint32_t dev_id_mask;
} chip_info_t;

#define MASK_ALL_BITS    0xFFFFFFFF
#define MASK_HIGH_16BITS 0xFFFF0000
#define MASK_LOW_16BITS  0xFFFF

/* Place all supported chip-id here.
 *
 * Generally speaking, we should use MASK_ALL_BITS to match the whole
 * chip/device id, however, sometimes you can make use of the mask
 * to reduce the length of the supported chip list.
 *
 * e.g
 * If current software supports following <chip_id, dev_id>:
 *
 * <0x7231, 0x20150001>
 * <0x7231, 0x20150002>
 * <0x7231, 0x20150003>
 *
 * Then you define the list as:
 *
 * {0x7231, MASK_ALL_BITS, 0x20150001, MASK_ALL_BITS},\
 * {0x7231, MASK_ALL_BITS, 0x20150002, MASK_ALL_BITS},\
 * {0x7231, MASK_ALL_BITS, 0x20150003, MASK_ALL_BITS},\
 *
 * If you are 100% sure that the device id 0x2015xxxx will never be used
 * in future, then you can grouped above 3 items as one item:
 *
 * {0x7231, MASK_ALL_BITS, 0x20150000, MASK_HIGH_16BITS},\
 *
 * If you are not sure whether device id 0x2015xxxx will be used, but 0x201500xx
 * will never be used in future, then you can define the chip lists as:
 *
 * {0x7231, MASK_ALL_BITS, 0x20150000, 0xFFFFF00},\
 *
 * */

#if (CFG_SOC_NAME == SOC_BK7231N)
	#define SUPPORTED_CHIPS \
	{\
		{CHIP_ID_BK7231N, MASK_ALL_BITS, DEVICE_ID_DEFAULT,     MASK_HIGH_16BITS},\
		{CHIP_ID_BK7231N, MASK_ALL_BITS, DEVICE_ID_BK7231N_A,   MASK_HIGH_16BITS},\
		{CHIP_ID_BK7231N, MASK_ALL_BITS, DEVICE_ID_BK7231N_P_B, MASK_HIGH_16BITS},\
	}
#elif (CFG_SOC_NAME == SOC_BK7221U)
	#define SUPPORTED_CHIPS \
	{\
		{CHIP_ID_BK7251,  MASK_ALL_BITS, DEVICE_ID_BK7251_A,    MASK_HIGH_16BITS},\
	}
#elif (CFG_SOC_NAME == SOC_BK7231U)
	#define SUPPORTED_CHIPS \
	{\
		{CHIP_ID_BK7231U, MASK_ALL_BITS, DEVICE_ID_BK7231U_A,   MASK_HIGH_16BITS},\
	}
#elif (CFG_SOC_NAME == SOC_BK7238)
	#define SUPPORTED_CHIPS \
	{\
		{CHIP_ID_BK7238,  MASK_ALL_BITS, DEVICE_ID_BK7238_MPW,  MASK_HIGH_16BITS},\
		{CHIP_ID_BK7238,  MASK_ALL_BITS, DEVICE_ID_BK7238_A,    MASK_HIGH_16BITS},\
	}
#endif






bool bk_is_chip_supported(void);
void bk_check_chip_id(void);
#endif
