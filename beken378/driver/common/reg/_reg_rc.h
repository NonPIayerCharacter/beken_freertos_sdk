#ifndef __REG_RC_H_
#define __REG_RC_H_

#define REG_RC_SIZE           428

#if !(CFG_SOC_NAME == SOC_BK7252N)
#define REG_RC_BASE_ADDR            0x01050000
#define REG_RC_BASE_ADDR_MASK       0xffff0000
#else
#define REG_RC_BASE_ADDR            0x0080d000
#define REG_RC_BASE_ADDR_MASK       0x00fff000
#endif

#endif // __REG_RC_H_

