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

#ifndef _IPCHKSUM_H_
#define _IPCHKSUM_H_

#if (SOC_BK7252N == CFG_SOC_NAME)
#define IPCHKSUM_DEBUG

#ifdef IPCHKSUM_DEBUG
#define IPCHKSUM_PRT      os_printf
#define IPCHKSUM_WARN     warning_prf
#define IPCHKSUM_FATAL    fatal_prf
#else
#define IPCHKSUM_PRT      null_prf
#define IPCHKSUM_WARN     null_prf
#define IPCHKSUM_FATAL    null_prf
#endif

#define IPCHKSUM_BASE                       (0x00A06200)

/* ip checksum base address */
#define IPCHKSUM_CHECKSUM_BASE_ADDR         (IPCHKSUM_BASE + 0*4)

/* ip checksum length */
#define IPCHKSUM_REG0X1_ADDR                (IPCHKSUM_BASE + 1*4)
#define IPCHKSUM_CHECKSUM_LENGTH_POSI       (0)
#define IPCHKSUM_CHECKSUM_LENGTH_MASK       (0xFFFF)

/* ip checksum start/enable */
#define IPCHKSUM_REG0X2_ADDR                (IPCHKSUM_BASE + 2*4)
#define IPCHKSUM_CHECKSUM_START             (1 << 0)

/* ip checksum data valid */
#define IPCHKSUM_REG0X3_ADDR                (IPCHKSUM_BASE + 3*4)
#define IPCHKSUM_IP_CHECKSUM_VALID          (1 << 0)

/* ip checksum data */
#define IPCHKSUM_REG0X4_ADDR                (IPCHKSUM_BASE + 4*4)
#define IPCHKSUM_IP_CHECKSUM_POSI           (0)
#define IPCHKSUM_IP_CHECKSUM_MASK           (0xFFFF)

/* ip checksum interrupt status/write 1 clear */
#define IPCHKSUM_REG0X5_ADDR                (IPCHKSUM_BASE + 5*4)
#define IPCHKSUM_CHECKSUM_INT_STATUS        (1 << 0)

extern UINT32 ipchksum_ctrl(UINT32 cmd, void *param);

#endif
#endif
