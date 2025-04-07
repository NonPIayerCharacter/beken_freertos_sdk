#ifndef _HPM_H_
#define _HPM_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define HPM_ADDR_BASE                               (0x008001C0)

#define HPM_REG0X0_CFG                              (HPM_ADDR_BASE + 0*4)
#define HPM_REG0X0_CFG_MONITOR_EN                   (1 << 0)
#define HPM_REG0X0_CFG_SINGLE_EN                    (1 << 1)
#define HPM_REG0X0_CFG_HPM_PERIOD_POSI              (8)
#define HPM_REG0X0_CFG_HPM_PERIOD_MASK              (0x07)
#define HPM_REG0X0_CFG_HPM_SHIFT_POSI               (12)
#define HPM_REG0X0_CFG_HPM_SHIFT_MASK               (0x03)
#define HPM_REG0X0_CFG_HPM_OFFSET_POSI              (16)
#define HPM_REG0X0_CFG_HPM_OFFSET_MASK              (0x3FF)
#define HPM_REG0X0_CFG_SOFT_RSTN                    (1 << 31)

#define HPM_REG0X1_THD                              (HPM_ADDR_BASE + 1*4)
#define HPM_REG0X1_THD_HPM_UP_LIMIT_POSI            (0)
#define HPM_REG0X1_THD_HPM_UP_LIMIT_MASK            (0x1FFF)
#define HPM_REG0X1_THD_HPM_DN_LIMIT_POSI            (16)
#define HPM_REG0X1_THD_HPM_DN_LIMIT_MASK            (0x1FFF)

#define HPM_REG0X2_RCD1                             (HPM_ADDR_BASE + 2*4)
#define HPM_REG0X2_RCD1_HPM_RECORD0_POSI            (0)
#define HPM_REG0X2_RCD1_HPM_RECORD0_MASK            (0x1FFF)
#define HPM_REG0X2_RCD1_HPM_RECORD0_VALID           (1 << 13)
#define HPM_REG0X2_RCD1_HPM_RECORD0_VALID_POSI      (13)
#define HPM_REG0X2_RCD1_HPM_RECORD0_VALID_MASK      (0x1)
#define HPM_REG0X2_RCD1_HPM_UP_WRANING              (1 << 14)
#define HPM_REG0X2_RCD1_HPM_DN_WRANING              (1 << 15)
#define HPM_REG0X2_RCD1_HPM_RECORD1_POSI            (16)
#define HPM_REG0X2_RCD1_HPM_RECORD1_MASK            (0x1FFF)
#define HPM_REG0X2_RCD1_HPM_RECORD1_VALID           (1 << 29)
#define HPM_REG0X2_RCD1_HPM_RECORD1_VALID_POSI      (29)
#define HPM_REG0X2_RCD1_HPM_RECORD1_VALID_MASK      (0x1)

#define HPM_REG0X3_RCD2                             (HPM_ADDR_BASE + 3*4)
#define HPM_REG0X3_RCD2_HPM_RECORD2_POSI            (0)
#define HPM_REG0X3_RCD2_HPM_RECORD2_MASK            (0x1FFF)
#define HPM_REG0X3_RCD2_HPM_RECORD2_VALID           (1 << 13)
#define HPM_REG0X3_RCD2_HPM_RECORD2_VALID_POSI      (13)
#define HPM_REG0X3_RCD2_HPM_RECORD2_VALID_MASK      (0x1)
#define HPM_REG0X3_RCD2_HPM_RECORD3_POSI            (16)
#define HPM_REG0X3_RCD2_HPM_RECORD3_MASK            (0x1FFF)
#define HPM_REG0X3_RCD2_HPM_RECORD3_VALID           (1 << 29)
#define HPM_REG0X3_RCD2_HPM_RECORD3_VALID_POSI      (29)
#define HPM_REG0X3_RCD2_HPM_RECORD3_VALID_MASK      (0x1)

extern UINT32 hpm_ctrl(UINT32 cmd, void *param);

#endif

#endif //_HPM_H_
