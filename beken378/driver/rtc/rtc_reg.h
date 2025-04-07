#ifndef _RTC_REG_H_
#define _RTC_REG_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define RTC_TICK_VAL_MAX                        (0xffffffff)
#define RTC_UPPER_VAL_MAX                       (0xffffffff)

#define RTC_CLK_PER_CNT_EQL_31_25_US            3125 / 100

#define RTC_BASE                                (0x00800180)

/*
 * RW: bit[6] enable the 32k clk of rtc core
 * RW1C: bit[5] the interrupt status of whehter the current counter(tick) value is equal selected tick value
 * RW1C: bit[4] the interrupt status of whehter the current counter(tick) value is equal upper tick value
 * RW: bit[3] the selected value interrupt enable:if the tick cnt increase to the selected tick value, will report irq to CPU
 * RW: bit[2] the upper tick value interrupt enable:if the tick cnt increase to the upper tick value, will report int to CPU, value is in;
 * RW: bit[1] pause the current counter(tick). RW: 0: goon increase tick cnt, 1: pause the current counter(tick)
 * RW: bit[0] reset current counter(tick) value to 0
 */
#define RTC_REG0X0                              (RTC_BASE + 0 * 4)
#define RTC_REG0X0_VAL                          (*((volatile uint32_t *)(RTC_REG0X0)))

#define RTC_REG0X0_CLK_EN_POS                   (6)
#define RTC_REG0X0_CLK_EN_BIT                   (1 << 6)
#define RTC_REG0X0_CLK_EN                       ((RTC_REG0X0_VAL) | (1 << 6))
#define RTC_REG0X0_CLK_DIS                      ((RTC_REG0X0_VAL) & ~(1 << 6))
#define RTC_REG0X0_CLK_EN_GET                   ((RTC_REG0X0_VAL) & (1 << 6))

#define RTC_REG0X0_TICK_INT_POS                 (5)
#define RTC_REG0X0_TICK_INT_BIT                 (1 << 5)
#define RTC_REG0X0_TICK_INT_CLR                 ((RTC_REG0X0_VAL) | (1 << 5))
#define RTC_REG0X0_TICK_INT_GET                 ((RTC_REG0X0_VAL) & (1 << 5))

#define RTC_REG0X0_AON_INT_POS                  (4)
#define RTC_REG0X0_AON_INT_BIT                  (1 << 4)
#define RTC_REG0X0_AON_INT_CLR                  ((RTC_REG0X0_VAL) | (1 << 4))
#define RTC_REG0X0_AON_INT_GET                  ((RTC_REG0X0_VAL) & (1 << 4))

#define RTC_REG0X0_TICK_INT_EN_POS              (3)
#define RTC_REG0X0_TICK_INT_EN_BIT              (1 << 3)
#define RTC_REG0X0_TICK_INT_EN                  ((RTC_REG0X0_VAL) | (1 << 3))
#define RTC_REG0X0_TICK_INT_DIS                 ((RTC_REG0X0_VAL) & ~(1 << 3))
#define RTC_REG0X0_TICK_INT_EN_GET              ((RTC_REG0X0_VAL) & (1 << 3))

#define RTC_REG0X0_AON_INT_EN_POS               (2)
#define RTC_REG0X0_AON_INT_EN_BIT               (1 << 2)
#define RTC_REG0X0_AON_INT_EN                   ((RTC_REG0X0_VAL) | (1 << 2))
#define RTC_REG0X0_AON_INT_DIS                  ((RTC_REG0X0_VAL) & ~(1 << 2))
#define RTC_REG0X0_AON_INT_EN_GET               ((RTC_REG0X0_VAL) & (1 << 2))

#define RTC_REG0X0_CNT_STOP_POS                 (1)
#define RTC_REG0X0_CNT_STOP_BIT                 (1 << 1)
#define RTC_REG0X0_CNT_STOP                     ((RTC_REG0X0_VAL) | (1 << 1))
#define RTC_REG0X0_CNT_START                    ((RTC_REG0X0_VAL) & ~(1 << 1))
#define RTC_REG0X0_CNT_STOP_GET                 ((RTC_REG0X0_VAL) & (1 << 1))

#define RTC_REG0X0_CNT_RST_POS                  (0)
#define RTC_REG0X0_CNT_RST_BIT                  (1 << 0)
#define RTC_REG0X0_CNT_RST                      ((RTC_REG0X0_VAL) | (1 << 0))
#define RTC_REG0X0_CNT_RST_CLR                  ((RTC_REG0X0_VAL) & ~(1 << 0))
#define RTC_REG0X0_CNT_RST_GET                  ((RTC_REG0X0_VAL) & (1 << 0))

/*
 * RW: set the upper tick value[31:0];
 * if the counter(tick) value is equal upper, the counter(tick) value will count form 0;
 * it will update up_int_sts and report irq to cpu if enable the irq:up_int_en;
 */
#define RTC_REG0X1                              (RTC_BASE + 1 * 4)
#define RTC_REG0X1_UP_VAL_L_MASK                (0xffffffff)
#define RTC_REG0X1_RTC_UP_VAL_L_SET(val)        (*((volatile uint32_t *)(RTC_REG0X1)) = (val))
#define RTC_REG0X1_RTC_UP_VAL_L_GET             (*((volatile uint32_t *)(RTC_REG0X1)))

/*
 * RW: set the selected tick value[31:0];
 * it will update select_int_sts and report irq to cpu if the counter(tick) value is equal this value,and enable the irq:select_int_en.
 */
#define RTC_REG0X2                              (RTC_BASE + 2 * 4)
#define RTC_REG0X2_TICK_VAL_L_MASK              (0xffffffff)
#define RTC_REG0X2_RTC_TICK_VAL_L_SET(val)      (*((volatile uint32_t *)(RTC_REG0X2)) = (val))
#define RTC_REG0X2_RTC_TICK_VAL_L_GET           (*((volatile uint32_t *)(RTC_REG0X2)))

/*
 * RO: current tick value of the counter[31:0]
 */
#define RTC_REG0X3                              (RTC_BASE + 3 * 4)
#define RTC_REG0X3_CNT_VAL_L_MASK               (0xffffffff)
#define RTC_REG0X3_RTC_CNT_VAL_L_SET(val)       (*((volatile uint32_t *)(RTC_REG0X3)) = (val))
#define RTC_REG0X3_RTC_CNT_VAL_L_GET            (*((volatile uint32_t *)(RTC_REG0X3)))

/*
 * RO: update upper value to lpo[31:0]
 */
#define RTC_REG0X4                              (RTC_BASE + 4 * 4)
#define RTC_REG0X4_UP_VAL_LPO_L_MASK            (0xffffffff)
#define RTC_REG0X4_RTC_UP_VAL_LPO_L_SET(val)    (*((volatile uint32_t *)(RTC_REG0X4)) = (val))
#define RTC_REG0X4_RTC_UP_VAL_LPO_L_GET         (*((volatile uint32_t *)(RTC_REG0X4)))

/*
 * RO: update tick value to lpo[31:0]
 */
#define RTC_REG0X5                              (RTC_BASE + 5 * 4)
#define RTC_REG0X5_TICK_VAL_LPO_L_MASK          (0xffffffff)
#define RTC_REG0X5_RTC_TICK_VAL_LPO_L_SET(val)  (*((volatile uint32_t *)(RTC_REG0X5)) = (val))
#define RTC_REG0X5_RTC_TICK_VAL_LPO_L_GET       (*((volatile uint32_t *)(RTC_REG0X5)))

/*
 * RW: set the upper tick value[63:32];
 * if the counter(tick) value is equal upper, the counter(tick) value will count form 0;
 * it will update up_int_sts and report irq to cpu if enable the irq:up_int_en;
 */
#define RTC_REG0X6                              (RTC_BASE + 6 * 4)
#define RTC_REG0X6_UP_VAL_H_MASK                (0xffffffff)
#define RTC_REG0X6_RTC_UP_VAL_H_SET(val)        (*((volatile uint32_t *)(RTC_REG0X6)) = (val))
#define RTC_REG0X6_RTC_UP_VAL_H_GET             (*((volatile uint32_t *)(RTC_REG0X6)))

/*
 * RW: set the selected tick value[63:32];
 * it will update select_int_sts and report irq to cpu if the counter(tick) value is equal this value,and enable the irq:select_int_en.
 */
#define RTC_REG0X7                              (RTC_BASE + 7 * 4)
#define RTC_REG0X7_TICK_VAL_H_MASK              (0xffffffff)
#define RTC_REG0X7_RTC_TICK_VAL_H_SET(val)      (*((volatile uint32_t *)(RTC_REG0X7)) = (val))
#define RTC_REG0X7_RTC_TICK_VAL_H_GET           (*((volatile uint32_t *)(RTC_REG0X7)))

/*
 * RO: update upper value to lpo[63:32]
 */
#define RTC_REG0X8                              (RTC_BASE + 8 * 4)
#define RTC_REG0X8_UP_VAL_LPO_H_MASK            (0xffffffff)
#define RTC_REG0X8_RTC_UP_VAL_LPO_H_SET(val)    (*((volatile uint32_t *)(RTC_REG0X8)) = (val))
#define RTC_REG0X8_RTC_UP_VAL_LPO_H_GET         (*((volatile uint32_t *)(RTC_REG0X8)))

/*
 * RO: update tick value to lpo[63:32]
 */
#define RTC_REG0X9                              (RTC_BASE + 9 * 4)
#define RTC_REG0X9_TICK_VAL_LPO_H_MASK          (0xffffffff)
#define RTC_REG0X9_RTC_TICK_VAL_LPO_H_SET(val)  (*((volatile uint32_t *)(RTC_REG0X9)) = (val))
#define RTC_REG0X9_RTC_TICK_VAL_LPO_H_GET       (*((volatile uint32_t *)(RTC_REG0X9)))

/*
 * RO: current tick value of the counter[63:32]
 */
#define RTC_REG0XA                              (RTC_BASE + 10 * 4)
#define RTC_REG0XA_CNT_VAL_H_MASK               (0xffffffff)
#define RTC_REG0XA_RTC_CNT_VAL_H_SET(val)       (*((volatile uint32_t *)(RTC_REG0XA)) = (val))
#define RTC_REG0XA_RTC_CNT_VAL_H_GET            (*((volatile uint32_t *)(RTC_REG0XA)))

extern UINT32 rtc_reg_ctrl(UINT32 cmd, void *param);

#endif

#endif //_RTC_REG_H_
