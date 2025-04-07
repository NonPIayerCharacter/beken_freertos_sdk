#ifndef _RTC_REG_PUB_H_
#define _RTC_REG_PUB_H_

#if (CFG_SOC_NAME == SOC_BK7252N)
#define RTC_REG_DEV_NAME        "rtc_reg"
#define RTC_CMD_MAGIC           (0xeee0000)
enum
{
    CMD_RTC_INIT = RTC_CMD_MAGIC + 1,
    CMD_RTC_START,
    CMD_RTC_TMR_PROG,
    CMD_RTC_TMR_CLEAR,
};
extern UINT32 rtc_reg_ctrl(UINT32 cmd, void *param);

extern void rtc_reg_init(void);
extern void rtc_reg_start(void);
extern void rtc_reg_exit(void);
extern uint64_t rtc_reg_get_time_us(void);

#define cal_get_time_us()       rtc_reg_get_time_us()
#endif

#endif // _RTC_REG_PUB_H_
// eof