#ifndef _MCU_PS_PUB_H_
#define _MCU_PS_PUB_H_

#include "typedef.h"
#include "fake_clock_pub.h"
#include "icu_pub.h"

#define     CFG_MCU_PS_SELECT_120M      1

typedef struct  mcu_ps {
	UINT8 mcu_ps_on;
	int peri_busy_count;
	UINT32 mcu_prevent;
} MCU_PS_INFO;

typedef enum
{
    LOW_LEVEL         = 0,
    HIGH_LEVEL        = 1,
    POSEDGE           = 2,
    NEGEDGE           = 3,
} WAKEUP_GPIO_TYPE;


typedef struct  sctrl_mcu_ps {
	UINT8 hw_sleep ;
	UINT8 first_sleep ;
	UINT8 mcu_use_dco;
	UINT8 wakeup_gpio_index;
	WAKEUP_GPIO_TYPE wakeup_gpio_type;
	UINT32 gpio_config_backup;
} SCTRL_MCU_PS_INFO;

#define     MCU_PS_CONNECT      CO_BIT(0)
#define     MCU_PS_ADD_KEY      CO_BIT(1)
#define     MCU_PS_GPIO_PREVENT      CO_BIT(2)

#define CHIP_U_MCU_WKUP_USE_TIMER  1

#define  PS_USE_UART_WAKE_ARM   1

extern void mcu_ps_init ( void );
extern void mcu_ps_exit ( void );
extern UINT32 mcu_power_save ( UINT32 );
extern void mcu_prevent_clear ( UINT32 );
extern void mcu_prevent_set ( UINT32 );
extern void peri_busy_count_dec ( void );
extern void peri_busy_count_add ( void );
extern UINT32 peri_busy_count_get ( void );
extern UINT32 mcu_prevent_get ( void );
extern UINT32 fclk_update_tick ( UINT32 tick );
extern void mcu_ps_dump ( void );
extern void ps_pwm_reconfig ( UINT32 , UINT8 );
extern void ps_pwm_resume_tick ( void );
extern void ps_pwm_suspend_tick ( UINT32 );
extern void ps_pwm_disable ( void );
extern void ps_pwm_enable ( void );
extern UINT32 ps_timer3_disable ( void );
extern void ps_timer3_enable ( UINT32 );
extern UINT32 ps_timer3_measure_prepare ( void );
extern UINT32 ps_pwm_int_status ( void );
extern UINT32 rtt_update_tick ( UINT32 tick );
extern UINT32 mcu_ps_is_on ( void );

typedef enum
{
    CB_HOLD_BY_TEMP_DETECT = 1,
    CB_HOLD_BY_VOLTAGE_DETECT,
} CB_HOLD_ON_TYPE;

extern void mcu_ps_cb_hold_on ( CB_HOLD_ON_TYPE type );
extern void mcu_ps_cb_release ( void );
extern UINT8 mcu_ps_get_cb_hold ( void );

#if CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
extern void mcu_ps_increase_clr(void);
extern UINT32 mcu_ps_machw_cal ( void );
extern UINT32 mcu_ps_machw_reset ( void );
extern UINT32 mcu_ps_machw_init ( void );
extern uint32 mcu_ps_need_pstick ( void );
#endif

#define  PS_PWM_ID  FCLK_PWM_ID
#if (PS_PWM_ID == PWM0)
#define MCU_PS_PWM_COUNTER      PWM0_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM0_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM0_CLK_BIT
#elif(PS_PWM_ID == PWM1)
#define MCU_PS_PWM_COUNTER      PWM1_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM1_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM1_CLK_BIT
#elif(PS_PWM_ID == PWM2)
#define MCU_PS_PWM_COUNTER      PWM2_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM2_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM2_CLK_BIT
#elif(PS_PWM_ID == PWM3)
#define MCU_PS_PWM_COUNTER      PWM3_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM3_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM3_CLK_BIT
#elif(PS_PWM_ID == PWM4)
#define MCU_PS_PWM_COUNTER      PWM4_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM4_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM4_CLK_BIT
#elif(PS_PWM_ID == PWM5)
#define MCU_PS_PWM_COUNTER      PWM5_COUNTER
#define MCU_PS_PWM_DUTY_CYCLE   PWM5_DUTY_CYCLE
#define PWD_MCU_WAKE_PWM_BIT      PWD_PWM5_CLK_BIT
#endif

#endif

