#include "sys_config.h"
#if (1 == CFG_USE_FORCE_LOWVOL_PS)

#include "arch.h"
#include "arm_arch.h"
#include "reg_mac_pl.h"
#include "rtos_pub.h"
#include "target_util_pub.h"

#include "icu.h"
#include "bk_timer_pub.h"
#include "sys_ctrl.h"
#include "sys_ctrl_pub.h"

#include "drv_model_pub.h"
#include "fake_clock_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"

#include "force_ps_pub.h"
#include "manual_ps_pub.h"
#include "bk_timer_extense.h"
#include "force_mac_ps.h"

#define MCU_PS_TIMER_IDX        BKTIMER3

#define WITH_BUCK_EN            (0)
#define BUCK_EN_PIN             (GPIO24)
#define BUCK_EN_PIN_LEVEL       (1)
#define BUCK_DISABLE_PIN_LEVEL  (0)

extern void sctrl_cali_dpll(UINT8 flag);
extern void mcu_ps_cal_increase_tick(UINT32 *lost_p);
extern UINT32 fclk_update_tick(UINT32 tick);
extern void ble_entry(void);
extern void ble_thread_exit(void);
extern UINT32 ble_thread_is_up(void);

typedef struct mcu_time_st {
	uint64_t last_wakeup_time;
	uint64_t cur_sleep_time;
	uint64_t next_wakeup_time;
} MCU_TIME_ST, *MCU_TIME_PTR;

MCU_TIME_ST mcu_time;

typedef struct mcu_sleep_st {
	MCU_S_CFG_ST cfg;
	uint32_t blocken_bakup;
} MCU_S_ST, *MCU_S_PTR;

MCU_S_ST g_mcu_sleep;

#if WITH_BUCK_EN
static void mcu_sleep_entry_with_buck(void)
{
	uint32_t param;
	param = 4;
	sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &param);

	bk_gpio_output(BUCK_EN_PIN, BUCK_DISABLE_PIN_LEVEL);
}

static void mcu_sleep_exit_with_buck(void)
{
	uint32_t param;

	bk_gpio_output(BUCK_EN_PIN, BUCK_EN_PIN_LEVEL);

	delay(60);

	param = 0;
	sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &param);
}
#endif
static void mcu_sleep_wait_uart_tx_done(void)
{
	//uart_wait_tx_done(1);
}

void gpio_set_hiz(void)
{
	int         i;

	for (i = 0; i < 29; i ++) {
		if (0x15f3cfc3 & BIT(i))
			REG_WRITE(0x00802800 + i * 4, 0x08);
	}
}

void mcu_sleep_clear_wake_gpio(void)
{
	REG_WRITE(SCTRL_GPIO_WAKEUP_EN, 0x0); //sys_ctrl : 0x48;
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF); //sys_ctrl : 0x4a;
}

void mcu_sleep_set_wake_gpio(UINT32 gpio_index_map, UINT32 gpio_edge_map)
{
	UINT32 reg, i, param;

	for (i = 0; i < BITS_INT; i++) {
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
		if (((i > GPIO1) && (i < GPIO6))
			|| ((i > GPIO11) && (i < GPIO14))
			|| ((i > GPIO17) && (i < GPIO20))
			|| ((i > GPIO24) && (i < GPIO26))
			|| ((i > GPIO26) && (i < GPIO28)))
			continue;
#endif
		if (gpio_index_map & (0x01UL << i)) {       /*set gpio 0~31 mode*/
			if (gpio_edge_map & (0x01UL << i)) {    //0:high,1:low.
				if (sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
					param = GPIO_CFG_PARAM(i, GMODE_INPUT);
				else
					param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLUP);

				sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
				if (0x1 != (UINT32)gpio_ctrl(CMD_GPIO_INPUT, &i)) {
					/*check gpio really input value,to correct wrong edge setting*/
					param = GPIO_CFG_PARAM(i, GMODE_INPUT);
					sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
					gpio_edge_map &= ~(0x01UL << i);
				}
			} else {
				if (sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
					param = GPIO_CFG_PARAM(i, GMODE_INPUT);
				else
					param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLDOWN);

				sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
				if (0x0 != (UINT32)gpio_ctrl(CMD_GPIO_INPUT, &i)) {
					/*check gpio really input value,to correct wrong edge setting*/
					param = GPIO_CFG_PARAM(i, GMODE_INPUT);
					sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
					gpio_edge_map |= (0x01UL << i);
				}
			}
		}
	}

	/* set gpio 0~31 mode*/
	reg = 0xFFFFFFFF;
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, reg);
	reg = gpio_edge_map;
	REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE, reg);
	reg = gpio_index_map;
	REG_WRITE(SCTRL_GPIO_WAKEUP_EN, reg);

	/* set gpio 31~32 mode*/
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
	reg = 0xFFFFFFFF;
	REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, reg);
#endif
	delay(8);// 116us,at least 100us
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF);

}

static void mcu_sleep_set_rosc_timer(UINT64 sleep_us)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	/* rosc32k timer wakeup */
	UINT32 reg, value, value_h, value_l;
	if (sleep_us < 32) {
		FS_FATAL("[mcu]: 32k timer, must large than 32us\r\n");
		return;
	}

	value = (UINT32)((UINT64)sleep_us / 31.25);

	value_h = ((value & 0xffff0000) >> 16);
	value_l = ((value & 0x0000ffff) >> 0);

	/* set rosc32k timer*/
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg |= ROSC_TIMER_INT_STATUS_BIT;
	REG_WRITE(SCTRL_ROSC_TIMER, reg);

	reg = REG_READ(SCTRL_ROSC_TIMER_H);
	reg &= ~(ROSC_TIMER_H_PERIOD_MASK << ROSC_TIMER_H_PERIOD_POSI);
	reg |= ((value_h & ROSC_TIMER_H_PERIOD_MASK) << ROSC_TIMER_H_PERIOD_POSI);
	REG_WRITE(SCTRL_ROSC_TIMER_H, reg);

	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg &= ~(ROSC_TIMER_PERIOD_MASK << ROSC_TIMER_PERIOD_POSI);
	reg |= ((value_l & ROSC_TIMER_PERIOD_MASK) << ROSC_TIMER_PERIOD_POSI);
	reg |= ROSC_TIMER_ENABLE_BIT;
	REG_WRITE(SCTRL_ROSC_TIMER, reg);

	GLOBAL_INT_RESTORE();
}

static void mcu_sleep_clear_rosc_timer(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	UINT32 reg;
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg |= ROSC_TIMER_INT_STATUS_BIT;
	reg &= ~ROSC_TIMER_ENABLE_BIT;
	REG_WRITE(SCTRL_ROSC_TIMER, reg);

	GLOBAL_INT_RESTORE();
}

static void mcu_sleep(PS_DEEP_WAKEUP_WAY wake_up_way,UINT64 sleep_us)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	UINT32 reg;

	if (g_mcu_sleep.cfg.sleep_mode == MCU_LOW_VOLTAGE_SLEEP) {
#if TOPTASK_BEACON_ALIGN
		if (lv_ps_get_start_flag()) {

		} else
#endif
		{
			// in mcu low voltage sleep, only gpio and rosc32k can wakeup
			mcu_time.cur_sleep_time = rtos_get_time_us();
			mcu_sleep_set_rosc_timer(sleep_us);
		}

#if WITH_BUCK_EN
		mcu_sleep_entry_with_buck();
#endif

#if TOPTASK_BEACON_ALIGN
		lv_ps_clear_anchor_point();
#endif
		if(PS_DEEP_WAKEUP_RTC != wake_up_way)
		{
#if(CFG_SOC_NAME == SOC_BK7238)
			rosc_calib_manual_trigger();
			rosc_calib_auto_trigger_disable();
#else
			REG_WRITE(SCTRL_ROSC_CAL, 0x35);
			REG_WRITE(SCTRL_ROSC_CAL, 0x36);
#endif
		}
		/* set arm sleep mode */
		reg = REG_READ(SCTRL_SLEEP);
		reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
		reg = reg | SLEEP_MODE_CFG_LOW_VOL_WORD;
		REG_WRITE(SCTRL_SLEEP, reg);

#if TOPTASK_BEACON_ALIGN
		lv_ps_wakeup_set_timepoint();
#endif
		/* cpu stop here */
		mcu_time.last_wakeup_time = rtos_get_time_us();

#if WITH_BUCK_EN
		mcu_sleep_exit_with_buck();
#endif

		/* rosc timer int come, cpu wake up here*/
		mcu_sleep_clear_rosc_timer();
	} else if (g_mcu_sleep.cfg.sleep_mode == MCU_NORMAL_SLEEP) {
		/* set wakeup bit */
		reg = REG_READ(ICU_ARM_WAKEUP_EN);
		reg |= (TIMER_ARM_WAKEUP_EN_BIT | UART2_ARM_WAKEUP_EN_BIT
				| UART1_ARM_WAKEUP_EN_BIT | GPIO_ARM_WAKEUP_EN_BIT
				//| MAC_ARM_WAKEUP_EN_BIT
				| PWM_ARM_WAKEUP_EN_BIT);
		if (g_mcu_sleep.cfg.off_ble == 0) {
			reg |= (BLE_ARM_WAKEUP_EN_BIT);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
			reg |= (BTDM_ARM_WAKEUP_EN_BIT);
#endif
		}
		reg |= g_mcu_sleep.cfg.wakeup_sig;
		REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
#if TOPTASK_BEACON_ALIGN
		lv_ps_clear_anchor_point();
#endif
		/* arm clock disable */
		reg = REG_READ(SCTRL_SLEEP);
		reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
		reg = reg | SLEEP_MODE_CFG_NORMAL_VOL_WORD;
		REG_WRITE(SCTRL_SLEEP, reg);

#if TOPTASK_BEACON_ALIGN
		lv_ps_wakeup_set_timepoint();
#endif
		mcu_time.last_wakeup_time = rtos_get_time_us();
	}

	GLOBAL_INT_RESTORE();
}

static void mcu_hw_sleep(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	UINT32 reg;

#if(CFG_SOC_NAME == SOC_BK7238)
	/*in order to output 26M clock,will make average current higher*/
	reg = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
	reg |= (AMPBIAS_OUTEN_BIT);
	sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg);
	while((sctrl_analog_get(SCTRL_ANALOG_CTRL3) & AMPBIAS_OUTEN_BIT) == 0);

	rosc_calib_manual_trigger();
	rosc_calib_auto_trigger(2);//1s
#else
	REG_WRITE(SCTRL_ROSC_CAL, 0x35);
	REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#endif

	/* MCLK(main clock) select:dco*/
	sctrl_mclk_select(MCLK_MODE_DCO, MCLK_DIV_0);
	/* dpll division reset*/
	reg = REG_READ(SCTRL_CONTROL);
	reg |= DPLL_CLKDIV_RESET_BIT;
	REG_WRITE(SCTRL_CONTROL, reg);

	// if close 26M, wait uart done for debug
	mcu_sleep_wait_uart_tx_done();

	/* dpll (480m) & xtal2rf  disable*/
	reg = REG_READ(SCTRL_BLOCK_EN_CFG);
	g_mcu_sleep.blocken_bakup = (reg & BLOCK_EN_VALID_MASK); // bakup block_en
	reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
	reg = reg | (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
	reg &= ~(BLOCK_EN_VALID_MASK); // disable all
	if (g_mcu_sleep.cfg.off_26M == 0) {
		// keep 26m
		reg |= (BLK_EN_26M_XTAL & g_mcu_sleep.blocken_bakup);
	}
	REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);

	/* center bias power down*/
	reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
	reg &= (~(CENTRAL_BAIS_ENABLE_BIT));
	sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
	while (sctrl_analog_get(SCTRL_ANALOG_CTRL2) & (CENTRAL_BAIS_ENABLE_BIT));

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
	/* turn off dsp and usb*/
	REG_WRITE(SCTRL_DSP_PWR, (DSP_PWD << DSP_PWD_POSI));
#endif

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
	/* turn off usb and ble*/
	if (g_mcu_sleep.cfg.off_ble)
		reg = ((USB_PWD << USB_PWD_POSI) | (BLE_PWD << BLE_PWD_POSI));
	else
		reg = (USB_PWD << USB_PWD_POSI);
	REG_WRITE(SCTRL_USB_PWR, reg);
#else
	if (g_mcu_sleep.cfg.off_ble) {
		reg = (BLE_PWD << BLE_PWD_POSI);
		REG_WRITE(SCTRL_USB_PWR, reg);
	}
#endif

#if (CFG_SOC_NAME == SOC_BK7231U)
	// bk7231u must write 0x40, 0x10 can't wakeup
	REG_WRITE(SCTRL_BLOCK_EN_MUX, 0x40);
#else

#endif

	GLOBAL_INT_RESTORE();
}

static void mcu_hw_wakeup(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
	UINT32 reg;

	/* center bias power on*/
	reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
	reg |= CENTRAL_BAIS_ENABLE_BIT;
	sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
	while ((sctrl_analog_get(SCTRL_ANALOG_CTRL2) & CENTRAL_BAIS_ENABLE_BIT)  == 0);

	/*dpll(480m)  & xtal2rf enable*/
	reg = REG_READ(SCTRL_BLOCK_EN_CFG);
	reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
	reg |= (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
	reg |= (g_mcu_sleep.blocken_bakup & BLOCK_EN_VALID_MASK);  // recover bakup bit
	reg |= (BLK_EN_DPLL_480M | BLK_EN_XTAL2RF);   // must be ensure this bits
	if (g_mcu_sleep.cfg.off_26M == 1)
		reg |= (BLK_EN_26M_XTAL & g_mcu_sleep.blocken_bakup);
	REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);

	/* MCLK(main clock) select:26M*/
	sctrl_mclk_select(MCLK_MODE_26M_XTAL, MCLK_DIV_0);

	/* dpll division reset release*/
	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~(DPLL_CLKDIV_RESET_BIT);
	REG_WRITE(SCTRL_CONTROL, reg);

	/* MCLK(main clock) select:dpll*//* MCLK division*/
#if (CFG_SOC_NAME == SOC_BK7231N)
	sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_5);
#elif (CFG_SOC_NAME == SOC_BK7238)
	sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_2);
#else
	sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_3);
#endif

	sctrl_cali_dpll(1);
	sddev_control(GPIO_DEV_NAME, CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT, NULL);

	/*open 32K Rosc calib*/
#if(CFG_SOC_NAME == SOC_BK7238)
	/*in order to output 26M clock,will make average current higher*/
	reg = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
	reg &= ~(AMPBIAS_OUTEN_BIT);
	sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg);
	while((sctrl_analog_get(SCTRL_ANALOG_CTRL3) & AMPBIAS_OUTEN_BIT));

	rosc_calib_manual_trigger();
	rosc_calib_auto_trigger(2);//1s
#else
	REG_WRITE(SCTRL_ROSC_CAL, 0x35);
	REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#endif

	GLOBAL_INT_RESTORE();
}


static UINT32 force_mcu_ps_start_tick_timer(void)
{
	return bk_timer_enable(BKTIMER3, FCLK_DURATION_MS);
}

static UINT32 force_mcu_ps_stop_tick_timer(void)
{
	return bk_timer_disable(BKTIMER3);
}

static UINT32 force_mcu_ps_start_ps_timer(UINT64 sleep_us)
{
	UINT32 sleep_ms = (UINT32)(sleep_us / 1000);
	return bk_timer_enable(MCU_PS_TIMER_IDX, sleep_ms);
}

static UINT32 force_mcu_ps_stop_ps_timer(void)
{
	// has deone in  measure_ps_timer
	//return bk_timer_disable(MCU_PS_TIMER_IDX);
	return 0;
}

static UINT32 force_mcu_ps_measure_ps_timer(void)
{
	UINT32 ret, has_finish = 0, sleep_ms;

	ret = bk_timer_get_int_status(MCU_PS_TIMER_IDX, &has_finish);
	FS_FATAL("%d,%d\r\n", ret, has_finish);
	if ((ret == BK_TIMER_SUCCESS) && (has_finish == 0))
		bk_timer_pre_measure(MCU_PS_TIMER_IDX);

	bk_timer_disable(MCU_PS_TIMER_IDX);

	if (ret == BK_TIMER_SUCCESS) {
		if (has_finish == 0)
			bk_timer_measure(MCU_PS_TIMER_IDX, &sleep_ms);
		else
			bk_timer_get_end_time(MCU_PS_TIMER_IDX, &sleep_ms);
	} else {
		FS_FATAL("[mcu]: timer measure failed\r\n");
		bk_timer_get_end_time(MCU_PS_TIMER_IDX, &sleep_ms);
	}
	FS_FATAL("^:%d\r\n", sleep_ms);

	return sleep_ms * 1000;
}

static UINT32 force_mcu_ps_entry(PS_DEEP_WAKEUP_WAY wake_up_way,UINT64 sleep_us)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	mcu_hw_sleep();
	mcu_sleep(wake_up_way,sleep_us);

	GLOBAL_INT_RESTORE();

	return 0;
}

static UINT32 force_mcu_ps_exit(void)
{
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	mcu_hw_wakeup();

	GLOBAL_INT_RESTORE();

	return 0;
}

void bk_send_byte(UINT8 uport, UINT8 data);
UINT32 force_mcu_ps(PS_DEEP_WAKEUP_WAY wake_up_way,UINT64 sleep_us, UINT32 sleep_mode)
{
	UINT32 sleep_tick = 0;
	uint32_t mac_status = 0, mac_timer_enbit = 0;
	UINT32 sleep_ms = 0;

	if (sleep_us < 0)
		return 0;
	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();
#if 1
#if TOPTASK_BEACON_ALIGN
	extern   uint32_t lv_ps_is_got_anchor_point(void);
	if (lv_ps_get_start_flag() && (! lv_ps_is_got_anchor_point()))
		lv_ps_beacon_missing_handler();

	if (lv_ps_get_start_flag()) {
		lv_ps_calc_sleep_duration();
		if (sleep_mode == MCU_LOW_VOLTAGE_SLEEP)
			sctrl_enable_lvps_rosc_timer();
		else if (sleep_mode == MCU_NORMAL_SLEEP) {
			sleep_us = (UINT64)(lv_ps_get_sleep_duration() * 32);
			force_mcu_ps_stop_tick_timer();
			force_mcu_ps_start_ps_timer(sleep_us);
			os_null_printf("s:%d\r\n", sleep_us);
		}
	}
#endif
	force_mac_ps_entry(&mac_status, &mac_timer_enbit);
#endif

#if 1
	if (sleep_mode == MCU_LOW_VOLTAGE_SLEEP)
		g_mcu_sleep.cfg.off_ble = 1;
	else
		g_mcu_sleep.cfg.off_ble = 0;
	g_mcu_sleep.cfg.off_26M = 1;
	g_mcu_sleep.cfg.sleep_mode = sleep_mode;
	g_mcu_sleep.cfg.sleep_us = sleep_us;
	g_mcu_sleep.cfg.wakeup_sig = 0;

	g_mcu_sleep.blocken_bakup = 0;

	if (g_mcu_sleep.cfg.sleep_mode == MCU_NORMAL_SLEEP) {
#if TOPTASK_BEACON_ALIGN
		if (lv_ps_get_start_flag()) {

		} else
#endif
		{
			mcu_time.cur_sleep_time = rtos_get_time_us();
			force_mcu_ps_stop_tick_timer();
			force_mcu_ps_start_ps_timer(sleep_us);
		}

		force_mcu_ps_entry(wake_up_way,sleep_us);
		force_mcu_ps_exit();

		sleep_us = force_mcu_ps_measure_ps_timer();
		force_mcu_ps_stop_ps_timer();
		force_mcu_ps_start_tick_timer();
		sleep_ms = (UINT32)(sleep_us / 1000);
	} else if (g_mcu_sleep.cfg.sleep_mode == MCU_LOW_VOLTAGE_SLEEP) {
        #if (CFG_SUPPORT_BLE == 1)
        UINT32 ble_up = ble_thread_is_up();
        if((g_mcu_sleep.cfg.off_ble == 1) && (ble_up == 1))
            ble_thread_exit();
        #endif
		force_mcu_ps_entry(wake_up_way,sleep_us);
		force_mcu_ps_exit();

#if (CFG_SUPPORT_BLE == 1)
        if((g_mcu_sleep.cfg.off_ble == 1) && (ble_up == 1))
            ble_entry();
#endif
		sleep_ms = (UINT32)((rtos_get_time_us() - mcu_time.cur_sleep_time) / 1000);
	}

	sleep_tick = BK_MS_TO_TICKS(sleep_ms);
#if CFG_USE_TICK_CAL && (0 == CFG_LOW_VOLTAGE_PS)
	mcu_ps_cal_increase_tick(&sleep_tick);
#endif
	fclk_update_tick(sleep_tick);
#endif

	force_mac_ps_exit(&mac_status, &mac_timer_enbit);
	GLOBAL_INT_RESTORE();

	return sleep_us;
}

UINT32 force_mcu_ps_init(void)
{
	mcu_time.last_wakeup_time = 0,
			 mcu_time.cur_sleep_time = 0,
					  mcu_time.next_wakeup_time = 0,

							   g_mcu_sleep.cfg.sleep_mode = MCU_LOW_VOLTAGE_SLEEP;
	g_mcu_sleep.cfg.off_ble = 1;
	g_mcu_sleep.cfg.sleep_us = 0xFFFFFFFF;
	g_mcu_sleep.cfg.wakeup_sig = 0x0;
	g_mcu_sleep.blocken_bakup = 0;

#if WITH_BUCK_EN
	bk_gpio_config_output(BUCK_EN_PIN);
#endif

	return 0;
}

UINT32 force_mcu_ps_set_wakeup_time(UINT64 wakeup_time_us)
{
#if (1 == CFG_LOW_VOLTAGE_PS)
	UINT64 cur_time_us = rtos_get_time_us();
	UINT32 sleep_us;

	if (cur_time_us >= wakeup_time_us)
		return 0;

	sleep_us = wakeup_time_us - cur_time_us;
	mcu_sleep_set_rosc_timer(sleep_us);
#endif
	return 1;
}

void force_mcu_ps_clear_wakeup_time(void)
{
#if (1 == CFG_LOW_VOLTAGE_PS)
	mcu_sleep_clear_rosc_timer();
#endif
}

UINT32 bk_force_instant_lowvol_sleep(PS_DEEP_CTRL_PARAM *lowvol_param)
{
	UINT64 sleep_us = 0;
	UINT32 ret = 0;
	// check adc is busy?
	{
		UINT32 check_cnt = 0;
		extern UINT32 saradc_check_busy(void);
		while(saradc_check_busy() == 1)
		{
			rtos_delay_milliseconds(100);
			if((check_cnt++) > 10)
			{
				return 0;
			}
		}
	}

	if ((lowvol_param->wake_up_way & PS_DEEP_WAKEUP_GPIO)) {
		if (lowvol_param->gpio_index_map) {
			os_printf("---enter lowvol sleep :wake up with gpio 0~31 ps: 0x%x 0x%x \r\n",
					  lowvol_param->gpio_index_map, lowvol_param->gpio_edge_map);
		}

		if (lowvol_param->gpio_last_index_map) {
			os_printf("---enter lowvol sleep :wake up with gpio32~39 ps: 0x%x 0x%x \r\n",
					  lowvol_param->gpio_last_index_map, lowvol_param->gpio_last_edge_map);
		}

		mcu_sleep_set_wake_gpio(lowvol_param->gpio_index_map, lowvol_param->gpio_edge_map);
	} else
		mcu_sleep_clear_wake_gpio();

	if ((lowvol_param->wake_up_way & PS_DEEP_WAKEUP_RTC)) {
		os_printf("---enter lowvol sleep :wake up with ");

		if (lowvol_param->lpo_32k_src == LPO_SELECT_32K_XTAL)
			os_printf(" xtal 32k ");
		else
			os_printf("  rosc ");

		os_printf(" ps :%d s\r\n", lowvol_param->sleep_time);

		if (lowvol_param->sleep_time > 0x218de)
			lowvol_param->sleep_time = 0x218de;

		sleep_us = (UINT64)((UINT64)(1000 * 1000 * (UINT64)lowvol_param->sleep_time));
	} else
		sleep_us = 0;

#if CFG_INT_WDG_ENABLED
	UINT32 parameter;
	parameter = PWD_ARM_WATCHDOG_CLK_BIT;
	sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, (void *)&parameter);
#endif

	ret = force_mcu_ps(lowvol_param->wake_up_way, sleep_us, lowvol_param-> sleep_mode);

#if CFG_INT_WDG_ENABLED
	parameter = PWD_ARM_WATCHDOG_CLK_BIT;
	sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, (void *)&parameter);
#endif

	return ret;
}

#endif

