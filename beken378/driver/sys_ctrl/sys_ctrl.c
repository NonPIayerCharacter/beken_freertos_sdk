#include "include.h"
#include "arm_arch.h"
#include "sys_ctrl_pub.h"
#include "sys_ctrl.h"
#include "target_util_pub.h"
#include "drv_model_pub.h"
#include "uart_pub.h"
#include "flash_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "reg_rc.h"
#include "co_math.h"
#include "rw_pub.h"
#include "manual_ps_pub.h"
#include "mcu_ps_pub.h"
#include "ps_debug_pub.h"
#include "power_save_pub.h"
#include "start_type_pub.h"
#include "ap_idle_pub.h"
#include "bk7011_cal_pub.h"
#include "low_voltage_ps.h"
#include "low_voltage_compensation.h"
#include "calendar_pub.h"
#include "ps.h"
#include "gpio.h"

#if CFG_SUPPORT_BLE && CFG_USE_BLE_PS
#include "ble_pub.h"
#include "ble.h"
#endif
#include "phy_trident.h"
#include "ate_app.h"

#if (CFG_SOC_NAME == SOC_BK7238)
#include "flash.h"
#endif

#if (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7231U)
#include "icu.h"
#endif

#if ((CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238))
#include "_reg_la.h"
#include "reg_mdm_cfg.h"
#include "reg_mac_pta.h"
#include "mac_phy_bypass_pub.h"
#define GPIO_WAKEUP_INT_BAK_ADDR REG_LA_BASE_ADDR
#else
#define GPIO_WAKEUP_INT_BAK_ADDR (0x0080a084)
#endif
#define GPIO_WAKEUP_INT_STATUS_BAK  (GPIO_WAKEUP_INT_BAK_ADDR + 0x1 * 4)
#define GPIO_WAKEUP_INT_STATUS1_BAK  (GPIO_WAKEUP_INT_BAK_ADDR + 0x2 * 4)

#define DPLL_DIV                0x0
#define DCO_CALIB_26M           26
#define DCO_CALIB_60M           60
#define DCO_CALIB_80M           80
#define DCO_CALIB_120M          120
#define DCO_CALIB_180M          180

UINT32 basic_frequency_for_delay;
#define BASIC_FREQUENCY_DPLL 480

#if (CFG_SOC_NAME == SOC_BK7221U)
#define DCO_CLK_SELECT          DCO_CALIB_180M
#define USE_DCO_CLK_POWON       1

UINT8  calib_charger[3] = {
    0x23,   //vlcf
    0x15,   //icp
    0x1b    //vcv
    };
#else
#define DCO_CLK_SELECT          DCO_CALIB_120M
#define USE_DCO_CLK_POWON       0 //BK7231N could not using DCO as main clock when boot since DCO_AMSEL_BIT should be disable during calibration
#endif

static SCTRL_PS_SAVE_VALUES ps_saves[2];

#if (CFG_USE_DEEP_PS && PS_SUPPORT_MANUAL_SLEEP)
static UINT32 ps_block_value = 0;
#endif
UINT32 deep_sleep_gpio_floating_map = 0;
#if CFG_USE_DEEP_PS
static UINT32 gpio_0_31_status = 0;
#endif
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
static UINT32 gpio_32_39_status = 0;
#endif
static SCTRL_MCU_PS_INFO sctrl_mcu_ps_info =
{
    .hw_sleep = 0,
    .mcu_use_dco = 0,
    .first_sleep = 1,
    .wakeup_gpio_index = GPIO24,
    .wakeup_gpio_type = POSEDGE,
};



static SDD_OPERATIONS sctrl_op =
{
    sctrl_ctrl
};

static UINT32 rf_hold_status = 0;
UINT32 rf_sleeped = 0;
UINT32 sta_rf_sleeped = 0;
UINT32 div_backup;

/*
 * records overclock count: > 0 if there's an API calls sctrl_overclk()
 */
static uint32_t sctrl_overclock_refcnt __maybe_unused = 0;

static void sctrl_rf_init(void);

/**********************************************************************/
void sctrl_dpll_delay10us(void)
{
    volatile UINT32 i = 0;

    for(i = 0; i < DPLL_DELAY_TIME_10US; i ++)
    {
        ;
    }
}

void sctrl_dpll_delay200us(void)
{
    volatile UINT32 i = 0;

    for(i = 0; i < DPLL_DELAY_TIME_200US; i ++)
    {
        ;
    }
}

void sctrl_ps_dpll_delay(UINT32 time)
{
    volatile UINT32 i = 0;

    for(i = 0; i < time; i ++)
    {
        ;
    }
}
#define INSTRUCTION_PER_CYCLE 13
#if 0
#define HIGH_TO_LOW 1
#define LOW_TO_HIGH 0
void sctrl_mclk_select(UINT32 mode,UINT32 div,int flag)
{
	UINT32 reg;

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
	reg &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
	if(flag)//high_to_low
	{
		reg |= mode;
		REG_WRITE(SCTRL_CONTROL,reg);
		reg |= (div << MCLK_DIV_POSI);
		REG_WRITE(SCTRL_CONTROL,reg);
	}
	else //low_to_high
	{
		reg |= (div << MCLK_DIV_POSI);
		REG_WRITE(SCTRL_CONTROL,reg);
		reg |= mode;
		REG_WRITE(SCTRL_CONTROL,reg);
	}

	switch(mode){
		case MCLK_MODE_DCO:
		basic_frequency_for_delay = DCO_CLK_SELECT/INSTRUCTION_PER_CYCLE+1;
		case MCLK_MODE_26M_XTAL:
			basic_frequency_for_delay = 26/INSTRUCTION_PER_CYCLE;
			break;
		case MCLK_MODE_DPLL:
			basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(div+1)+1;
			break;
		case MCLK_MODE_LPO:
			break;
		default:
			break;
	}
	GLOBAL_INT_RESTORE();
}
#endif
void sctrl_mclk_select(UINT32 mode,UINT32 div)
{
	UINT32 reg;

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
	reg &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
	reg |= mode;
	reg |= div << MCLK_DIV_POSI;
	REG_WRITE(SCTRL_CONTROL,reg);

	switch(mode){
		case MCLK_MODE_DCO:
			basic_frequency_for_delay = DCO_CLK_SELECT/INSTRUCTION_PER_CYCLE+1;
		case MCLK_MODE_26M_XTAL:
			basic_frequency_for_delay = 26/INSTRUCTION_PER_CYCLE;
			break;
		case MCLK_MODE_DPLL:
			basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(div+1)+1;
			break;
		case MCLK_MODE_LPO:
			break;
		default:
			break;
	}
	GLOBAL_INT_RESTORE();
}
void sctrl_cali_dpll(UINT8 flag)
{
    UINT32 param;

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    UINT32 trx_0xc = 0;
    UINT32 pwtbl_0x34 = 0;
    extern void bk7011_update_tx_power_when_cal_dpll(int start_or_stop, UINT32 *trx_0xc, UINT32 *pwtbl_0x34);
    if(!flag)
        bk7011_update_tx_power_when_cal_dpll(1, &trx_0xc, &pwtbl_0x34);
#endif
    param = sctrl_analog_get(SCTRL_ANALOG_CTRL0);
    param &= ~(SPI_TRIG_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

    if(!flag)
        sctrl_dpll_delay10us();
    else
        sctrl_ps_dpll_delay(60);

    param |= (SPI_TRIG_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

    param = sctrl_analog_get(SCTRL_ANALOG_CTRL0);
    param &= ~(SPI_DET_EN);
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

    if(!flag)
        sctrl_dpll_delay200us();
    else
        sctrl_ps_dpll_delay(340);

    param = sctrl_analog_get(SCTRL_ANALOG_CTRL0);
    param |= (SPI_DET_EN);
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    if(!flag)
        bk7011_update_tx_power_when_cal_dpll(0, &trx_0xc, &pwtbl_0x34);
#endif
}

void sctrl_dpll_isr(void)
{
#if (CFG_SOC_NAME == SOC_BK7231N)
	if ((DEVICE_ID_BK7231N_P & DEVICE_ID_MASK) != (sctrl_ctrl(CMD_GET_DEVICE_ID, NULL) & DEVICE_ID_MASK))
	{
		os_printf("BIAS Cali\r\n");
		bk7011_cal_bias();
	}
#elif (CFG_SOC_NAME == SOC_BK7236)
	os_printf("BIAS Cali\r\n");
    bk7011_cal_bias();
#endif
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT, NULL);
    if (get_ate_mode_state())
    {
        // change tx power when cali dpll in ATE mode, RF already hold
        sctrl_cali_dpll(0);
    }
    else
    {
        // keep tx power when cali dpll in APP mode, avoid access TRX
        sctrl_cali_dpll(1);
    }
    os_printf("DPLL Unlock\r\n");
}

void sctrl_dpll_int_open(void)
{
    UINT32 param;

    param = (FIQ_DPLL_UNLOCK_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);

    #if (CFG_SOC_NAME != SOC_BK7231)
    param = 1;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_EN_DPLL_UNLOOK_INT, &param);
    #endif
}

void sctrl_dpll_int_close(void)
{
    UINT32 param;

    #if (CFG_SOC_NAME != SOC_BK7231)
    param = 0;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_EN_DPLL_UNLOOK_INT, &param);
    #endif

    param = (FIQ_DPLL_UNLOCK_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

#if (CFG_SOC_NAME == SOC_BK7238)
void rosc_calib_manual_trigger(void)
{
    UINT32 reg_val;
    /* manual trigger ROSC cali in 32k mode */
    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg_val &= ~ROSC_SPI_TRIG_BIT;
    reg_val |= ROSC_CAL_MODE_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg_val);

    reg_val |= ROSC_SPI_TRIG_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg_val);
}
void rosc_calib_auto_trigger(uint32_t interval)
{
    UINT32 reg_val;

    /* enable rosc cal and config calib interval */
    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL5);
    reg_val |= ROSC_TRIG_EN_BIT;

    reg_val &= ~(ROSC_CAL_INTERVAL_MASK << ROSC_CAL_INTERVAL_POSI);
    reg_val |= (interval & ROSC_CAL_INTERVAL_MASK << ROSC_CAL_INTERVAL_POSI);
    sctrl_analog_set(SCTRL_ANALOG_CTRL5, reg_val);

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val |= ROSC_CALIB_AUTO;
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);

}
void rosc_calib_auto_trigger_disable(void)
{
    UINT32 reg_val;

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val &= ~ROSC_CALIB_AUTO;
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
}

void sctrl_rosc_cali(void)
{
    UINT32 reg_val;

    /* disable 32K ROSC manual mode */
    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
    reg_val &= ~MANU_EN_32K;
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg_val);

    rosc_calib_manual_trigger();

#if (( 1 == CFG_LOW_VOLTAGE_PS)&&(0 == CFG_LOW_VOLTAGE_PS_32K_DIV))
    rosc_calib_auto_trigger(4);//2s
#else
    rosc_calib_auto_trigger(1); //500ms
#endif
}

#endif

void sctrl_dco_cali(UINT32 speed)
{
    UINT32 reg_val;

    switch(speed)
    {
        case DCO_CALIB_180M:
        reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
        reg_val &= ~((DCO_CNTI_MASK << DCO_CNTI_POSI) | (DCO_DIV_MASK << DCO_DIV_POSI));
        reg_val |= ((0xDD & DCO_CNTI_MASK) << DCO_CNTI_POSI);
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
        reg_val |= DIV_BYPASS_BIT;
#endif
        sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
        break;

        case DCO_CALIB_120M:
        reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
        reg_val &= ~((DCO_CNTI_MASK << DCO_CNTI_POSI) | (DCO_DIV_MASK << DCO_DIV_POSI));
        reg_val |= ((0x127 & DCO_CNTI_MASK) << DCO_CNTI_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
        break;

        case DCO_CALIB_80M:
        reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
        reg_val &= ~((DCO_CNTI_MASK << DCO_CNTI_POSI) | (DCO_DIV_MASK << DCO_DIV_POSI));
        reg_val |= ((0x0C5 & DCO_CNTI_MASK) << DCO_CNTI_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
        break;

        case DCO_CALIB_60M:
        reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
        reg_val &= ~((DCO_CNTI_MASK << DCO_CNTI_POSI) | (DCO_DIV_MASK << DCO_DIV_POSI));
        reg_val |= ((0x127 & DCO_CNTI_MASK) << DCO_CNTI_POSI);
        reg_val |= ((0x02 & DCO_DIV_MASK) << DCO_DIV_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
        break;

        default:
        reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
        reg_val &= ~((DCO_CNTI_MASK << DCO_CNTI_POSI) | (DCO_DIV_MASK << DCO_DIV_POSI));
        reg_val |= ((0xC0 & DCO_CNTI_MASK) << DCO_CNTI_POSI);
        reg_val |= ((0x03 & DCO_DIV_MASK) << DCO_DIV_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
        break;
    }

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val &= ~(SPI_RST_BIT);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    reg_val &= ~(DCO_AMSEL_BIT);
#endif
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val |= SPI_RST_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val |= DCO_TRIG_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);

    reg_val = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
    reg_val &= ~(DCO_TRIG_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, reg_val);
}

void sctrl_set_cpu_clk_dco(void)
{
    UINT32 reg_val;

    reg_val = REG_READ(SCTRL_CONTROL);
    reg_val &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
    reg_val &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);

    reg_val |= ((MCLK_FIELD_DCO&MCLK_MUX_MASK) << MCLK_MUX_POSI);
    reg_val |= HCLK_DIV2_EN_BIT;

    REG_WRITE(SCTRL_CONTROL, reg_val);
    delay(10);
    basic_frequency_for_delay = DCO_CLK_SELECT/INSTRUCTION_PER_CYCLE+1;
    sctrl_mcu_ps_info.mcu_use_dco = 1;
}

#if CFG_USE_STA_PS
void sctrl_flash_select_dco(void)
{
    DD_HANDLE flash_hdl;
    UINT32 status;

    /* Flash 26MHz clock select dco clock*/
    flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);

    ddev_control(flash_hdl, CMD_FLASH_SET_DCO, 0);

    //flash get id  shouldn't remove
    ddev_control(flash_hdl, CMD_FLASH_GET_ID, &status);
}

void sctrl_sta_ps_init(void)
{
    UINT32 reg;
    extern void power_save_wakeup_isr(void);

#if ((CFG_SOC_NAME == SOC_BK7231)||(( 1 == CFG_LOW_VOLTAGE_PS)&&(1 == CFG_LOW_VOLTAGE_PS_32K_DIV)))
    reg = LPO_SELECT_32K_DIV;
#else
    reg = LPO_SELECT_ROSC;
#endif
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_SET_LOW_PWR_CLK, &reg);

#if ((CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U))
    reg = REG_READ(SCTRL_BLOCK_EN_MUX);
    reg &=~(0x1FF);
    reg |=(0x40);
    REG_WRITE(SCTRL_BLOCK_EN_MUX, reg);
#endif

#if (( 1 == CFG_LOW_VOLTAGE_PS)&&(1 == CFG_LOW_VOLTAGE_PS_32K_DIV))
    reg = REG_READ(SCTRL_BLOCK_EN_MUX);
    reg &=~(0x1FF);
    reg |=(0x10);
    REG_WRITE(SCTRL_BLOCK_EN_MUX, reg);
#endif

    #if PS_WAKEUP_MOTHOD_RW
    intc_service_register(FIQ_MAC_WAKEUP, PRI_FIQ_MAC_WAKEUP, power_save_wakeup_isr);
    nxmac_enable_lp_clk_switch_setf(0x01);
    os_printf("sctrl_sta_ps_init\r\n");
    #endif

    sctrl_flash_select_dco();

    power_save_rf_ps_wkup_semlist_init();

    PS_DEBUG_UP_OUT;
    PS_DEBUG_RX_OUT;
    PS_DEBUG_CK_OUT;
    PS_DEBUG_BCN_OUT;
    PS_DEBUG_DOWN_OUT;
    PS_DEBUG_PWM_OUT;
}
#endif

#if CFG_USE_BLE_PS
void sctrl_ble_ps_init(void)
{
}
#endif

static void sctrl_mac_ahb_slave_clock_enable(void)
{
	UINT32 reg;
#if (CFG_SOC_NAME == SOC_BK7271)
	reg = REG_READ(SCTRL_CONTROL);
	REG_WRITE(SCTRL_CONTROL, reg | MAC_HCLK_EN_BIT);
#else
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | MAC_HCLK_EN_BIT);
#endif
}

static void sctrl_mac_ahb_slave_clock_disable(void)
{
	UINT32 reg;
#if (CFG_SOC_NAME == SOC_BK7271)
	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~MAC_HCLK_EN_BIT;
	REG_WRITE(SCTRL_CONTROL, reg);
#else
	reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
	reg &= ~MAC_HCLK_EN_BIT;
	REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);
#endif
}

#if (CFG_SOC_NAME == SOC_BK7238)
void sctrl_init(void)
{
    UINT32 param;

    REG_WRITE(SCTRL_FLASH_DELAY, 0x20000);

    sddev_register_dev(SCTRL_DEV_NAME, &sctrl_op);

    /*config main clk*/
    /*sys_ctrl <0x4c> */
    param = 0x00171110;//0x00151510;    LDO BIAS CALIBRATION
    REG_WRITE(SCTRL_BIAS, param);

    /*sys_ctrl <0x16>, trig spi */
    param = 0x81105B56;//wangjian20220923 dpll_tsten=0
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

    param = 0x3CC019C0;
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, param);
    /*do dco Calibration*/
    sctrl_dco_cali(DCO_CLK_SELECT);

    param = 0x7F00769A;
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, param);

    param = 0xE65013C0; //qunshan20221215: increase xtal ldo current to improve evm; qunshan20230106: <29:28>=2
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, param);

    /*sys_ctrl <0x1a> */
    param = 0x5011A7F0;//0x5F91A7F0;
    sctrl_analog_set(SCTRL_ANALOG_CTRL4, param);

    param = 0x10088200;//wangjian20220923
    sctrl_analog_set(SCTRL_ANALOG_CTRL5, param);

    /*sys ctrl clk gating, for rx dma dead*/
    REG_WRITE(SCTRL_CLK_GATING, 0x1ff);

    /*init VDD reg*/
    REG_WRITE(SCTRL_DIGTAL_VDD, 0x8288);

    /*set always on vdddig to 8*/
    param = 0x8;
    sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &param);

    /*enable blk clk
      Attention: ENABLE 26m xtal block(BLK_BIT_26M_XTAL), for protect 32k circuit
     */
    param = BLK_BIT_26M_XTAL | BLK_BIT_DPLL_480M | BLK_BIT_XTAL2RF | BLK_BIT_DCO;
    sctrl_ctrl(CMD_SCTRL_BLK_ENABLE, &param);

    sctrl_cali_dpll(0);
    sctrl_ps_dpll_delay(DPLL_DELAY_TIME_200US);

#if USE_DCO_CLK_POWON
    sctrl_set_cpu_clk_dco();
#else
    param = REG_READ(SCTRL_CONTROL);
    param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
    param &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
    #if (CFG_SOC_NAME == SOC_BK7221U)
    /* BK7221U ahb bus max rate is 90MHZ, so ahb bus need div 2 from MCU clock */
    /* AHB bus is very import to AUDIO and DMA */
    param |= HCLK_DIV2_EN_BIT;
    #endif // (CFG_SOC_NAME == SOC_BK7221U)
    #if CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_DIV_7 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_7+1)+1;
    #elif (CFG_SOC_NAME == SOC_BK7231N)
    param |= ((MCLK_DIV_5 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_5+1)+1;
    #elif (CFG_SOC_NAME == SOC_BK7271)
    param |= ((MCLK_DIV_3 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_3+1)+1;
    #elif (CFG_SOC_NAME == SOC_BK7238)
    param |= ((MCLK_DIV_2 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_2+1)+1;
    #else // CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_DIV_3 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_3+1)+1;
    #endif // CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_FIELD_DPLL & MCLK_MUX_MASK) << MCLK_MUX_POSI);
    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    param |= (BLE_RF_PTA_EN_BIT);
    #endif
    REG_WRITE(SCTRL_CONTROL, param);
    #endif // (!USE_DCO_CLK_POWON)

    /*mac & modem power up */
    sctrl_ctrl(CMD_SCTRL_MAC_POWERUP, NULL);
    sctrl_ctrl(CMD_SCTRL_MODEM_POWERUP, NULL);

    /*regist intteruppt handler for Dpll unlock*/
    intc_service_register(FIQ_DPLL_UNLOCK, PRI_FIQ_DPLL_UNLOCK, sctrl_dpll_isr);

    sctrl_sub_reset();
    /* huaming20220722 fix access TRX/RCB bus issue */
    mdm_fe1clkforce_setf(1);

#if (CFG_SOC_NAME == SOC_BK7231N)
	sctrl_fix_dpll_div();
#endif

	/*32K Rosc calib*/
    sctrl_rosc_cali();

    sctrl_mac_ahb_slave_clock_enable();
    sctrl_rf_init();

    /* skip romboot as default */
    param = 1;
    sctrl_ctrl(CMD_SCTRL_SKIP_BOOT, (void *)&param);
#if (CFG_SOC_NAME == SOC_BK7238)
    param = REG_READ(SCTRL_CONTROL);
    param |= (PWD_ARMCLK_IN_WFI_BIT);
    REG_WRITE(SCTRL_CONTROL, param);
#endif

}
#else
void sctrl_init(void)
{
    UINT32 param;

    sddev_register_dev(SCTRL_DEV_NAME, &sctrl_op);

    /*enable blk clk
      Attention: ENABLE 26m xtal block(BLK_BIT_26M_XTAL), for protect 32k circuit
     */
#if (( 1 == CFG_LOW_VOLTAGE_PS)&&(1 == CFG_LOW_VOLTAGE_PS_32K_DIV))
    param = BLK_BIT_26M_XTAL | BLK_BIT_DPLL_480M | BLK_BIT_XTAL2RF | BLK_BIT_DCO | BLK_BIT_ANALOG_SYS_LDO;
#else
    param = BLK_BIT_26M_XTAL | BLK_BIT_DPLL_480M | BLK_BIT_XTAL2RF | BLK_BIT_DCO;
#endif
    sctrl_ctrl(CMD_SCTRL_BLK_ENABLE, &param);

    /*config main clk*/
    #if !USE_DCO_CLK_POWON
    param = REG_READ(SCTRL_CONTROL);
    param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
    param &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
    #if (CFG_SOC_NAME == SOC_BK7221U)
    /* BK7221U ahb bus max rate is 90MHZ, so ahb bus need div 2 from MCU clock */
    /* AHB bus is very import to AUDIO and DMA */
    param |= HCLK_DIV2_EN_BIT;
    #endif // (CFG_SOC_NAME == SOC_BK7221U)
    #if CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_DIV_7 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_7+1)+1;
    #elif (CFG_SOC_NAME == SOC_BK7231N)
    param |= ((MCLK_DIV_5 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_5+1)+1;
    #elif (CFG_SOC_NAME == SOC_BK7271)
    param |= ((MCLK_DIV_3 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_3+1)+1;
    #else // CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_DIV_3 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_3+1)+1;
    #endif // CFG_SYS_REDUCE_NORMAL_POWER
    param |= ((MCLK_FIELD_DPLL & MCLK_MUX_MASK) << MCLK_MUX_POSI);
	#if (CFG_SOC_NAME == SOC_BK7231N)
	param |= (BLE_RF_PTA_EN_BIT);
	#endif
    REG_WRITE(SCTRL_CONTROL, param);
    #endif // (!USE_DCO_CLK_POWON)

    /*sys_ctrl <0x4c> */
    param = 0x00171710;//0x00151510;    LDO BIAS CALIBRATION
    REG_WRITE(SCTRL_BIAS, param);

    /*mac & modem power up */
    sctrl_ctrl(CMD_SCTRL_MAC_POWERUP, NULL);
    sctrl_ctrl(CMD_SCTRL_MODEM_POWERUP, NULL);

    /*sys_ctrl <0x16>, trig spi */
    //170209,from 0x819A54B to 0x819A55B for auto detect dpll unlock
    //170614 from 0x819A55B to 0x819A59B for more easy to trigger
    //181101 xamp:0xf-0 for xtal may dead
#if (CFG_SOC_NAME == SOC_BK7231N)
#if (CFG_XTAL_FREQUENCE == CFG_XTAL_FREQUENCE_40M)
    param = 0x71125B57;
#else
    param = 0x71104953;//wangjian20200918 Reg0x16<3:1>=1 Reg0x16<9>=0 Reg0x16<13:10>=2
#endif
#else
    param = 0x819A59B;
#endif
    sctrl_analog_set(SCTRL_ANALOG_CTRL0, param);

    sctrl_cali_dpll(0);

#if (CFG_SOC_NAME == SOC_BK7231N)
    param = 0x3CC019C2;//wangjian20200918 Reg0x17<1>=1
#else
    param = 0x6AC03102;
#endif
    sctrl_analog_set(SCTRL_ANALOG_CTRL1, param);
    /*do dco Calibration*/
    sctrl_dco_cali(DCO_CLK_SELECT);
    #if USE_DCO_CLK_POWON
    sctrl_set_cpu_clk_dco();
    #endif

#if (CFG_SOC_NAME == SOC_BK7231)
    param = 0x24006000;
#elif (CFG_SOC_NAME == SOC_BK7231N)
	if ((DEVICE_ID_BK7231N_P & DEVICE_ID_MASK) == (sctrl_ctrl(CMD_GET_DEVICE_ID, NULL) & DEVICE_ID_MASK))
	{
		param = 0x580120E2;//wangjian20210422<28:23>=30 as default for BK7231P//qunshan20230717<17:16>=1
	}
	else
	{
		param = 0x500020E2;//0x400020E0; //wangjian20200822 0x40032030->0x48032030->0x48022032//wangjian20200903<17:16>=0//qunshan20201127<28:23>=20
	}
#elif (CFG_SOC_NAME == SOC_BK7236)
	param = 0x500020E2;//0x400020E0; //wangjian20200822 0x40032030->0x48032030->0x48022032//wangjian20200903<17:16>=0//qunshan20201127<28:23>=20
#elif (CFG_SOC_NAME == SOC_BK7271)
    param = 0x80208B00; //for 32k if enable BLK_BIT_ROSC32K
#else
    param = 0x24006080;   // xtalh_ctune   // 24006080
    param &= ~(XTALH_CTUNE_MASK<< XTALH_CTUNE_POSI);
    param |= ((0x10&XTALH_CTUNE_MASK) << XTALH_CTUNE_POSI);
#endif // (CFG_SOC_NAME == SOC_BK7231)
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, param);

    #if (CFG_SOC_NAME == SOC_BK7221U)
    param = CHARGE_ANALOG_CTRL3_CHARGE_DEFAULT_VALUE;
#elif (CFG_SOC_NAME == SOC_BK7231N)
    param = 0x70000000; //wangjiang20200822 0x00000000->0x70000000
    #else
    param = 0x4FE06C50;
    #endif
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, param);

    /*sys_ctrl <0x1a> */
    #if (CFG_SOC_NAME == SOC_BK7231)
    param = 0x59E04520;
    #elif (CFG_SOC_NAME == SOC_BK7221U)
    param = CHARGE_ANALOG_CTRL4_CAL_DEFAULT_VALUE;
#elif (CFG_SOC_NAME == SOC_BK7231N)
    param = 0x19C04520;
    #else
    param = 0x59C04520;  // 0x59E04520
    #endif // (CFG_SOC_NAME == SOC_BK7231)
    sctrl_analog_set(SCTRL_ANALOG_CTRL4, param);

    /*regist intteruppt handler for Dpll unlock*/
    intc_service_register(FIQ_DPLL_UNLOCK, PRI_FIQ_DPLL_UNLOCK, sctrl_dpll_isr);

    sctrl_sub_reset();

#if (CFG_SOC_NAME == SOC_BK7231N)
	sctrl_fix_dpll_div();
#endif

	/*sys ctrl clk gating, for rx dma dead*/
	REG_WRITE(SCTRL_CLK_GATING, 0x3f);

	/* increase VDD voltage*/
#if (CFG_SOC_NAME == SOC_BK7231N)
	param = 4;//wyg// original=3
    #elif CFG_SYS_REDUCE_NORMAL_POWER
	param = 4;
    #else
	param = 5;
    #endif
    sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &param);

	/*32K Rosc calib*/
    #if (( 1 == CFG_LOW_VOLTAGE_PS)&&(0 == CFG_LOW_VOLTAGE_PS_32K_DIV))
    REG_WRITE(SCTRL_ROSC_CAL, 0x37);
    #else
    REG_WRITE(SCTRL_ROSC_CAL, 0x7);
    #endif

    #if (CFG_SOC_NAME == SOC_BK7221U)
    #if (CFG_USE_AUDIO)
    sctrl_analog_set(SCTRL_ANALOG_CTRL8, 0x0033587C);
    sctrl_analog_set(SCTRL_ANALOG_CTRL9, 0x82204607);
    sctrl_analog_set(SCTRL_ANALOG_CTRL10, 0x80801027);
    #endif // CFG_USE_AUDIO
    #endif // (CFG_SOC_NAME == SOC_BK7221U)

    sctrl_mac_ahb_slave_clock_enable();
    sctrl_rf_init();

#if (CFG_SOC_NAME == SOC_BK7231N)
    param = REG_READ(SCTRL_SLEEP);
    param |= BOOT_SKIP_ENABLE_BIT; //enable boot_skip for sctrl_reboot_with_deep_sleep on bk7231nl
    REG_WRITE(SCTRL_SLEEP, param);
#endif // (CFG_SOC_NAME == SOC_BK7231N)
}
#endif

void sctrl_exit(void)
{
    sddev_unregister_dev(SCTRL_DEV_NAME);
}

void sctrl_modem_core_reset(void)
{
    sctrl_ctrl(CMD_SCTRL_MODEM_CORE_RESET, 0);
}

void sctrl_sub_reset(void)
{
    sctrl_ctrl(CMD_SCTRL_MPIF_CLK_INVERT, 0);
    sctrl_ctrl(CMD_SCTRL_MODEM_CORE_RESET, 0);
    sctrl_ctrl(CMD_SCTRL_MODEM_SUBCHIP_RESET, 0);
    sctrl_ctrl(CMD_SCTRL_MAC_SUBSYS_RESET, 0);
    sctrl_ctrl(CMD_SCTRL_USB_SUBSYS_RESET, 0);
}

static void sctrl_rf_init(void)
{
    UINT32 reg;

    /* Modem AHB clock enable*/
    reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
    REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | PHY_HCLK_EN_BIT);

    /* Modem Subsystem clock 480m enable*/
    reg = REG_READ(SCTRL_CONTROL);
#if (CFG_SOC_NAME == SOC_BK7238)
    reg &= ~MAC_CLK480M_PWD_BIT;
#endif
    reg &= ~MODEM_CLK480M_PWD_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);

	/*Enable BK7011:rc_en,ch0_en*/
    rc_cntl_stat_set(0x09);

    rf_sleeped = 0;
}

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238))
void sctrl_flash_select_flash_controller(void)
{
	UINT32 reg;

	reg = REG_READ(SCTRL_CONTROL);
	reg &= ~(FLASH_SPI_MUX_BIT);

	REG_WRITE(SCTRL_CONTROL, reg);
}

void sctrl_flash_select_spi_controller(void)
{
	UINT32 reg;

	reg = REG_READ(SCTRL_CONTROL);
	reg |= FLASH_SPI_MUX_BIT;

	REG_WRITE(SCTRL_CONTROL, reg);
}
#endif

void ps_delay(volatile UINT16 times)
{
	volatile UINT32	delay = times;
    while(delay--) ;
}

__maybe_unused static void sctrl_unconditional_mac_sleep(void);
static void sctrl_unconditional_mac_sleep(void)
{
    UINT32 reg;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    if(sta_rf_sleeped == 0)
    {
        sctrl_mac_ahb_slave_clock_disable();

        /* Mac Subsystem clock 480m disable*/
        reg = REG_READ(SCTRL_CONTROL);
        REG_WRITE(SCTRL_CONTROL, reg | MAC_CLK480M_PWD_BIT);
        PS_DEBUG_CK_TRIGER;

        sta_rf_sleeped = 1;
    }

    GLOBAL_INT_RESTORE();
}

__maybe_unused static void sctrl_unconditional_mac_wakeup(void);
static void sctrl_unconditional_mac_wakeup(void)
{
    UINT32 reg;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if( sta_rf_sleeped == 1 )
    {
        sctrl_mac_ahb_slave_clock_enable();
        /* Mac Subsystem clock 480m enable*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~MAC_CLK480M_PWD_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);
        sta_rf_sleeped = 0;
    }
    GLOBAL_INT_RESTORE();
}


//////////////////////////////////////////////////////////////////////////

void sctrl_ps_dump()
{
    UINT32 i;

    os_printf("reg dump\r\n");
    os_printf("sys\r\n0x%8x:0x%8x\r\n", SCTRL_CONTROL, REG_READ(SCTRL_CONTROL));
    os_printf("0x%8x:0x%8x\r\n", SCTRL_MODEM_CORE_RESET_PHY_HCLK, REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK));
    os_printf("0x%8x:0x%8x\r\n", SCTRL_BLOCK_EN_CFG, REG_READ(SCTRL_BLOCK_EN_CFG));
    os_printf("0x%8x:0x%8x\r\n", SCTRL_ROSC_CAL, REG_READ(SCTRL_ROSC_CAL));
    os_printf("0x%8x:0x%8x\r\n", SCTRL_ANALOG_CTRL2, sctrl_analog_get(SCTRL_ANALOG_CTRL2));
    os_printf("0x%8x:0x%8x\r\n", ICU_INTERRUPT_ENABLE, REG_READ(ICU_INTERRUPT_ENABLE));
    os_printf("0x%8x:0x%8x\r\n", ICU_PERI_CLK_PWD, REG_READ(ICU_PERI_CLK_PWD));
    os_printf("0x%8x:0x%8x\r\n", SCTRL_SLEEP, REG_READ(SCTRL_SLEEP));
    os_printf("0x%8x:0x%8x\r\n", ICU_ARM_WAKEUP_EN, REG_READ(ICU_ARM_WAKEUP_EN));
    os_printf("mac\r\n0x%8x:0x%8x\r\n", NXMAC_TIMERS_INT_UN_MASK_ADDR, nxmac_timers_int_un_mask_get());
    os_printf("0x%8x:0x%8x\r\n", NXMAC_DOZE_CNTRL_1_ADDR, nxmac_doze_cntrl_1_get());
    os_printf("0x%8x:0x%8x\r\n", NXMAC_DOZE_CNTRL_2_ADDR, nxmac_doze_cntrl_2_get());
    os_printf("0x%8x:0x%8x\r\n", NXMAC_BCN_CNTRL_1_ADDR, nxmac_bcn_cntrl_1_get());
    os_printf("saves dump\r\n");
    for(i = 0; i < (3 * (sizeof(SCTRL_PS_SAVE_VALUES) / 4)); i++)
        os_printf(" %d 0x%x\r\n", i, *((UINT32 *)(&ps_saves) + i));
}

void sctrl_disable_rosc_timer(void)
{
	UINT32 reg;
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg &= ~ (ROSC_TIMER_ENABLE_BIT);
	REG_WRITE(SCTRL_ROSC_TIMER,reg);
}

void sctrl_enable_rosc_timer(UINT32 rosc_period)
{
	UINT32 reg;
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg |= ROSC_TIMER_INT_STATUS_BIT;
	REG_WRITE(SCTRL_ROSC_TIMER,reg);

#if (CFG_SOC_NAME != SOC_BK7231)
	/* set high 16bit sleep time*/
	reg = (rosc_period >> 16) & 0xffff;
	REG_WRITE(SCTRL_ROSC_TIMER_H,reg);
#endif

	/* set high 16bit sleep time*/
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg &= ~(ROSC_TIMER_PERIOD_MASK << ROSC_TIMER_PERIOD_POSI);
	reg |= ((rosc_period & 0xffff) << ROSC_TIMER_PERIOD_POSI);
	REG_WRITE(SCTRL_ROSC_TIMER,reg);
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg |= ROSC_TIMER_ENABLE_BIT;
	REG_WRITE(SCTRL_ROSC_TIMER,reg);

}

#if (1 == CFG_LOW_VOLTAGE_PS)
bool sctrl_set_gpio_wakeup_index(UINT8 gpio_index, WAKEUP_GPIO_TYPE gpio_type)
{
    if( gpio_index >= GPIONUM )
    {
        os_printf("gpio%d must be less than %d!\r\n",gpio_index,GPIONUM);
        return false;
    }

    if( gpio_type > NEGEDGE )
    {
        os_printf("gpiowakeup type %d must be less than or equal to %d!\r\n",gpio_type,NEGEDGE);
        return false;
    }
    sctrl_mcu_ps_info.wakeup_gpio_index = gpio_index;
    sctrl_mcu_ps_info.wakeup_gpio_type = gpio_type;
    return true;
}

void sctrl_enable_gpio_wakeup(UINT8 gpio_index ,WAKEUP_GPIO_TYPE gpio_wakeup_type)
{
    UINT32 param;
    UINT32 gpio_index_map = 1<<gpio_index;

    if( gpio_index >= GPIONUM)
        return;

    if(gpio_wakeup_type == LOW_LEVEL)
    {
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE, 0);
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, 0);
    }
    else if(gpio_wakeup_type == HIGH_LEVEL)
    {
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE, 0xFFFFFFFF);
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, 0);
    }
    else if(gpio_wakeup_type == POSEDGE)
    {
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE, 0);
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, 0xFFFFFFFF);
    }
    else if(gpio_wakeup_type == NEGEDGE)
    {
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE,0xFFFFFFFF);
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, 0xFFFFFFFF);
    }
    else
        return;

    sctrl_mcu_ps_info.gpio_config_backup = REG_READ(GPIO_BASE_ADDR + gpio_index * 4);

    if((gpio_wakeup_type == HIGH_LEVEL) || (gpio_wakeup_type == POSEDGE))
    {
        param = GPIO_CFG_PARAM(gpio_index, GMODE_INPUT_PULLDOWN);
    }
    else
    {
        param = GPIO_CFG_PARAM(gpio_index, GMODE_INPUT_PULLUP);
    }

    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);

    REG_WRITE(SCTRL_GPIO_WAKEUP_EN, 0x0);
    REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF);

    REG_WRITE(SCTRL_GPIO_WAKEUP_EN,gpio_index_map);
}
#endif

#if (CFG_SOC_NAME == SOC_BK7238)
void enter_sleep(UINT32 reg)
{
	volatile  int  i;
	i = flash_get_id();
	REG_WRITE(SCTRL_SLEEP, reg);
	for(i =0; i < 1000; i ++);
}
#endif

uint64_t ble_lv_duration_us = 0;

void sctrl_hw_sleep(UINT32 peri_clk)
{
    UINT32 reg;
    PS_DEBUG_DOWN_TRIGER;
    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(2);
    }
#if (1 == CFG_LOW_VOLTAGE_PS)
    /* Modem pwd*/
//    REG_WRITE(SCTRL_PWR_MAC_MODEM, MODEM_PWD << MODEM_PWD_POSI);
    sctrl_enable_gpio_wakeup(sctrl_mcu_ps_info.wakeup_gpio_index, sctrl_mcu_ps_info.wakeup_gpio_type);
#else
    REG_WRITE(SCTRL_GPIO_WAKEUP_EN,0x0);
#endif

    reg = REG_READ(ICU_INTERRUPT_ENABLE);
    reg &= ~(CO_BIT(FIQ_DPLL_UNLOCK));
    REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

    if(power_save_if_rf_sleep())
    {
        reg = REG_READ(ICU_ARM_WAKEUP_EN);
        reg |= (MAC_ARM_WAKEUP_EN_BIT);
        REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
    }

    #if CFG_SUPPORT_BLE && CFG_USE_BLE_PS
    if(if_ble_sleep())
    {
        reg = REG_READ(ICU_ARM_WAKEUP_EN);
        reg |= (BLE_ARM_WAKEUP_EN_BIT);
        #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        reg |= (BTDM_ARM_WAKEUP_EN_BIT);
        #endif
        REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
    }
    #endif

#if (1 == CFG_LOW_VOLTAGE_PS)
	REG_WRITE(ICU_ARM_WAKEUP_EN, 0);
#endif

    PS_DEBUG_DOWN_TRIGER;

#if (1 == CFG_LOW_VOLTAGE_PS)
    //REG_WRITE(SCTRL_ROSC_CAL, 0x35);
    //REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#else
    #if ((CFG_SOC_NAME == SOC_BK7231) || (CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U))
        REG_WRITE(SCTRL_ROSC_CAL, 0x35);
        REG_WRITE(SCTRL_ROSC_CAL, 0x37);
    #endif
#endif
    if(sctrl_mcu_ps_info.mcu_use_dco == 0)
    {
        /* MCLK(main clock) select:dco*/
        sctrl_mclk_select(MCLK_MODE_DCO, MCLK_DIV_1);
        PS_DEBUG_DOWN_TRIGER;
    }

    /*close 32K Rosc calib*/
#if (CFG_SOC_NAME == SOC_BK7231)
#if (1 == CFG_LOW_VOLTAGE_PS)
        //REG_WRITE(SCTRL_ROSC_CAL, 0x26);
#else
        REG_WRITE(SCTRL_ROSC_CAL, 0x36);
#endif
#endif
    PS_DEBUG_DOWN_TRIGER;
#if (CFG_SOC_NAME != SOC_BK7238)
    /* dpll division reset*/
    reg = REG_READ(SCTRL_CONTROL);
    reg |= DPLL_CLKDIV_RESET_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);
    PS_DEBUG_DOWN_TRIGER;
#endif
    /* dpll (480m) & xtal2rf  disable*/
    reg = REG_READ(SCTRL_BLOCK_EN_CFG);
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg = reg | (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg &= ~(BLK_EN_DPLL_480M | BLK_EN_XTAL2RF );
#if (CFG_SOC_NAME == SOC_BK7238)&&(1 == CFG_LOW_VOLTAGE_PS)&&(0 == CFG_LOW_VOLTAGE_PS_32K_DIV)
    reg &= ~(BLK_EN_26M_XTAL  );
#endif
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
    PS_DEBUG_DOWN_TRIGER;
#if (CFG_SOC_NAME == SOC_BK7238)
    reg = REG_READ(SCTRL_CONTROL);
    reg |= (PWD_AHBCLK_IN_SLEEP_BIT);
    REG_WRITE(SCTRL_CONTROL, reg);
    /*in order to output 26M clock*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
    reg |= (AMPBIAS_OUTEN_BIT);//wangjian suggest amp config before central bias
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg);
    while((sctrl_analog_get(SCTRL_ANALOG_CTRL3) & AMPBIAS_OUTEN_BIT) == 0);
#endif
#if (CFG_SOC_NAME != SOC_BK7221U)
    /* center bias power down*/
    if( 0 == mcu_ps_get_cb_hold() )
    {
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
        reg &= (~(CENTRAL_BAIS_ENABLE_BIT));
        sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
        while(sctrl_analog_get(SCTRL_ANALOG_CTRL2) & (CENTRAL_BAIS_ENABLE_BIT));
    }
#endif
    sctrl_mcu_ps_info.hw_sleep = 1;
    PS_DEBUG_DOWN_TRIGER;
#if PS_CLOSE_PERI_CLK
    /* close all peri clock*/
    ps_saves[0].peri_clk_cfg= REG_READ(ICU_PERI_CLK_PWD);
    REG_WRITE(ICU_PERI_CLK_PWD, peri_clk);
#endif
#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
    uint64_t current_time = cal_get_time_us();
    lv_ps_info_mcu_sleep(current_time);
#endif

    ps_delay(1);
    /* arm clock disable */
    reg = REG_READ(SCTRL_SLEEP);
    reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
    #if (1 == CFG_LOW_VOLTAGE_PS)
    reg = reg | SLEEP_MODE_CFG_LOW_VOL_WORD;
    #else
    reg = reg | SLEEP_MODE_CFG_NORMAL_VOL_WORD;
    #endif
#if (1 == CFG_LOW_VOLTAGE_PS)
//	lv_ps_clear_anchor_point();

#if SYS_CTRL_USE_VDD_BULK
    //reg = 4;
    //sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &reg);

    REG_WRITE(SCTRL_DIGTAL_VDD, 0x40);
//    REG_WRITE((0x00802800+(24*4)), 0x00);
#endif
//    REG_WRITE((0x00802800+(23*4)), 0x02);
#endif

    /* mcu enter sleep */
#if (CFG_SOC_NAME == SOC_BK7238)
    uint64_t time_saved;
    time_saved = cal_get_time_us();
    enter_sleep(reg);
    ble_lv_duration_us = cal_get_time_us() - time_saved;
#else
    REG_WRITE(SCTRL_SLEEP, reg);
#endif

#if (1 == CFG_LOW_VOLTAGE_PS)&&(0==CFG_LV_PS_WITH_IDLE_TICK)
    lv_ps_rf_reinit = 0;

//	REG_WRITE((0x00802800+(23*4)), 0x00);
    lv_ps_wakeup_set_timepoint();
    lv_ps_clear_anchor_point();
    PS_DEBUG_BCN_TRIGER;

#if SYS_CTRL_USE_VDD_BULK
//    REG_WRITE((0x00802800+(24*4)), 0x02);
    sctrl_ps_dpll_delay(6500);
    //reg = 0;
    //sctrl_ctrl(CMD_SCTRL_SET_VDD_VALUE, &reg);
    REG_WRITE(SCTRL_DIGTAL_VDD, 0x00);
#endif
#endif

    ps_delay(1);//5
}

void sctrl_hw_wakeup(void)
{
    UINT32 reg;

	PS_DEBUG_BCN_TRIGER;
#if (CFG_SOC_NAME != SOC_BK7221U)
    /* center bias power on*/
    if( 0 == mcu_ps_get_cb_hold() )
    {
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
        reg |= CENTRAL_BAIS_ENABLE_BIT;
        sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
        while((sctrl_analog_get(SCTRL_ANALOG_CTRL2) & CENTRAL_BAIS_ENABLE_BIT)  == 0);
    }
#endif
#if (CFG_SOC_NAME == SOC_BK7238)
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(PWD_AHBCLK_IN_SLEEP_BIT);
    REG_WRITE(SCTRL_CONTROL, reg);
    /*in order to output 26M clock*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
    reg &= ~(AMPBIAS_OUTEN_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg);
    while((sctrl_analog_get(SCTRL_ANALOG_CTRL3) & AMPBIAS_OUTEN_BIT));
#endif
    /*dpll(480m)  & xtal2rf enable*/
    reg = REG_READ(SCTRL_BLOCK_EN_CFG);
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg |= (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg |= ( BLK_EN_DPLL_480M | BLK_EN_XTAL2RF );
#if (CFG_SOC_NAME == SOC_BK7238)&&(1 == CFG_LOW_VOLTAGE_PS)&&(0 == CFG_LOW_VOLTAGE_PS_32K_DIV)
    reg |= (BLK_EN_26M_XTAL  );
#endif
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
    ps_delay(10);
    PS_DEBUG_BCN_TRIGER;
#if (CFG_SOC_NAME == SOC_BK7238)
    #if (1 == CFG_LOW_VOLTAGE_PS)&&(0 == CFG_LOW_VOLTAGE_PS_32K_DIV)
    uint64_t previous_us = cal_get_time_us();
    while(REG_READ(SCTRL_ANALOG_STATE) & DPLL_UNLOCK_STATE_BIT);
    g_dpll_lock_delay = cal_get_time_us() - previous_us;
    #endif
    sctrl_cali_dpll(1);
    PS_DEBUG_BCN_TRIGER;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT, NULL);

    /* MCLK(main clock) select:dpll*//* MCLK division*/
    sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_7);
    PS_DEBUG_BCN_TRIGER;
#else
    /*cali dpll before switch to dpll*/
    sctrl_cali_dpll(1);
    if(sctrl_mcu_ps_info.mcu_use_dco == 0)
    {
        /* MCLK(main clock) select:26M*/
        sctrl_mclk_select(MCLK_MODE_26M_XTAL, MCLK_DIV_0);
        ps_delay(500);
	    PS_DEBUG_BCN_TRIGER;

        /* dpll division reset release*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(DPLL_CLKDIV_RESET_BIT);
        REG_WRITE(SCTRL_CONTROL, reg);

        /* MCLK(main clock) select:dpll*//* MCLK division*/
        sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_7);
	    PS_DEBUG_BCN_TRIGER;
    }
    else
    {
        ps_delay(500);

        /* dpll division reset release*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(DPLL_CLKDIV_RESET_BIT);
        REG_WRITE(SCTRL_CONTROL, reg);
	    PS_DEBUG_BCN_TRIGER;
    }

    PS_DEBUG_BCN_TRIGER;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT, NULL);
#endif

    sctrl_mcu_ps_info.hw_sleep = 0;

    #if ((CFG_SOC_NAME == SOC_BK7231) || (CFG_SOC_NAME == SOC_BK7231U) || (CFG_SOC_NAME == SOC_BK7221U))
    /*open 32K Rosc calib*/
    REG_WRITE(SCTRL_ROSC_CAL, 0x35);
    REG_WRITE(SCTRL_ROSC_CAL, 0x37);
    #endif

    reg = REG_READ(ICU_INTERRUPT_ENABLE);
    reg |= (CO_BIT(FIQ_DPLL_UNLOCK));
    REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

#if (1 == CFG_LOW_VOLTAGE_PS)
    /* Modem pwd*/
/*    reg = REG_READ(SCTRL_PWR_MAC_MODEM);
    reg &= ~(MODEM_PWD_MASK << MODEM_PWD_POSI);
    reg = reg | (MODEM_PWU << MODEM_PWD_POSI);
    REG_WRITE(SCTRL_PWR_MAC_MODEM, reg); */

    if(REG_READ(SCTRL_GPIO_WAKEUP_INT_STATUS))
    {
        REG_WRITE(SCTRL_GPIO_WAKEUP_EN, 0x0);
        REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF);
        mcu_ps_exit();
        lv_ps_wake_up_way = PS_DEEP_WAKEUP_GPIO;
        REG_WRITE(GPIO_BASE_ADDR + sctrl_mcu_ps_info.wakeup_gpio_index * 4,sctrl_mcu_ps_info.gpio_config_backup);
    }

    reg = REG_READ(SCTRL_ROSC_TIMER);
    if( reg & ROSC_TIMER_INT_STATUS_BIT)
    {
        lv_ps_wake_up_way = PS_DEEP_WAKEUP_RTC;
    }
    reg &= ~ (ROSC_TIMER_ENABLE_BIT);
    REG_WRITE(SCTRL_ROSC_TIMER,reg);
#endif

    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(4);
    }
	PS_DEBUG_BCN_TRIGER;
}

UINT8 sctrl_if_rf_sleep(void)
{
    UINT32 value;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    value =  rf_sleeped;
    GLOBAL_INT_RESTORE();
    return value;
}

static void sctrl_rf_sleep(void)
{
    UINT32 reg;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if(0 == rf_sleeped)
    {
        /*Disable BK7011*/
#if (1 == CFG_LOW_VOLTAGE_PS)
        if (1 == lv_ps_rf_pre_pwr_down)
        {
            lv_ps_rf_pre_pwr_down = 0;
        }
        else
#endif
        {
            rc_cntl_stat_set(0x0);
        }

        rf_sleeped = 1;

        /* Modem AHB clock disable*/
        reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
        reg &= ~PHY_HCLK_EN_BIT;
        REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);
        /* Modem Subsystem clock 480m disable*/
        reg = REG_READ(SCTRL_CONTROL);
        REG_WRITE(SCTRL_CONTROL, reg | MODEM_CLK480M_PWD_BIT);
    }

    GLOBAL_INT_RESTORE();
}

static void sctrl_rf_wakeup(void)
{
	UINT32 reg;
	GLOBAL_INT_DECLARATION();

	GLOBAL_INT_DISABLE();
	if(rf_sleeped == 1)
	{
		/* Modem AHB clock enable*/
		#if (CFG_SOC_NAME == SOC_BK7271)
		reg = REG_READ(SCTRL_CONTROL);
		REG_WRITE(SCTRL_CONTROL, reg | PHY_HCLK_EN_BIT);
		#else
		reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
		REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | PHY_HCLK_EN_BIT);
		#endif

		/* Modem Subsystem clock 480m enable*/
		reg = REG_READ(SCTRL_CONTROL);
		reg &= ~MODEM_CLK480M_PWD_BIT;
		REG_WRITE(SCTRL_CONTROL, reg);

		/*Enable BK7011:rc_en,ch0_en*/
		rc_cntl_stat_set(0x09);

#if (CFG_SOC_NAME == SOC_BK7231N) && (0 == CFG_LOW_VOLTAGE_PS)
		sctrl_fix_dpll_div();
		phy_wakeup_rf_reinit();
		phy_wakeup_wifi_reinit();
#elif (CFG_SOC_NAME == SOC_BK7238) && (0 == CFG_LOW_VOLTAGE_PS)
		phy_wakeup_rf_reinit();
		phy_wakeup_wifi_reinit();
#endif

		rf_sleeped = 0;
	}
	GLOBAL_INT_RESTORE();
}

#if CFG_USE_STA_PS

void sctrl_sta_rf_sleep(void)
{
    UINT32 reg;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    if(((!bk_wlan_has_role(VIF_STA))
        || power_save_if_rf_sleep())
    && ap_if_ap_rf_sleep()
    && (sta_rf_sleeped == 0)
        )
    {
        reg = RF_HOLD_BY_STA_BIT;
        sctrl_ctrl(CMD_RF_HOLD_BIT_CLR, &reg);

        sctrl_mac_ahb_slave_clock_disable();

        /* Mac Subsystem clock 480m disable*/
        reg = REG_READ(SCTRL_CONTROL);
        REG_WRITE(SCTRL_CONTROL, reg | MAC_CLK480M_PWD_BIT);

#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
        lv_ps_info_rf_sleep(0);
#endif
        sta_rf_sleeped = 1;
    }

    GLOBAL_INT_RESTORE();
}
void sctrl_sta_rf_wakeup(void)
{
    UINT32 reg;
    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();
    if( sta_rf_sleeped == 1 )
    {
        if(sctrl_mcu_ps_info.hw_sleep == 1)
        {
            //if rf add mcu up meanwhile
            os_printf("err, hw not up\r\n");
        }

        sctrl_mac_ahb_slave_clock_enable();
        /* Mac Subsystem clock 480m enable*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~MAC_CLK480M_PWD_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);

        reg = RF_HOLD_BY_STA_BIT;
        sctrl_ctrl(CMD_RF_HOLD_BIT_SET, &reg);

//    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
//        phy_wakeup_wifi_reinit();
//    #endif

#if ((1 == CFG_LOW_VOLTAGE_PS) && (1 == CFG_LOW_VOLTAGE_PS_TEST))
        lv_ps_info_rf_wakeup(0);
#endif
        sta_rf_sleeped = 0;
    }
    GLOBAL_INT_RESTORE();
}
#endif

#if CFG_USE_MCU_PS
#if (CFG_SOC_NAME == SOC_BK7238)
extern uint8_t rwip_driver_ext_wakeup_get(void);
#endif
UINT8 sctrl_if_mcu_can_sleep(void)
{
    return (((! bk_wlan_has_role(VIF_STA))
        || power_save_if_rf_sleep())
#if CFG_SUPPORT_BLE && CFG_USE_BLE_PS
        &&(!ble_thread_is_up() || if_ble_sleep())
#if (CFG_SOC_NAME == SOC_BK7238)
        &&(!rwip_driver_ext_wakeup_get())
#endif
#endif
        && ap_if_ap_rf_sleep()
        && sctrl_if_rf_sleep()
        && (sctrl_mcu_ps_info.hw_sleep == 0));
}

void sctrl_mcu_sleep(UINT32 peri_clk)
{
    UINT32 reg;

    if(sctrl_if_mcu_can_sleep())
    {
        reg = REG_READ(ICU_ARM_WAKEUP_EN);

        if(sctrl_mcu_ps_info.first_sleep == 1)
        {
            reg = 0x0;
            sctrl_mcu_ps_info.first_sleep = 0;
        }
#if (CHIP_U_MCU_WKUP_USE_TIMER && (CFG_SOC_NAME != SOC_BK7231))
        reg |= (TIMER_ARM_WAKEUP_EN_BIT | UART2_ARM_WAKEUP_EN_BIT
            | UART1_ARM_WAKEUP_EN_BIT| GPIO_ARM_WAKEUP_EN_BIT
            | PWM_ARM_WAKEUP_EN_BIT);
#else
        reg |= (PWM_ARM_WAKEUP_EN_BIT | UART2_ARM_WAKEUP_EN_BIT
            | UART1_ARM_WAKEUP_EN_BIT| GPIO_ARM_WAKEUP_EN_BIT
            );
#endif
        if(mcu_ps_get_cb_hold() == CB_HOLD_BY_TEMP_DETECT ||
           mcu_ps_get_cb_hold() == CB_HOLD_BY_VOLTAGE_DETECT)
        {
            reg |= SARADC_ARM_WAKEUP_EN_BIT;
        }
        REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
#if (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7231U)
        reg = REG_READ(ICU_PERI_CLK_PWD);
        reg |= (PWD_ARM_WATCHDOG_CLK);
        REG_WRITE(ICU_PERI_CLK_PWD, reg);
#endif
        sctrl_hw_sleep(peri_clk);
    }
    else
    {
        PS_DEBUG_DOWN_TRIGER;
        delay(1);
#if PS_CLOSE_PERI_CLK
        /* close all peri clock*/
        ps_saves[0].peri_clk_cfg= REG_READ(ICU_PERI_CLK_PWD);
        REG_WRITE(ICU_PERI_CLK_PWD, peri_clk);
#endif

        PS_DEBUG_DOWN_TRIGER;
#if (CFG_SOC_NAME == SOC_BK7231)
        REG_WRITE(SCTRL_ROSC_CAL, 0x35);
        REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#endif
        WFI();
    }
    delay(5);
}

UINT32 sctrl_mcu_wakeup(void)
{
    UINT32 wkup_type;

    if(sctrl_mcu_ps_info.hw_sleep == 1)
    {
        sctrl_hw_wakeup();
#if (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7231U)
        UINT32 reg;
        reg = REG_READ(ICU_PERI_CLK_PWD);
        reg &= ~(PWD_ARM_WATCHDOG_CLK);
        REG_WRITE(ICU_PERI_CLK_PWD, reg);
#endif
        #if PS_CLOSE_PERI_CLK
        REG_WRITE(ICU_PERI_CLK_PWD, ps_saves[0].peri_clk_cfg);
        #endif
        wkup_type = 1;
    }
    else
    {
    	PS_DEBUG_BCN_TRIGER;
        delay(2);
    	PS_DEBUG_BCN_TRIGER;

        #if PS_CLOSE_PERI_CLK
        /* recovery periphral unit clock config*/
        REG_WRITE(ICU_PERI_CLK_PWD, ps_saves[0].peri_clk_cfg);
        #endif
        wkup_type = 0;
     }
    return wkup_type;
}

void sctrl_mcu_init(void)
{
    UINT32 reg;

    reg = REG_READ(SCTRL_CONTROL);
    div_backup = (reg >> MCLK_DIV_POSI) & MCLK_DIV_MASK;
    reg &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
#if (CFG_SOC_NAME == SOC_BK7221U)
    reg &= ~HCLK_DIV2_EN_BIT;
#endif
    reg |= ((MCLK_DIV_7 & MCLK_DIV_MASK) << MCLK_DIV_POSI);

    reg &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
    reg |= ((MCLK_FIELD_DPLL & MCLK_MUX_MASK) << MCLK_MUX_POSI);
    REG_WRITE(SCTRL_CONTROL, reg);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(MCLK_DIV_7+1)+1;
    sctrl_mcu_ps_info.mcu_use_dco = 0;
}

void sctrl_mcu_exit(void)
{
#if (USE_DCO_CLK_POWON )
    sctrl_set_cpu_clk_dco();
#else
    UINT32 reg;
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);

#if (CFG_SOC_NAME == SOC_BK7221U)
	reg |= HCLK_DIV2_EN_BIT;
#endif
    reg |= ((div_backup & MCLK_DIV_MASK) << MCLK_DIV_POSI);
    REG_WRITE(SCTRL_CONTROL, reg);
    basic_frequency_for_delay = BASIC_FREQUENCY_DPLL/INSTRUCTION_PER_CYCLE/(div_backup+1)+1;
    sctrl_mcu_ps_info.mcu_use_dco = 0;

#endif
}
#endif

void sctrl_subsys_power(UINT32 cmd)
{
    UINT32 reg = 0;
    UINT32 reg_val;
    UINT32 reg_word = 0;

    switch(cmd)
    {
    case CMD_SCTRL_DSP_POWERDOWN:
        reg = SCTRL_DSP_PWR;
        reg_word = DSP_PWD;
        break;

    case CMD_SCTRL_DSP_POWERUP:
        reg = SCTRL_DSP_PWR;
        reg_word = DSP_PWU;
        break;

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
    case CMD_SCTRL_USB_POWERDOWN:
        reg = SCTRL_USB_PWR;
        reg_val = REG_READ(SCTRL_USB_PWR);
        reg_val &= ~(USB_PWD_MASK << USB_PWD_POSI);
        reg_val |= USB_PWD << USB_PWD_POSI;
        reg_word = reg_val;
        break;

    case CMD_SCTRL_USB_POWERUP:
        reg = SCTRL_USB_PWR;
        reg_val = REG_READ(SCTRL_USB_PWR);
        reg_val &= ~(USB_PWD_MASK << USB_PWD_POSI);
        reg_val |= USB_PWU << USB_PWD_POSI;
        reg_word = reg_val;
        break;
#endif

    case CMD_SCTRL_MAC_POWERDOWN:
        reg = SCTRL_PWR_MAC_MODEM;
        reg_val = REG_READ(SCTRL_PWR_MAC_MODEM);
        reg_val &= ~(MAC_PWD_MASK << MAC_PWD_POSI);
        reg_val |= MAC_PWD << MAC_PWD_POSI;
        reg_word = reg_val;
        break;

    case CMD_SCTRL_MAC_POWERUP:
        reg = SCTRL_PWR_MAC_MODEM;
        reg_val = REG_READ(SCTRL_PWR_MAC_MODEM);
        reg_val &= ~(MAC_PWD_MASK << MAC_PWD_POSI);
        reg_val |= MAC_PWU << MAC_PWD_POSI;
        reg_word = reg_val;
        break;

    case CMD_SCTRL_MODEM_POWERDOWN:
        reg = SCTRL_PWR_MAC_MODEM;
        reg_val = REG_READ(SCTRL_PWR_MAC_MODEM);
        reg_val &= ~(MODEM_PWD_MASK << MODEM_PWD_POSI);
        reg_val |= MODEM_PWD << MODEM_PWD_POSI;
        reg_word = reg_val;
        break;

    case CMD_SCTRL_BLE_POWERDOWN:
        reg = SCTRL_USB_PWR;
        reg_val = REG_READ(SCTRL_USB_PWR);
        reg_val &= ~(BLE_PWD_MASK << BLE_PWD_POSI);
        reg_val |= BLE_PWD << BLE_PWD_POSI;
        reg_word = reg_val;
        break;

    case CMD_SCTRL_MODEM_POWERUP:
        reg = SCTRL_PWR_MAC_MODEM;
        reg_val = REG_READ(SCTRL_PWR_MAC_MODEM);
        reg_val &= ~(MODEM_PWD_MASK << MODEM_PWD_POSI);
        reg_val |= MODEM_PWU << MODEM_PWD_POSI;
        reg_word = reg_val;
        break;

     case CMD_SCTRL_BLE_POWERUP:
        reg = SCTRL_USB_PWR;
        reg_val = REG_READ(SCTRL_USB_PWR);
        reg_val &= ~(BLE_PWD_MASK << BLE_PWD_POSI);
        reg_val |= BLE_PWU << BLE_PWD_POSI;
        reg_word = reg_val;
        break;

    default:
        break;
    }

    if(reg)
    {
        REG_WRITE(reg, reg_word);
    }
}

void sctrl_subsys_reset(UINT32 cmd)
{
    UINT32 reg = 0;
    UINT32 reset_word = 0;

    switch(cmd)
    {
    case CMD_SCTRL_MODEM_SUBCHIP_RESET:
        reg = SCTRL_MODEM_SUBCHIP_RESET_REQ;
        reset_word = MODEM_SUBCHIP_RESET_WORD;
        break;

    case CMD_SCTRL_MAC_SUBSYS_RESET:
        reg = SCTRL_MAC_SUBSYS_RESET_REQ;
        reset_word = MAC_SUBSYS_RESET_WORD;
        break;

    case CMD_SCTRL_USB_SUBSYS_RESET:
        reg = SCTRL_USB_SUBSYS_RESET_REQ;
        reset_word = USB_SUBSYS_RESET_WORD;
        break;

    case CMD_SCTRL_DSP_SUBSYS_RESET:
        reg = SCTRL_DSP_SUBSYS_RESET_REQ;
        reset_word = DSP_SUBSYS_RESET_WORD;
        break;

    default:
        break;
    }

    if(reg)
    {
        REG_WRITE(reg, reset_word);
        delay(10);
        REG_WRITE(reg, 0);
    }

    return;
}

#if (CFG_SOC_NAME == SOC_BK7231N)
void sctrl_fix_dpll_div(void)
{
    if ((DEVICE_ID_BK7231N_P & DEVICE_ID_MASK) == (sctrl_ctrl(CMD_GET_DEVICE_ID, NULL) & DEVICE_ID_MASK)) {
		return ;
	}

	uint32 reg;
	uint32 cpu_clock;

	GLOBAL_INT_DECLARATION();
	GLOBAL_INT_DISABLE();

	reg = REG_READ(SCTRL_CONTROL);
	cpu_clock = reg & 0xFF;
	reg = (reg & 0xFFFFFF00) | 0x52;
	REG_WRITE(SCTRL_CONTROL, reg);

	delay(1);

	REG_WRITE(SCTRL_MODEM_SUBCHIP_RESET_REQ, MODEM_SUBCHIP_RESET_WORD);
	REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) | (1 << 14));

	delay(1);

	REG_WRITE(SCTRL_MODEM_SUBCHIP_RESET_REQ, 0);
	REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) & ~(1 << 14));

	delay(1);

	reg = REG_READ(SCTRL_CONTROL);
	reg = (reg & 0xFFFFFF00) | cpu_clock;
	REG_WRITE(SCTRL_CONTROL, reg);

	delay(1);

	GLOBAL_INT_RESTORE();
}
#endif

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
void sctrl_mdm_reset(void)
{
    volatile INT32 i;
    GLOBAL_INT_DECLARATION();

    os_printf("sctrl_mdm_reset\r\n");

    // Disable the interrupts
    GLOBAL_INT_DISABLE();

    if (1)
    {
        /* MAC reset */
        REG_WRITE(SCTRL_MODEM_SUBCHIP_RESET_REQ, MODEM_SUBCHIP_RESET_WORD);
        REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) | DPLL_CLKDIV_RESET_BIT);

        for(i = 0; i < 100; i++);

        REG_WRITE(SCTRL_MODEM_SUBCHIP_RESET_REQ, 0);
        REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) & ~DPLL_CLKDIV_RESET_BIT);

        for(i = 0; i < 100; i++);
    }
    else
    {
        /* Modem pwd */
        sctrl_ctrl(CMD_SCTRL_MODEM_POWERDOWN, NULL);
        REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) | DPLL_CLKDIV_RESET_BIT);

        for(i = 0; i < 100; i++);

        /* Modem pwu */
        sctrl_ctrl(CMD_SCTRL_MODEM_POWERUP, NULL);
        REG_WRITE(SCTRL_CONTROL, REG_READ(SCTRL_CONTROL) & ~DPLL_CLKDIV_RESET_BIT);

        for(i = 0; i < 100; i++);
    }

#if (CFG_SOC_NAME == SOC_BK7238)
    mpb_regs_reset();
#endif
    phy_wakeup_rf_reinit();
    phy_wakeup_wifi_reinit();

    // Restore the interrupts
    GLOBAL_INT_RESTORE();
}
#endif

#if CFG_USE_FAKERTC_PS
UINT32 block_en_value;

int sctrl_unconditional_mac_set_doze(void)
{
    UINT32 reg;
    DD_HANDLE flash_hdl;
    UINT32 status;

    if(rwnxl_get_status_in_doze())
    {
        //os_printf("forbid 1\r\n");
        return -1;
    }
    nxmac_enable_idle_interrupt_setf(1);
    delay(1);

    reg = REG_READ(ICU_INTERRUPT_ENABLE);
    reg &= ~(CO_BIT(FIQ_MAC_TX_RX_MISC)
             | CO_BIT(FIQ_MAC_TX_RX_TIMER)
             | CO_BIT(FIQ_MAC_RX_TRIGGER)
             | CO_BIT(FIQ_MAC_TX_TRIGGER)
             | CO_BIT(FIQ_MAC_GENERAL)
             | CO_BIT(FIQ_MAC_PROT_TRIGGER));
    REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

    if(REG_READ((ICU_BASE + 19 * 4))
            & (CO_BIT(FIQ_MAC_TX_RX_MISC)
               | CO_BIT(FIQ_MAC_TX_RX_TIMER)
               | CO_BIT(FIQ_MAC_RX_TRIGGER)
               | CO_BIT(FIQ_MAC_TX_TRIGGER)
               | CO_BIT(FIQ_MAC_PROT_TRIGGER)
              ))
    {
        reg = REG_READ(ICU_INTERRUPT_ENABLE);
        reg |= (CO_BIT(FIQ_MAC_TX_RX_MISC)
                 | CO_BIT(FIQ_MAC_TX_RX_TIMER)
                 | CO_BIT(FIQ_MAC_RX_TRIGGER)
                 | CO_BIT(FIQ_MAC_TX_TRIGGER)
                 | CO_BIT(FIQ_MAC_GENERAL)
                 | CO_BIT(FIQ_MAC_PROT_TRIGGER));
        REG_WRITE(ICU_INTERRUPT_ENABLE, reg);
        //os_printf("forbid 2\r\n");
        return -1;
    }

    if(wifi_mac_state_set_idle() != 0)
    {
        reg = REG_READ(ICU_INTERRUPT_ENABLE);
        reg |= (CO_BIT(FIQ_MAC_TX_RX_MISC)
                 | CO_BIT(FIQ_MAC_TX_RX_TIMER)
                 | CO_BIT(FIQ_MAC_RX_TRIGGER)
                 | CO_BIT(FIQ_MAC_TX_TRIGGER)
                 | CO_BIT(FIQ_MAC_GENERAL)
                 | CO_BIT(FIQ_MAC_PROT_TRIGGER));
        REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

        //os_printf("forbid 3\r\n");
        return -1;
    }
	return 0;
}

int sctrl_unconditional_mac_exit_doze(void)
{
    UINT32 reg;

    delay(10);
	PS_DEBUG_BCN_TRIGER;

    wifi_mac_state_set_prev();
    	PS_DEBUG_BCN_TRIGER;

    reg = REG_READ(ICU_INTERRUPT_ENABLE);
    reg |= (CO_BIT(FIQ_MAC_TX_RX_MISC)
             | CO_BIT(FIQ_MAC_TX_RX_TIMER)
             | CO_BIT(FIQ_MAC_RX_TRIGGER)
             | CO_BIT(FIQ_MAC_TX_TRIGGER)
             | CO_BIT(FIQ_MAC_GENERAL)
             | CO_BIT(FIQ_MAC_PROT_TRIGGER));
    reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
    REG_WRITE(ICU_INTERRUPT_ENABLE, reg);
}

volatile uint32_t g_mac_sleep_flag = 0;

int sctrl_unconditional_normal_sleep(UINT32 peri_clk)
{
    UINT32 reg;
    DD_HANDLE flash_hdl;
    UINT32 status;
    PS_DEBUG_DOWN_TRIGER;

    if(sctrl_unconditional_mac_set_doze() != 0)
    {
        return -1;
    }

    ///sctrl_ctrl(CMD_SCTRL_CLOSE_ADC_MIC_ANALOG, NULL);
    ///sctrl_ctrl(CMD_SCTRL_CLOSE_DAC_ANALOG, NULL);

    uart_wait_tx_over();

    reg = RF_HOLD_BY_STA_BIT;
    sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_CLR, &reg);
	g_mac_sleep_flag = 1;
    sctrl_unconditional_mac_sleep();

    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(2);
    }

    PS_DEBUG_DOWN_TRIGER;

#if (CFG_SOC_NAME == SOC_BK7231)
    REG_WRITE(SCTRL_ROSC_CAL, 0x35);
    REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#endif


    if(sctrl_mcu_ps_info.mcu_use_dco == 0)
    {
        /* MCLK(main clock) select:dco*/
        sctrl_mclk_select(MCLK_MODE_DCO, MCLK_DIV_0);
        PS_DEBUG_DOWN_TRIGER;
    }


    /*close 32K Rosc calib*/
#if (CFG_SOC_NAME == SOC_BK7231)
    REG_WRITE(SCTRL_ROSC_CAL, 0x36);
#endif
    PS_DEBUG_DOWN_TRIGER;
    /* dpll division reset*/
    reg = REG_READ(SCTRL_CONTROL);
    reg |= (0x1 << 14);
    REG_WRITE(SCTRL_CONTROL, reg);
    PS_DEBUG_DOWN_TRIGER;
    /* dpll (480m) & xtal2rf  disable*/
    reg = REG_READ(SCTRL_BLOCK_EN_CFG);
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg = reg | (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg &= ~(BLK_EN_DPLL_480M | BLK_EN_XTAL2RF );
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
    PS_DEBUG_DOWN_TRIGER;
#if (CFG_SOC_NAME != SOC_BK7221U)
    /* center bias power down*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg &= (~ CENTRAL_BAIS_ENABLE_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
#endif
    sctrl_mcu_ps_info.hw_sleep = 1;
#if (CFG_SOC_NAME != SOC_BK7221U)
    while(sctrl_analog_get(SCTRL_ANALOG_CTRL2) & CENTRAL_BAIS_ENABLE_BIT);
#endif
    PS_DEBUG_DOWN_TRIGER;

    reg = REG_READ(ICU_ARM_WAKEUP_EN);
    reg = (/*MAC_ARM_WAKEUP_EN_BIT|*/TIMER_ARM_WAKEUP_EN_BIT
        |UART1_ARM_WAKEUP_EN_BIT|UART2_ARM_WAKEUP_EN_BIT| GPIO_ARM_WAKEUP_EN_BIT);
    REG_WRITE(ICU_ARM_WAKEUP_EN, reg);
    block_en_value = REG_READ(SCTRL_BLOCK_EN_CFG);;



#if 1
    /* close all peri clock*/
    ps_saves[0].peri_clk_cfg= REG_READ(ICU_PERI_CLK_PWD);
    REG_WRITE(ICU_PERI_CLK_PWD, peri_clk);
#endif

    ps_delay(1);
    /* arm clock disable */
    reg = REG_READ(SCTRL_SLEEP);
    reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
    reg = reg | SLEEP_MODE_CFG_NORMAL_VOL_WORD;
    REG_WRITE(SCTRL_SLEEP, reg);
    ps_delay(1);//5

    return 0;
}

void sctrl_unconditional_normal_wakeup()
{
    UINT32 reg;

	PS_DEBUG_BCN_TRIGER;
#if (CFG_SOC_NAME != SOC_BK7221U)
    /* center bias power on*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg |= CENTRAL_BAIS_ENABLE_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);

    while((sctrl_analog_get(SCTRL_ANALOG_CTRL2) & CENTRAL_BAIS_ENABLE_BIT)  == 0);
#endif
    /*dpll(480m)  & xtal2rf enable*/
    reg = REG_READ(SCTRL_BLOCK_EN_CFG);
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg |= (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg |= (BLK_EN_DPLL_480M | BLK_EN_XTAL2RF );
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
    ps_delay(10);
    PS_DEBUG_BCN_TRIGER;

    if(sctrl_mcu_ps_info.mcu_use_dco == 0)
    {
        /* MCLK(main clock) select:26M*/
        sctrl_mclk_select(MCLK_MODE_26M_XTAL, MCLK_DIV_0);

        ps_delay(500);
	    PS_DEBUG_BCN_TRIGER;
        /* dpll division reset release*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(0x1<<14);
        REG_WRITE(SCTRL_CONTROL, reg);

        /* MCLK(main clock) select:dpll*//* MCLK division*/
        sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_7);
	    PS_DEBUG_BCN_TRIGER;

    }
    else
    {
        ps_delay(500);

        /* dpll division reset release*/
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(0x1<<14);
        REG_WRITE(SCTRL_CONTROL, reg);
	    PS_DEBUG_BCN_TRIGER;

    }

    sctrl_mcu_ps_info.hw_sleep = 0;
    sctrl_cali_dpll(1);
    PS_DEBUG_BCN_TRIGER;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CLR_DPLL_UNLOOK_INT_BIT, NULL);

    /*open 32K Rosc calib*/
#if (CFG_SOC_NAME == SOC_BK7231)
    REG_WRITE(SCTRL_ROSC_CAL, 0x35);
    REG_WRITE(SCTRL_ROSC_CAL, 0x37);
#endif
    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(4);
    }

    #if 1
    REG_WRITE(ICU_PERI_CLK_PWD, ps_saves[0].peri_clk_cfg);
    #endif

	g_mac_sleep_flag = 0;
    sctrl_unconditional_mac_wakeup();
    delay(10);

    reg = RF_HOLD_BY_STA_BIT;
    sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_SET, &reg);
    delay(10);

    wifi_mac_state_set_prev();

	PS_DEBUG_BCN_TRIGER;

    reg = REG_READ(ICU_INTERRUPT_ENABLE);
    reg |= (CO_BIT(FIQ_MAC_TX_RX_MISC)
             | CO_BIT(FIQ_MAC_TX_RX_TIMER)
             | CO_BIT(FIQ_MAC_RX_TRIGGER)
             | CO_BIT(FIQ_MAC_TX_TRIGGER)
             | CO_BIT(FIQ_MAC_GENERAL)
             | CO_BIT(FIQ_MAC_PROT_TRIGGER));
    reg &= ~(CO_BIT(FIQ_MAC_WAKEUP));
    REG_WRITE(ICU_INTERRUPT_ENABLE, reg);

    delay(10);

    return;
}
#endif

#if CFG_USE_FORCE_LOWVOL_PS
int bk_get_lv_sleep_wakeup_gpio_status(void)
{
	gpio_0_31_status = REG_READ(SCTRL_GPIO_WAKEUP_INT_STATUS);
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	gpio_32_39_status = REG_READ(SCTRL_GPIO_WAKEUP_INT_STATUS1);
#endif
	return 0;
}

#endif

#if CFG_USE_DEEP_PS

#if PS_SUPPORT_MANUAL_SLEEP
void sctrl_enter_rtos_idle_sleep(PS_DEEP_CTRL_PARAM deep_param)
{
    DD_HANDLE flash_hdl;
    UINT32 status;
    UINT32 param;
    UINT32 reg;

    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(2);
    }

	uart_wait_tx_over();
    /* close all peri clock*/
    ps_saves[1].peri_clk_cfg= REG_READ(ICU_PERI_CLK_PWD);
    REG_WRITE(ICU_PERI_CLK_PWD, 0xfffff);

    uart2_exit();
    #if CFG_USE_UART1
    uart1_exit();
    #endif

    reg = REG_READ(SCTRL_ROSC_TIMER);
    reg &= ~ (ROSC_TIMER_ENABLE_BIT);
    REG_WRITE(SCTRL_ROSC_TIMER,reg);

    REG_WRITE(SCTRL_GPIO_WAKEUP_EN,0x0);

    REG_WRITE(SCTRL_BLOCK_EN_MUX, 0x0);

    param = LPO_SELECT_ROSC;
    sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_SET_LOW_PWR_CLK, &param);

    /* close all peri int*/
    ps_saves[1].int_enable_cfg = REG_READ(ICU_INTERRUPT_ENABLE);
    REG_WRITE(ICU_INTERRUPT_ENABLE, 0);

	/*Enable BK7011:rc_en,ch0_en*/
    rc_cntl_stat_set(0x0);

    /* MAC pwd*/
    REG_WRITE(SCTRL_PWR_MAC_MODEM, MAC_PWD << MAC_PWD_POSI);

    sctrl_mac_ahb_slave_clock_disable();

    /* Mac Subsystem clock 480m disable*/
    reg = REG_READ(SCTRL_CONTROL);
    REG_WRITE(SCTRL_CONTROL, reg | MAC_CLK480M_PWD_BIT);

    /* Modem pwd*/
    REG_WRITE(SCTRL_PWR_MAC_MODEM, MODEM_PWD << MODEM_PWD_POSI);

    /* Modem AHB clock disable*/
    reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
    reg &= ~PHY_HCLK_EN_BIT;
    REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

    /* Modem Subsystem clock 480m disable*/
    reg = REG_READ(SCTRL_CONTROL);
    REG_WRITE(SCTRL_CONTROL, reg | MODEM_CLK480M_PWD_BIT);

    /* Flash 26MHz clock select dco clock*/
    flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
    ASSERT(DD_HANDLE_UNVALID != flash_hdl);
    ddev_control(flash_hdl, CMD_FLASH_SET_DCO, 0);

    /* MCLK(main clock) select:dco*/ /* MCLK division*/
    sctrl_mclk_select(MCLK_MODE_DCO, MCLK_DIV_0);

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

    ps_delay(10);

    /*close 32K Rosc calib*/
#if(CFG_SOC_NAME == SOC_BK7238)
    rosc_calib_auto_trigger_disable();
#else
    REG_WRITE(SCTRL_ROSC_CAL, 0x36);
#endif

    ps_block_value = REG_READ(SCTRL_BLOCK_EN_CFG) & BLOCK_EN_VALID_MASK;
    /* ALL disable*/
    reg = 0x0;
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg = reg | (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg &= ~(BLOCK_EN_VALID_MASK);
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);

    /* center bias power down*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg &= (~CENTRAL_BAIS_ENABLE_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);

    reg = 0xFFFFFFFF;
    REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS,reg);

    if(deep_param.deep_wkway == PS_DEEP_WAKEUP_RTC
        && deep_param.param != 0xffffffff)
    {
        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg |= ROSC_TIMER_INT_STATUS_BIT;
        REG_WRITE(SCTRL_ROSC_TIMER,reg);
        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg &= ~(ROSC_TIMER_PERIOD_MASK << ROSC_TIMER_PERIOD_POSI);
        reg |= (deep_param.param << ROSC_TIMER_PERIOD_POSI);
        REG_WRITE(SCTRL_ROSC_TIMER,reg);
        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg |= ROSC_TIMER_ENABLE_BIT;
        REG_WRITE(SCTRL_ROSC_TIMER,reg);

        reg = 0x0;
        reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
        reg = reg | (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
        reg |= (BLK_EN_ANALOG_SYS_LDO | BLK_EN_DIGITAL_CORE);
        REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
        reg = REG_READ(SCTRL_BLOCK_EN_MUX);
        reg &= (~0x1FF);
        reg |= (0x1 << 8);
        REG_WRITE(SCTRL_BLOCK_EN_MUX, reg);
    }
    else if(deep_param.deep_wkway == PS_DEEP_WAKEUP_GPIO)
    {
        reg = 0xFFFFFFFF;
        REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS,reg);
        reg = deep_param.gpio_lv;
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE,reg);
        reg = deep_param.param;
        REG_WRITE(SCTRL_GPIO_WAKEUP_EN,reg);

    }

    ps_delay(10);
    /* arm clock disable */
    reg = REG_READ(SCTRL_SLEEP);
    reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
    reg = reg | SLEEP_MODE_CFG_LOW_VOL_WORD;
    REG_WRITE(SCTRL_SLEEP, reg);
    delay(5);
}

void sctrl_exit_rtos_idle_sleep(void)
{
    UINT32 reg;

    /* center bias power on*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg |= CENTRAL_BAIS_ENABLE_BIT;
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);

    /*dpll(480m) & 26m xtal & xtal2rf enable*/
    reg = REG_READ(SCTRL_BLOCK_EN_MUX);
    reg &= ~(0x1 << 4);
    REG_WRITE(SCTRL_BLOCK_EN_MUX, reg);
    reg = REG_READ(SCTRL_BLOCK_EN_CFG);
    reg &= ~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI);
    reg |= (BLOCK_EN_WORD_PWD << BLOCK_EN_WORD_POSI);
    reg |= ps_block_value;
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);

    delay(20);  //delay 1.1ms for 26MHz DCO clock. need change for other dco clock frequency
    sctrl_cali_dpll(1);
    /* dpll division reset release*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(0x1<<14);
    REG_WRITE(SCTRL_CONTROL, reg);

    /* MCLK(main clock) select:dpll*//* MCLK division*/
    sctrl_mclk_select(MCLK_MODE_DPLL, MCLK_DIV_7);

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

    sctrl_mac_ahb_slave_clock_enable();

    /* Mac Subsystem clock 480m enable*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~MAC_CLK480M_PWD_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);

    /* Modem AHB clock enable*/
    reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
    REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg | PHY_HCLK_EN_BIT);

    /* Modem Subsystem clock 480m enable*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~MODEM_CLK480M_PWD_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);

    /* Modem pwd*/
    reg = REG_READ(SCTRL_PWR_MAC_MODEM);
    reg &= ~(MODEM_PWD_MASK << MODEM_PWD_POSI);
    reg = reg | (0 << MODEM_PWD_POSI);
    REG_WRITE(SCTRL_PWR_MAC_MODEM, reg);

	/*Enable BK7011:rc_en,ch0_en*/
    rc_cntl_stat_set(0x09);

    /* recovery periphral unit clock config*/
    REG_WRITE(ICU_PERI_CLK_PWD, ps_saves[1].peri_clk_cfg);

    /* recovery periphral unit int config*/
    REG_WRITE(ICU_INTERRUPT_ENABLE, ps_saves[1].int_enable_cfg);

    uart2_init();
    #if CFG_USE_UART1
    uart1_init();
    #endif

    if(4 == flash_get_line_mode())
    {
        flash_set_line_mode(4);
    }

    os_printf("idle wake up!\r\n");
}
#endif

void sctrl_deep_sleep_wkup_source_check(PS_DEEP_CTRL_PARAM *deep_param)
{
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)

    UINT32 param = 0;

    if ((deep_param->wake_up_way & PS_DEEP_WAKEUP_GPIO)&&(deep_param->wake_up_way & PS_DEEP_WAKEUP_USB))
    {
        /* conflict, disable corresponding GPIO and choose USB for wakeup */
        if(deep_param->gpio_index_map & (0x01UL << GPIO25))
        {
            /* disable GPIO25 , other GPIOs still can be used for wakeup */
            deep_param->gpio_index_map &= ~(0x01UL << GPIO25);
            os_printf("GPIO25 olny used for USB wakeup!\r\n");
        }
        if(deep_param->gpio_index_map & (0x01UL << GPIO28))
        {
            /* disable GPIO28 , other GPIOs still can be used for wakeup */
            deep_param->gpio_index_map &= ~(0x01UL << GPIO28);
            os_printf("GPIO28 olny used for USB wakeup!\r\n");
        }
    }
    else if(deep_param->wake_up_way & PS_DEEP_WAKEUP_GPIO)
    {
        /* gpio25 or gpio28 negedge wakeup ,have to power down USB system ,otherwise it will produce conflict */
        if(((deep_param->gpio_index_map & (0x01UL << GPIO25))&&( deep_param->gpio_edge_map & (0x01UL << GPIO25)))
        ||((deep_param->gpio_index_map & (0x01UL << GPIO28))&&( deep_param->gpio_edge_map & (0x01UL << GPIO28))))
        {
            sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_USB_POWERDOWN, &param);
        }
    }
    else
    {
        ;
    }
#endif
}

void sctrl_enter_rtos_deep_sleep(PS_DEEP_CTRL_PARAM *deep_param)
{
    DD_HANDLE flash_hdl;
    UINT32 status;
    UINT32 param;
    UINT32 reg;
    UINT32 i;
    sctrl_deep_sleep_wkup_source_check(deep_param);
    uart_wait_tx_over();

    /* close all peri clock*/
    REG_WRITE(ICU_PERI_CLK_PWD, 0xfffff);  //  icu: 0x2;

#if CFG_USE_UART2
	uart2_exit();
#endif
#if CFG_USE_UART1
	uart1_exit();
#endif

#if (CFG_SOC_NAME == SOC_BK7231U) || (SOC_BK7231N == CFG_SOC_NAME) || (CFG_SOC_NAME == SOC_BK7238)
    reg = REG_READ(SCTRL_LOW_PWR_CLK);
    reg &=~(LPO_CLK_MUX_MASK);
    reg |=(LPO_SRC_ROSC << LPO_CLK_MUX_POSI);
    REG_WRITE(SCTRL_LOW_PWR_CLK, reg);
    #if(CFG_SOC_NAME == SOC_BK7238)
    /*in order to output 26M clock,will make average current higher*/
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
    reg |= (AMPBIAS_OUTEN_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL3, reg);
    while((sctrl_analog_get(SCTRL_ANALOG_CTRL3) & AMPBIAS_OUTEN_BIT) == 0);

    rosc_calib_manual_trigger();
    rosc_calib_auto_trigger(4);//2s
    #else
    REG_WRITE(SCTRL_ROSC_CAL, 0x75);
    REG_WRITE(SCTRL_ROSC_CAL, 0x77);
    #endif
#else
    /*ana_reg set*/
    REG_WRITE(SCTRL_ANALOG_CTRL0, 0x7819a59b);
    REG_WRITE(SCTRL_ANALOG_CTRL1, 0x7819a59b);
    REG_WRITE(SCTRL_ANALOG_CTRL2, 0x84036080);
    REG_WRITE(SCTRL_ANALOG_CTRL3, 0x180004a0);
    REG_WRITE(SCTRL_ANALOG_CTRL4, 0x84200e52);
    REG_WRITE(SCTRL_ANALOG_CTRL5, 0x3b13b13b);
#if (CFG_SOC_NAME != SOC_BK7231)
    REG_WRITE(SCTRL_ANALOG_CTRL6, 0xb09350);
#endif
#if (CFG_SOC_NAME == SOC_BK7221U)
    REG_WRITE(SCTRL_ANALOG_CTRL7, 0x441a7f0);
    REG_WRITE(SCTRL_ANALOG_CTRL8, 0x3b187c);
    REG_WRITE(SCTRL_ANALOG_CTRL9, 0x82204007);
    REG_WRITE(SCTRL_ANALOG_CTRL10, 0x80801432);
#endif
#endif
    ps_delay(10);
    if(PS_DEEP_WAKEUP_RTC != deep_param->wake_up_way)
    {
        #if(CFG_SOC_NAME == SOC_BK7238)
        rosc_calib_auto_trigger_disable();
        #else
        REG_WRITE(SCTRL_ROSC_CAL, 0x36);
        #endif
    }
    /*clear int*/
    REG_WRITE(ICU_INTERRUPT_ENABLE, 0);

    extern void gpio_ops_disable_filter(void);
    gpio_ops_disable_filter();

	/*  disable gpio0~31*/
    REG_WRITE(SCTRL_GPIO_WAKEUP_EN,0x0);  //sys_ctrl : 0x48;
    REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS,0xFFFFFFFF);  //sys_ctrl : 0x4a;

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
	/*	disable gpio32~39*/
	REG_WRITE(SCTRL_GPIO_WAKEUP_EN1,0x0);  //sys_ctrl : 0x51;
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS1,0xFF);  //sys_ctrl : 0x53;
#endif

    REG_WRITE(SCTRL_BLOCK_EN_MUX, 0x0);   //sys_ctrl : 0x4F;

    /* ROSC_TIMER_int_clear*/
    reg = REG_READ(SCTRL_ROSC_TIMER);
   	reg = reg| ROSC_TIMER_INT_STATUS_BIT ;
    REG_WRITE(SCTRL_ROSC_TIMER,reg);      //sys_ctrl : 0x47;

	/*ROSC_TIMER close */
	reg = REG_READ(SCTRL_ROSC_TIMER);
	reg = reg & (~ROSC_TIMER_ENABLE_BIT);																   //'C'
	REG_WRITE(SCTRL_ROSC_TIMER,reg);

    reg = REG_READ(SCTRL_LOW_PWR_CLK);
    reg &=~(LPO_CLK_MUX_MASK);
    if(deep_param->lpo_32k_src == LPO_SELECT_32K_XTAL)
    {
        reg |=(LPO_SRC_32K_XTAL << LPO_CLK_MUX_POSI);
    }
    else if(deep_param->lpo_32k_src == LPO_SELECT_ROSC)
    {
        reg |=(LPO_SRC_ROSC << LPO_CLK_MUX_POSI);
    }
    else if (deep_param->lpo_32k_src == LPO_SELECT_32K_DIV)
    {
        reg |=(LPO_SRC_32K_DIV << LPO_CLK_MUX_POSI);
    }

    REG_WRITE(SCTRL_LOW_PWR_CLK, reg);    //sys_ctrl : 0x40;

    /* close all peri int*/
	//    REG_WRITE(ICU_INTERRUPT_ENABLE, 0);

    /* MAC pwd*/
    REG_WRITE(SCTRL_PWR_MAC_MODEM, MAC_PWD << MAC_PWD_POSI);   //sys_ctrl : 0x43;

    sctrl_mac_ahb_slave_clock_disable();

    /* Mac Subsystem clock 480m disable*/
    reg = REG_READ(SCTRL_CONTROL);
    REG_WRITE(SCTRL_CONTROL, reg | MAC_CLK480M_PWD_BIT);

    /* Modem pwd*/
    REG_WRITE(SCTRL_PWR_MAC_MODEM, MODEM_PWD << MODEM_PWD_POSI);

    /* Modem AHB clock disable*/
    reg = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
    reg &= ~PHY_HCLK_EN_BIT;
    REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

    /* Modem Subsystem clock 480m disable*/
    reg = REG_READ(SCTRL_CONTROL);
    REG_WRITE(SCTRL_CONTROL, reg | MODEM_CLK480M_PWD_BIT);

    /* Flash 26MHz clock select dco clock*/
    flash_hdl = ddev_open(FLASH_DEV_NAME, &status, 0);
    ASSERT(DD_HANDLE_UNVALID != flash_hdl);
    ddev_control(flash_hdl, CMD_FLASH_SET_DCO, 0);

    /* MCLK(main clock) select:dco*/ /* MCLK division*/
    sctrl_mclk_select(MCLK_MODE_DCO, MCLK_DIV_0);

    if(deep_param->lpo_32k_src == LPO_SELECT_32K_XTAL)
    {
        reg = REG_READ(SCTRL_CONTROL);
        reg =((reg & (~0xF0)) | (0<<4));
        reg =((reg & (~0x03)) | (0<<MCLK_MUX_POSI));
        reg =((reg & (~0x100)) | FLASH_26M_MUX_BIT);
        REG_WRITE(SCTRL_CONTROL,reg); //sys_ctrl : 0x02;
    }

    ps_delay(10);

    reg = 0x0;
   	reg = (reg &(~(BLOCK_EN_WORD_MASK << 20))&(~(0x7FFFUL<<5)) &(~(0x01UL<<1)));
   	reg = (reg |(BLOCK_EN_WORD_PWD<< 20 )|BLK_EN_FLASH|BLK_EN_DIGITAL_CORE|BLK_EN_ANALOG_SYS_LDO);
    if(deep_param->lpo_32k_src == LPO_SELECT_32K_XTAL)
    {
        reg = (reg |BLK_EN_32K_XTAL|BLK_EN_26M_XTAL);
    }
    else if(deep_param->lpo_32k_src == LPO_SELECT_ROSC)
    {
        reg = (reg |BLK_EN_ROSC32K);//test
    }
    else if(deep_param->lpo_32k_src == LPO_SELECT_32K_DIV)
    {
        reg = (reg |BLK_EN_26M_XTAL);
    }
    REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);                  //sys_ctrl : 0x4B;                   //'E'

#if (CFG_SOC_NAME != SOC_BK7231U) && (SOC_BK7231N != CFG_SOC_NAME) && (CFG_SOC_NAME != SOC_BK7238)
    reg = REG_READ(SCTRL_ROSC_CAL);                           //ROSC Calibration disable
    reg =(reg  & (~0x01));
    REG_WRITE(SCTRL_ROSC_CAL, reg);
#endif

    for(i=0; i<GPIONUM; i++)
    {
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        if(((i > GPIO1) && (i < GPIO6))
            || ((i > GPIO11) && (i < GPIO14))
            || ((i > GPIO17) && (i < GPIO20))
            || ((i > GPIO24) && (i < GPIO26))
            || ((i > GPIO26) && (i < GPIO28)))
        {
            continue;
        }
#endif
        if(((i < BITS_INT)&&(deep_param->gpio_stay_lo_map & (0x01UL << i)))
        ||((deep_param->gpio_index_map & (0x01UL << i)))
        ||((deep_param->gpio_last_index_map & (0x01UL << i)))
        ||((i >= BITS_INT)&&(deep_param->gpio_stay_hi_map & (0x01UL << (i - BITS_INT)))) )
        {
            continue;
        }
        param = GPIO_CFG_PARAM(i, GMODE_DEEP_PS);	/*set gpio 0~39 as high impendance*/
        sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
    }

    if (((deep_param->wake_up_way & PS_DEEP_WAKEUP_RTC))
        && (deep_param->sleep_time!= 0xffffffff))
    {
	/*ROSC_TIMER  init*/
#if (CFG_SOC_NAME != SOC_BK7231)
        reg = (deep_param->sleep_time >> 16)& 0xffff;                                          //'A'
        REG_WRITE(SCTRL_ROSC_TIMER_H,reg);
#endif

        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg |= ROSC_TIMER_INT_STATUS_BIT;                                                                   //'C'
        REG_WRITE(SCTRL_ROSC_TIMER,reg);  //sys_ctrl : 0x47;

        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg &= ~(ROSC_TIMER_PERIOD_MASK << ROSC_TIMER_PERIOD_POSI);
        reg |= ((deep_param->sleep_time & ROSC_TIMER_PERIOD_MASK) << ROSC_TIMER_PERIOD_POSI);
        REG_WRITE(SCTRL_ROSC_TIMER,reg);   //sys_ctrl : 0x47;                         //'D'

        reg = REG_READ(SCTRL_ROSC_TIMER);
        reg |= ROSC_TIMER_ENABLE_BIT;
        REG_WRITE(SCTRL_ROSC_TIMER,reg);  //sys_ctrl : 0x47;                             //'B'

        if(deep_param->lpo_32k_src == LPO_SELECT_32K_XTAL)
        {
    		REG_WRITE(SCTRL_CONTROL, 0x330100);
            REG_WRITE(SCTRL_BLOCK_EN_CFG, (0x15D|(0xA5C<<20)));
            REG_WRITE(SCTRL_ROSC_CAL, 0x30);
            REG_WRITE(SCTRL_LOW_PWR_CLK, 0x01);
            REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, 0x03);
            REG_WRITE(SCTRL_CLK_GATING, 0x1ff);
        }
    }

#if ((CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238))
    if ((deep_param->wake_up_way & PS_DEEP_WAKEUP_GPIO))
    {
        for (i = 0; i < BITS_INT; i++)
        {
#if(BITS_INT > GPIONUM)
            if(i >= GPIONUM)
            {
                break;
            }
#endif
            if (deep_param->gpio_index_map & (0x01UL << i))			/*set gpio 0~31 mode*/
            {
                if( deep_param->gpio_edge_map & (0x01UL << i))      //0:high,1:low.
                {
                    if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
                    {
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT);
                    } else {
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLUP);
                    }

                    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                    if(0x1 != (UINT32)gpio_ctrl( CMD_GPIO_INPUT, &i))
                    {   /*check gpio really input value,to correct wrong edge setting*/
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT);
                        sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                        deep_param->gpio_edge_map &= ~(0x01UL << i);
                    }
                }
                else
                {
                    if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
                    {
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT);
                    } else {
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLDOWN);
                    }

                    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                    if(0x0 != (UINT32)gpio_ctrl( CMD_GPIO_INPUT, &i))
                    {   /*check gpio really input value,to correct wrong edge setting*/
                        param = GPIO_CFG_PARAM(i, GMODE_INPUT);
                        sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                        deep_param->gpio_edge_map |= (0x01UL << i);
                    }
                }
            }
        }

        for (i = 0; i < (GPIONUM - BITS_INT); i++)
        {
            if (deep_param->gpio_last_index_map & (0x01UL << i))				/*set gpio 32~39 mode*/
            {
                if( deep_param->gpio_last_edge_map  & (0x01UL << i))
                {
                    if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
                    {
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT);
                    } else {
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT_PULLUP);
                    }

                    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                    reg = i + BITS_INT;
                    if(0x1 != (UINT32)gpio_ctrl( CMD_GPIO_INPUT, &reg))
                    {   /*check gpio really input value,to correct wrong edge setting*/
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT);
                        sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                        deep_param->gpio_last_edge_map &= ~(0x01UL << i);
                    }
                }
                else
                {
                    if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
                    {
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT);
                    } else {
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT_PULLDOWN);
                    }

                    sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                    reg = i + BITS_INT;
                    if(0x0 != (UINT32)gpio_ctrl( CMD_GPIO_INPUT, &reg))
                    {   /*check gpio really input value,to correct wrong edge setting*/
                        param = GPIO_CFG_PARAM(i + BITS_INT, GMODE_INPUT);
                        sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
                        deep_param->gpio_last_edge_map |= (0x01UL << i);
                    }
                }
            }
        }

		/* set gpio 0~31 mode*/
        reg = 0xFFFFFFFF;
        REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS,reg);
        reg = deep_param->gpio_edge_map;
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE,reg);
        reg = deep_param->gpio_index_map;
        REG_WRITE(SCTRL_GPIO_WAKEUP_EN,reg);

		/* set gpio 31~32 mode*/
        reg = 0xFF;
        REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS1,reg);

        reg = deep_param->gpio_last_edge_map;
        REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE1,reg);

        reg = deep_param->gpio_last_index_map;
        REG_WRITE(SCTRL_GPIO_WAKEUP_EN1,reg);
    }
#elif ((CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238))
    if(( deep_param->wake_up_way & PS_DEEP_WAKEUP_GPIO ))
    {
        for ( i = 0; i < BITS_INT; i++ )
        {
#if(BITS_INT > GPIONUM)
            if( i >= GPIONUM )
            {
                break;
            }
#endif
            if((( i > GPIO1 ) && ( i < GPIO6 ))
            || (( i > GPIO11 ) && ( i < GPIO14 ))
            || (( i > GPIO17 ) && ( i < GPIO20 ))
            || (( i > GPIO24 ) && ( i < GPIO26 ))
            || (( i > GPIO26 ) && ( i < GPIO28 )))
            {
                continue;
            }

			if ( deep_param->gpio_index_map & ( 0x01UL << i ))
			{
				int type_h,type_l;
				type_l = deep_param->gpio_edge_map;
				type_h = 0xFFFFFFFF;

				/* low level or negedge wakeup */
				if(( type_h & ( 0x01UL << i )) == ( type_l & ( 0x01UL << i )))
				{
					if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
					{
						param = GPIO_CFG_PARAM(i, GMODE_INPUT);
					} else {
						param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLUP);
					}

					sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
				}
				else    /* high level or posedge wakeup */
				{
					if(sctrl_get_deep_sleep_gpio_floating_map() & (0x01UL << i))
					{
						param = GPIO_CFG_PARAM(i, GMODE_INPUT);
					} else {
						param = GPIO_CFG_PARAM(i, GMODE_INPUT_PULLDOWN);
					}

					sddev_control(GPIO_DEV_NAME, CMD_GPIO_CFG, &param);
				}

				REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE, type_l);
				REG_WRITE(SCTRL_GPIO_WAKEUP_TYPE_SELECT, type_h);
			}
        }

        reg = deep_param->gpio_index_map;
        REG_WRITE(SCTRL_GPIO_WAKEUP_EN,reg);
    }
#endif
        delay(11);//106.4us ,at least 100us
        REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS,0xFFFFFFFF);

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
    REG_WRITE(SCTRL_USB_PLUG_WAKEUP,USB_PLUG_IN_INT_BIT|USB_PLUG_OUT_INT_BIT);
    if(deep_param->wake_up_way & PS_DEEP_WAKEUP_USB)
    {
        REG_WRITE(SCTRL_USB_PLUG_WAKEUP,USB_PLUG_IN_EN_BIT|USB_PLUG_OUT_EN_BIT);
    }
#endif

#ifdef BK_DEEP_SLEEP_DEBUG
	BK_DEEP_SLEEP_PRT("SCTRL_CONTROL=0x%08X\r\n", REG_READ(SCTRL_CONTROL));
	BK_DEEP_SLEEP_PRT("SCTRL_SLEEP=0x%08X\r\n", REG_READ(SCTRL_SLEEP));
	BK_DEEP_SLEEP_PRT("SCTRL_ROSC_TIMER=0x%08X\r\n", REG_READ(SCTRL_ROSC_TIMER));
	BK_DEEP_SLEEP_PRT("SCTRL_BLOCK_EN_CFG=0x%08X\r\n", REG_READ(SCTRL_BLOCK_EN_CFG));
	BK_DEEP_SLEEP_PRT("SCTRL_ROSC_CAL=0x%08X\r\n", REG_READ(SCTRL_ROSC_CAL));
	BK_DEEP_SLEEP_PRT("SCTRL_BLOCK_EN_MUX=0x%08X\r\n", REG_READ(SCTRL_BLOCK_EN_MUX));
	BK_DEEP_SLEEP_PRT("SCTRL_LOW_PWR_CLK=0x%08X\r\n", REG_READ(SCTRL_LOW_PWR_CLK));
	BK_DEEP_SLEEP_PRT("SCTRL_PWR_MAC_MODEM=0x%08X\r\n", REG_READ(SCTRL_PWR_MAC_MODEM));
	BK_DEEP_SLEEP_PRT("SCTRL_MODEM_CORE_RESET_PHY_HCLK=0x%08X\r\n", REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK));
	BK_DEEP_SLEEP_PRT("SCTRL_CLK_GATING=0x%08X\r\n", REG_READ(SCTRL_CLK_GATING));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_INT_STATUS=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_INT_STATUS));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_TYPE=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_TYPE));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_EN=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_EN));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_INT_STATUS1=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_INT_STATUS1));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_TYPE1=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_TYPE1));
	BK_DEEP_SLEEP_PRT("SCTRL_GPIO_WAKEUP_EN1=0x%08X\r\n", REG_READ(SCTRL_GPIO_WAKEUP_EN1));
#endif

    /* center bias power down*/
#if (CFG_SOC_NAME == SOC_BK7238)
    reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
    reg &= (~CENTRAL_BAIS_ENABLE_BIT);
    sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
#endif

    /* enter deep_sleep mode */
    reg = REG_READ(SCTRL_SLEEP);
    reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
    reg = reg | SLEEP_MODE_CFG_DEEP_WORD;
    REG_WRITE(SCTRL_SLEEP, reg);

    delay(5);
}

int bk_init_deep_wakeup_gpio_status(void)
{
	gpio_0_31_status = REG_READ(GPIO_WAKEUP_INT_STATUS_BAK);
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	gpio_32_39_status = REG_READ(GPIO_WAKEUP_INT_STATUS1_BAK);
#endif
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF);
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS1, 0xFFFFFFFF);
#endif
	return 0;
}

uint32_t bk_save_deep_get_wakeup_gpio_status(void)
{
	uint32_t wakeup_gpio_num = -1;

	if(gpio_0_31_status)
	{
		for(int i=0;i<32;i++)
		{
			if(gpio_0_31_status&0x01)
			{
				wakeup_gpio_num = i;
				break;
			}
			gpio_0_31_status = gpio_0_31_status >> 1;
		}
	}
	#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	else if(gpio_32_39_status)
	{
		for(int i=32;i<40;i++)
		{
			if(gpio_32_39_status&0x01)
			{
				wakeup_gpio_num = i;
				break;
			}
			gpio_32_39_status = gpio_32_39_status >> 1;
		}
	}
	#endif
	return wakeup_gpio_num;
}

void bk_save_deep_set_wakeup_gpio_status(uint32_t wakeup_gpio_num)
{
	if((wakeup_gpio_num >= 0) && (wakeup_gpio_num < 32))
	{
		gpio_0_31_status = (1 << wakeup_gpio_num);
	}
	#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	else if(wakeup_gpio_num < 40)
	{
		gpio_32_39_status = (1 << (wakeup_gpio_num - 32));
	}
	#endif
}

int bk_misc_wakeup_get_gpio_num(void)
{
	int wakeup_gpio_num = -1;

	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS, 0xFFFFFFFF);//clear status
	#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	REG_WRITE(SCTRL_GPIO_WAKEUP_INT_STATUS1, 0xFFFFFFFF);//clear status
	#endif

	if((0 == gpio_0_31_status)
	#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	&& (0 == gpio_32_39_status)
	#endif
	)
	{
		wakeup_gpio_num = -1;
		return wakeup_gpio_num;
	}

	if(gpio_0_31_status)
	{
		for(int i=0;i<32;i++)
		{
			if(gpio_0_31_status&0x01)
			{
				wakeup_gpio_num = i;
				break;
			}

			gpio_0_31_status = gpio_0_31_status >> 1;
		}
	}
	#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238)
	else if(gpio_32_39_status)
	{
		for(int i=32;i<40;i++)
		{
			if(gpio_32_39_status&0x01)
			{
				wakeup_gpio_num = i;
				break;
			}

			gpio_32_39_status = gpio_32_39_status >> 1;
		}
	}
	#endif

	return wakeup_gpio_num;
}


RESET_SOURCE_STATUS sctrl_get_deep_sleep_wake_soure(void)
{
    RESET_SOURCE_STATUS waked_source = 0;

    if(REG_READ(SCTRL_ROSC_TIMER) & ROSC_TIMER_INT_STATUS_BIT)
    {
        waked_source = RESET_SOURCE_DEEPPS_RTC;
    }
    else if(gpio_0_31_status
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
    || gpio_32_39_status
#endif
    )
    {
        waked_source = RESET_SOURCE_DEEPPS_GPIO;
    }
#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
    else if(REG_READ(SCTRL_USB_PLUG_WAKEUP) & (USB_PLUG_IN_INT_BIT | USB_PLUG_OUT_INT_BIT))
    {
        waked_source = RESET_SOURCE_DEEPPS_USB;
    }
#endif

    return waked_source;
}

void sctrl_set_deep_sleep_gpio_floating_map(UINT32 gpio_last_floating_map)
{
	deep_sleep_gpio_floating_map = gpio_last_floating_map;
}

UINT32 sctrl_get_deep_sleep_gpio_floating_map(void)
{
	 return deep_sleep_gpio_floating_map;
}

void sctrl_reboot_with_deep_sleep(UINT32 sleep_ms)
{
    UINT32 reg;

    sleep_ms = 32 * sleep_ms; //32000 * ms / 1000 = 32 * ms

    reg = (sleep_ms >> 16)& 0xffff;                                          //'A'
    REG_WRITE(SCTRL_ROSC_TIMER_H,reg);

    reg = REG_READ(SCTRL_ROSC_TIMER);
    reg |= ROSC_TIMER_INT_STATUS_BIT;                                       //'C'
    REG_WRITE(SCTRL_ROSC_TIMER,reg);  //sys_ctrl : 0x47;

    reg = REG_READ(SCTRL_ROSC_TIMER);
    reg &= ~(ROSC_TIMER_PERIOD_MASK << ROSC_TIMER_PERIOD_POSI);
    reg |= ((sleep_ms & ROSC_TIMER_PERIOD_MASK) << ROSC_TIMER_PERIOD_POSI);
    REG_WRITE(SCTRL_ROSC_TIMER,reg);   //sys_ctrl : 0x47;                         //'D'

    reg = REG_READ(SCTRL_ROSC_TIMER);
    reg |= ROSC_TIMER_ENABLE_BIT;
    REG_WRITE(SCTRL_ROSC_TIMER,reg);  //sys_ctrl : 0x47;                             //'B'

    /* enter deep_sleep mode */
    reg = REG_READ(SCTRL_SLEEP);
    reg &= ~(SLEEP_MODE_MASK << SLEEP_MODE_POSI);
    reg = reg | SLEEP_MODE_CFG_DEEP_WORD;
    REG_WRITE(SCTRL_SLEEP, reg);
}
#endif

#if (CFG_SOC_NAME != SOC_BK7231)
static int sctrl_read_efuse(void *param)
{
    UINT32 reg, ret = -1;
    EFUSE_OPER_PTR efuse;
    efuse = (EFUSE_OPER_PTR)param;

    if(efuse) {
        reg = REG_READ(SCTRL_EFUSE_CTRL);
        reg &= ~(EFUSE_OPER_ADDR_MASK << EFUSE_OPER_ADDR_POSI);
        reg &= ~(EFUSE_OPER_DIR);

        reg |= ((efuse->addr & EFUSE_OPER_ADDR_MASK) << EFUSE_OPER_ADDR_POSI);
        reg |= (EFUSE_OPER_EN);
        REG_WRITE(SCTRL_EFUSE_CTRL, reg);

        do {
            reg = REG_READ(SCTRL_EFUSE_CTRL);
        }while(reg & EFUSE_OPER_EN);

        reg = REG_READ(SCTRL_EFUSE_OPTR);
        if(reg & EFUSE_OPER_RD_DATA_VALID) {
            efuse->data = ((reg >> EFUSE_OPER_RD_DATA_POSI) & EFUSE_OPER_RD_DATA_MASK);
            ret = 0;
        } else {
            efuse->data = 0xFF;
        }
    }
    return ret;
}

static int check_efuse_can_write(UINT8 new_byte, UINT8 old_byte)
{
    if(new_byte == old_byte)
    {
        // no need to read
        return 1;
    }

    for(int i=0; i<8; i++)
    {
        UINT8 old_bit = ((old_byte >> i) & 0x01);
        UINT8 new_bit = ((new_byte >> i) & 0x01);

        if ((old_bit) && (!new_bit))
        {
            // can not change old from 1 to 0
            return 0;
        }
    }

    return 2;
}

static int sctrl_write_efuse(void *param)
{
	UINT32 reg, ret = -1;
	EFUSE_OPER_ST *efuse, efuse_bak;

#if (CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7238)
	os_printf("BK7251/7238 cannot write efuse via register\r\n");
	goto wr_exit;
#endif

	efuse = (EFUSE_OPER_PTR)param;
	if (efuse) {
		efuse_bak.addr = efuse->addr;
		efuse_bak.data = efuse->data;
		if (sctrl_read_efuse(&efuse_bak) == 0) {
			//read before write, ensure this byte and this bit no wrote
			ret = check_efuse_can_write(efuse->data, efuse_bak.data);
			if (ret == 0) {
				ret = -1;
				goto wr_exit;
			} else if (ret == 1) {
				ret = 0;
				goto wr_exit;
			}
		}

		// enable vdd2.5v first
		reg = REG_READ(SCTRL_CONTROL);
		reg |= EFUSE_VDD25_EN;
		REG_WRITE(SCTRL_CONTROL, reg);

		reg = REG_READ(SCTRL_EFUSE_CTRL);
		reg &= ~(EFUSE_OPER_ADDR_MASK << EFUSE_OPER_ADDR_POSI);
		reg &= ~(EFUSE_OPER_WR_DATA_MASK << EFUSE_OPER_WR_DATA_POSI);

		reg |= EFUSE_OPER_DIR;
		reg |= ((efuse->addr & EFUSE_OPER_ADDR_MASK) << EFUSE_OPER_ADDR_POSI);
		reg |= ((efuse->data & EFUSE_OPER_WR_DATA_MASK) << EFUSE_OPER_WR_DATA_POSI);
		reg |= EFUSE_OPER_EN;
		REG_WRITE(SCTRL_EFUSE_CTRL, reg);

		do {
			reg = REG_READ(SCTRL_EFUSE_CTRL);
		} while (reg & EFUSE_OPER_EN);

		// disable vdd2.5v at last
		reg = REG_READ(SCTRL_CONTROL);
		reg &= ~EFUSE_VDD25_EN;
		REG_WRITE(SCTRL_CONTROL, reg);

		// check, so read
		reg = efuse->data;
		efuse->data = 0;
		if (sctrl_read_efuse(param) == 0) {
			if (((UINT8)reg) == efuse->data)
				ret = 0;
		}
	}

wr_exit:
	return ret;
}
#endif // (CFG_SOC_NAME != SOC_BK7231)

#if CFG_USE_USB_CHARGE
#if (CFG_SOC_NAME == SOC_BK7221U)
UINT32 usb_charge_oper_val(UINT32 elect)
{
    if(elect >= 450)
    {
        /*EXTERNAL CC elect*/
        if(elect > 750)
        {
            elect = 750;
        }
        return (elect - 450) / 20;
    }
    else
    {
        /*INTERNAL CC elect*/
        if(elect > 250)
        {
            elect = 250;
        }
        else if(elect < 20)
        {
            elect = 20;
        }

        if(elect >= 100 && elect <= 250)
        {
            return (elect - 100) / 10 + 16;
        }
        else
        {
            return (elect - 20) / 10;
        }
    }
}

UINT32 usb_power_is_pluged(void)
{
    UINT32 reg;
    reg = sctrl_analog_get(SCTRL_CHARGE_STATUS);
    return (reg & (1 << 21));
}

void charger_module_enable(UINT32 enable)
{
    sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4) & ~(1 << 12)) | (!!enable << 12));
}

void charger_vlcf_calibration(UINT32 type)
{

    if(type == 0)
    {
        /*Internal hardware calibration*/
        /*vlcf calibration*/
        /*>>> Added 5V voltage on Vusb*/
        /*>>> Added 4.2V voltage on vbattery*/
        /*>>> Set pwd=0*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 12));
        /*charge mode select*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
        /*calEn*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 21));
        /*softCalen*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 20));
        /*vlcfSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 18));
        /*IcalSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 17));
        /*vcvSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 16));
        /*vlcf_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 21));
        /*vlcf_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 21));
        /*Wait for at least 4 clock cycles*/
        delay_ms(1);
        /*vlcf_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 21));
        /*Waiting for 1ms, calibration finished*/
        delay_ms(1);
        /*Read the value vcal<5:0>, Recorded*/
        calib_charger[0] = (sctrl_analog_get(SCTRL_CHARGE_STATUS) >> CHARGE_VCAL_POS) & CHARGE_VCAL_MASK ;

    }
    else
    {
        /*External software calibration*/
        /*TODO*/
    }
}

void charger_icp_calibration(UINT32 type)
{

    if(type == 0)
    {
        /*Internal hardware calibration*/
        /*Icp calibration*/
        /*>>> Added parallel 60ohm resistor and 100nF capacitor from vbattery to ground.(Removed the external 4.2V)*/
        /*>>> Set pwd=0*/
        /*>>> Porb=0*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 12));
        /*Icp=60mA*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                                              & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (0x4 << CHARGE_LCP_POS));
        /*calEn*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 21));
        /*softCalen*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 20));
        /*vlcfSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 18));
        /*vcal<5:0>=previous vlcf calibration value*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                                              & ~(CHARGE_VCAL_MASK << 0)) | (calib_charger[0] << 0));
        /*IcalSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 17));
        /*vcvSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 16));
        /*Ical_trig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 20));
        /*Ical_trig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 20));
        /*Wait for at least 4 clock cycles*/
        delay_ms(1);
        /*Ical_trig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 20));
        /*Waiting for 1ms, calibration finished*/
        delay_ms(1);
        /*Read the value Ical<4:0>, Recorded*/
        calib_charger[1] = (sctrl_analog_get(SCTRL_CHARGE_STATUS) >> CHARGE_LCAL_POS) & CHARGE_LCAL_MASK ;

    }
    else
    {
        /*External software calibration*/
        /*TODO*/
    }
}

void charger_vcv_calibration(UINT32 type)
{

    if(type == 0)
    {
        /*Internal hardware calibration*/
        /*vcv calibration*/
        /*>>> Added 5V voltage on Vusb*/
        /*>>> Added 4.2V voltage on vbattery*/
        /*>>> Set pwd=0*/
        /*>>> Porb=0*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 12));
        /*charge mode select*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
        /*Icp=60mA*/
        /*sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
        *                                      & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (0x4 << CHARGE_LCP_POS));
        **/
        /*calEn*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 21));
        /*softCalen*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 20));
        /*vlcfSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 18));
        /*vcal<5:0>=previous vlcf calibration value*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                                              & ~(CHARGE_VCAL_MASK << 0)) | (calib_charger[0] << 0));
        /*IcalSel*/
        //sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 17));
        /*Ical<4:0>=previous Ical calibration value*/
        /*sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)\
        *                                     & ~(CHARGE_LCAL_MASK << 27)) | (calib_charger[1] << 27));
        */
        /*vcvSel*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 16));
        /*vcv_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 19));
        /*vcv_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 19));
        /*Wait for at least 4 clock cycles*/
        delay_ms(1);
        /*vcv_caltrig*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 19));
        /*Waiting for 1ms, calibration finished*/
        delay_ms(1);
        /*Read the value vcvcal<4:0>, Recorded*/
        calib_charger[2] = (sctrl_analog_get(SCTRL_CHARGE_STATUS) >> CHARGE_VCVCAL_POS) & CHARGE_VCVCAL_MASK ;

    }
    else
    {
        /*External software calibration*/
        /*TODO*/
    }
}

void charger_calib_get(UINT8 value[])
{
    value[0] = calib_charger[0];
    value[1] = calib_charger[1];
    value[2] = calib_charger[2];
    return;
}

void charger_calib_set(UINT8 value[])
{
    if(!value[0] || !value[1] || !value[2])
    {
        return;
    }

    calib_charger[0] = value[0];
    calib_charger[1] = value[1];
    calib_charger[2] = value[2];
    return;
}

UINT32 charger_is_full(void)
{
    UINT32 reg;
    reg = sctrl_analog_get(SCTRL_CHARGE_STATUS);
    return (reg & (1 << 20));
}

void charger_start(void *param)
{
    UINT32 charge_cal_type ;
    CHARGE_OPER_ST *chrg;

    chrg = (CHARGE_OPER_ST *)param;

    if(! usb_power_is_pluged())
    {
        os_printf("%s: not pluged\r\n", __FUNCTION__);
        return;
    }

    charger_calib_set(chrg->cal);
    os_printf("%s: %d %d %d %x %x %x\r\n", __FUNCTION__, chrg->type, chrg->step, chrg->elect,
              calib_charger[0], calib_charger[1], calib_charger[2]);
    if(chrg->step == STEP_START)
    {
        if(chrg->type == INTERNAL_HW_MODE || chrg->type == INTERNAL_SW_MODE)
        {
            /*Internal*/
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 21));

            if(chrg->type == INTERNAL_HW_MODE)
            {
                /*Internal ,hw control*/
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 19));
            }
            else if(chrg->type == INTERNAL_SW_MODE)
            {
                /*Internal ,sw control*/
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            }

            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 28));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 27));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 12));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 13)); //vcvcalEn_spilv
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 11));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 18));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 17));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 16));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                              & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (0x1f << CHARGE_LCP_POS));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                              & ~(CHARGE_VCAL_MASK << 0)) | (calib_charger[0] << 0));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)\
                              & ~(CHARGE_LCAL_MASK << 27)) | (calib_charger[1] << 27));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)\
                              & ~(CHARGE_LCAL_MASK << 22)) | (calib_charger[2] << 22));

            if(chrg->type == INTERNAL_HW_MODE)
            {
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                                  & ~(CHARGE_LC2CVDLYLV_MASK << CHARGE_LC2CVDLYLV_POS)) | (4 << CHARGE_LC2CVDLYLV_POS));
            }
            else if(chrg->type == INTERNAL_SW_MODE)
            {
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                                  & ~(CHARGE_LC2CVDLYLV_MASK << CHARGE_LC2CVDLYLV_POS)));
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 22));
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                                  & ~(CHARGE_VLCSWLV_MASK << CHARGE_VLCSWLV_POS)) | (8 << CHARGE_VLCSWLV_POS));
            }

        }
        else if(chrg->type == EXTERNAL_HW_MODE || chrg->type == EXTERNAL_SW_MODE)
        {
            /*External*/
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 21));

            if(chrg->type == EXTERNAL_HW_MODE)
            {
                /*External ,hw control*/
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 19));
            }
            else if(chrg->type == EXTERNAL_SW_MODE)
            {
                /*External ,sw control*/
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            }

            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 28));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 27));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 12));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 13)); //vcvcalEn_spilv

            if(chrg->type == EXTERNAL_HW_MODE)
            {
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 11));
            }
            else if(chrg->type == EXTERNAL_SW_MODE)
            {
                //sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 11));
                sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 11));
            }

            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 18));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 17));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 16));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                              & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (0x1f << CHARGE_LCP_POS));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                              & ~(CHARGE_VCAL_MASK << 0)) | (calib_charger[0] << 0));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)\
                              & ~(CHARGE_LCAL_MASK << 27)) | (calib_charger[1] << 27));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)\
                              & ~(CHARGE_LCAL_MASK << 22)) | (calib_charger[2] << 22));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 28));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                              & ~(CHARGE_LC2CVDLYLV_MASK << CHARGE_LC2CVDLYLV_POS)));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) & ~ (1 << 15));

            if(chrg->type == EXTERNAL_HW_MODE)
            {
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3) | (1 << 22));
                sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                                  & ~(CHARGE_VLCSWLV_MASK << CHARGE_VLCSWLV_POS)) | (0xf << CHARGE_VLCSWLV_POS));
            }
        }
    }
    else if(chrg->step == STEP_TRICKLE)
    {
        /*trickle charge*/
        if(chrg->type == INTERNAL_SW_MODE || chrg->type == EXTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)  \
                              & ~(CHARGE_MANMODE_MASK << CHARGE_MANMODE_POS)) | (4 << CHARGE_MANMODE_POS));
        }
    }
    else if(chrg->step == STEP_EXTER_CC)
    {
        /*External CC charge*/
        if(chrg->type == EXTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                              & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (usb_charge_oper_val(chrg->elect) << CHARGE_LCP_POS));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)  \
                              & ~(CHARGE_MANMODE_MASK << CHARGE_MANMODE_POS)) | (2 << CHARGE_MANMODE_POS));
        }
    }
    else if(chrg->step == STEP_INTER_CC)
    {
        /*Internal CC charge*/
        sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3)\
                          & ~(CHARGE_LCP_MASK << CHARGE_LCP_POS)) | (usb_charge_oper_val(chrg->elect) << CHARGE_LCP_POS));

        if(chrg->type == INTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)  \
                              & ~(CHARGE_MANMODE_MASK << CHARGE_MANMODE_POS)) | (2 << CHARGE_MANMODE_POS));
        }

        if(chrg->type == EXTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)  \
                              & ~(CHARGE_MANMODE_MASK << CHARGE_MANMODE_POS)) | (2 << CHARGE_MANMODE_POS));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) & ~ (1 << 15));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 11));
        }
    }
    else if(chrg->step == STEP_INTER_CV)
    {
        /*Internal CV charge*/
        if(chrg->type == INTERNAL_SW_MODE || chrg->type == EXTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) | (1 << 19));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, (sctrl_analog_get(SCTRL_ANALOG_CTRL4)  \
                              & ~(CHARGE_MANMODE_MASK << CHARGE_MANMODE_POS)) | (1 << CHARGE_MANMODE_POS));
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4) & ~ (1 << 15));
        }

        if( chrg->type == EXTERNAL_SW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 11));
        }

        if(chrg->type == EXTERNAL_HW_MODE)
        {
            sctrl_analog_set(SCTRL_ANALOG_CTRL4, sctrl_analog_get(SCTRL_ANALOG_CTRL4)& ~(1 << 11));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, sctrl_analog_get(SCTRL_ANALOG_CTRL3)& ~(1 << 22));
            sctrl_analog_set(SCTRL_ANALOG_CTRL3, (sctrl_analog_get(SCTRL_ANALOG_CTRL3) \
                              & ~(CHARGE_VLCSWLV_MASK << CHARGE_VLCSWLV_POS)));
        }
    }

}


void charger_stop(UINT32 type)
{
    os_printf("%s\r\n", __FUNCTION__);
    charger_module_enable(0);
}
#endif
#endif


UINT32 sctrl_ctrl(UINT32 cmd, void *param)
{
    UINT32 ret;
    UINT32 reg;
	GLOBAL_INT_DECLARATION();

    ret = SCTRL_SUCCESS;
	GLOBAL_INT_DISABLE();
    switch(cmd)
    {
#if PS_SUPPORT_MANUAL_SLEEP
    case CMD_SCTRL_NORMAL_SLEEP:
        reg = RF_HOLD_BY_STA_BIT;
        sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_CLR, &reg);
        sctrl_hw_sleep(*(UINT32 *)param);
        break;

    case CMD_SCTRL_NORMAL_WAKEUP:
        sctrl_hw_wakeup();
        delay(50);
        reg = RF_HOLD_BY_STA_BIT;
        sddev_control(SCTRL_DEV_NAME, CMD_RF_HOLD_BIT_SET, &reg);
        break;
#endif
#if CFG_USE_DEEP_PS
#if PS_SUPPORT_MANUAL_SLEEP
    case CMD_SCTRL_RTOS_IDLE_SLEEP:
        sctrl_enter_rtos_idle_sleep(*(PS_DEEP_CTRL_PARAM *)param);
        break;

    case CMD_SCTRL_RTOS_IDLE_WAKEUP:
        sctrl_exit_rtos_idle_sleep();
        break;
#endif

    case CMD_SCTRL_RTOS_DEEP_SLEEP:
        sctrl_enter_rtos_deep_sleep((PS_DEEP_CTRL_PARAM *)param);
        break;
#endif

    case CMD_GET_CHIP_ID:
        ret = REG_READ(SCTRL_CHIP_ID);
        break;

    case CMD_SCTRL_SET_FLASH_DPLL:
        reg = REG_READ(SCTRL_CONTROL);
        reg |= FLASH_26M_MUX_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_SCTRL_SET_FLASH_DCO:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~FLASH_26M_MUX_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_SCTRL_DSP_POWERDOWN:
    case CMD_SCTRL_USB_POWERDOWN:
    case CMD_SCTRL_MODEM_POWERDOWN:
    case CMD_SCTRL_MAC_POWERDOWN:
    case CMD_SCTRL_DSP_POWERUP:
    case CMD_SCTRL_USB_POWERUP:
    case CMD_SCTRL_MAC_POWERUP:
    case CMD_SCTRL_MODEM_POWERUP:
    case CMD_SCTRL_BLE_POWERDOWN:
    case CMD_SCTRL_BLE_POWERUP:
        sctrl_subsys_power(cmd);
        break;

    case CMD_GET_DEVICE_ID:
        ret = REG_READ(SCTRL_DEVICE_ID);
        break;

    case CMD_GET_SCTRL_CONTROL:
        *((UINT32 *)param) = REG_READ(SCTRL_CONTROL);
        break;

    case CMD_SET_SCTRL_CONTROL:
        REG_WRITE(SCTRL_CONTROL, *((UINT32 *)param));
        break;

    case CMD_SCTRL_MCLK_SELECT:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(MCLK_MUX_MASK << MCLK_MUX_POSI);
        reg |= ((*(UINT32 *)param) & MCLK_MUX_MASK) << MCLK_MUX_POSI;
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_SCTRL_MCLK_DIVISION:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
        reg |= ((*(UINT32 *)param) & MCLK_DIV_MASK) << MCLK_DIV_POSI;
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_SCTRL_MCLK_MUX_GET:
        reg = ((REG_READ(SCTRL_CONTROL) >> MCLK_MUX_POSI) & MCLK_MUX_MASK);
        *(UINT32 *)param = reg;
        break;

    case CMD_SCTRL_MCLK_DIV_GET:
        reg = ((REG_READ(SCTRL_CONTROL) >> MCLK_DIV_POSI) & MCLK_DIV_MASK);
        *(UINT32 *)param = reg;
        break;

    case CMD_SCTRL_RESET_SET:
        reg = REG_READ(SCTRL_RESET);
        reg |= ((*(UINT32 *)param) & SCTRL_RESET_MASK);
        REG_WRITE(SCTRL_RESET, reg);
        break;

    case CMD_SCTRL_RESET_CLR:
        reg = REG_READ(SCTRL_RESET);
        reg &= ~((*(UINT32 *)param) & SCTRL_RESET_MASK);
        REG_WRITE(SCTRL_RESET, reg);
        break;

    case CMD_SCTRL_MODEM_SUBCHIP_RESET:
    case CMD_SCTRL_MAC_SUBSYS_RESET:
    case CMD_SCTRL_USB_SUBSYS_RESET:
    case CMD_SCTRL_DSP_SUBSYS_RESET:
        sctrl_subsys_reset(cmd);
        break;

    case CMD_SCTRL_MODEM_CORE_RESET:
        ret = REG_READ(SCTRL_MODEM_CORE_RESET_PHY_HCLK);
        ret = ret & (~((MODEM_CORE_RESET_MASK) << MODEM_CORE_RESET_POSI));
        reg = ret | ((MODEM_CORE_RESET_WORD & MODEM_CORE_RESET_MASK)
                     << MODEM_CORE_RESET_POSI);
        REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

        delay(1);
        reg = ret;
        REG_WRITE(SCTRL_MODEM_CORE_RESET_PHY_HCLK, reg);

        /*resetting, and waiting for done*/
        reg = REG_READ(SCTRL_RESET);
        while(reg & MODEM_CORE_RESET_BIT)
        {
            delay(10);
            reg = REG_READ(SCTRL_RESET);
        }
#if (CFG_SOC_NAME == SOC_BK7238)
        mpb_regs_reset();
#endif
        ret = SCTRL_SUCCESS;
        break;

    case CMD_SCTRL_MPIF_CLK_INVERT:
#if (CFG_SOC_NAME != SOC_BK7238)
        reg = REG_READ(SCTRL_CONTROL);
        reg |= MPIF_CLK_INVERT_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);
#endif // (CFG_SOC_NAME != SOC_BK7238)
        break;

    case CMD_SCTRL_BLK_ENABLE:
        reg = REG_READ(SCTRL_BLOCK_EN_CFG);
        reg &= (~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI));
        reg |= (BLOCK_EN_WORD_PWD & BLOCK_EN_WORD_MASK) << BLOCK_EN_WORD_POSI;
        reg |= ((*(UINT32 *)param) & BLOCK_EN_VALID_MASK);
        REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
        break;

    case CMD_SCTRL_BLK_DISABLE:
        reg = REG_READ(SCTRL_BLOCK_EN_CFG);
		reg &= (~(BLOCK_EN_WORD_MASK << BLOCK_EN_WORD_POSI));
		reg |= (BLOCK_EN_WORD_PWD & BLOCK_EN_WORD_MASK) << BLOCK_EN_WORD_POSI;
        reg &= ~((*(UINT32 *)param) & BLOCK_EN_VALID_MASK);
        REG_WRITE(SCTRL_BLOCK_EN_CFG, reg);
        break;

    case CMD_SCTRL_BIAS_REG_SET:
        reg = REG_READ(SCTRL_BIAS);
        reg |= (*(UINT32 *)param);
        REG_WRITE(SCTRL_BIAS, reg);
        break;

    case CMD_SCTRL_BIAS_REG_CLEAN:
        reg = REG_READ(SCTRL_BIAS);
        reg &= ~(*(UINT32 *)param);
        REG_WRITE(SCTRL_BIAS, reg);
        break;

    case CMD_SCTRL_BIAS_REG_READ:
        ret = REG_READ(SCTRL_BIAS);
        break;

    case CMD_SCTRL_BIAS_REG_WRITE:
        REG_WRITE(SCTRL_BIAS, *(UINT32 *)param);
        break;

    case CMD_SCTRL_ANALOG_CTRL4_SET:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL4);
        reg |= (*(UINT32 *)param);
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, reg);
        break;

    case CMD_SCTRL_ANALOG_CTRL4_CLEAN:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL4);
        reg &= ~(*(UINT32 *)param);
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, reg);
        break;

    case CMD_SCTRL_CALI_DPLL:
        sctrl_cali_dpll(0);
        break;

#if (CFG_SOC_NAME != SOC_BK7231)
    case CMD_SCTRL_SET_XTALH_CTUNE:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
        reg &= ~(XTALH_CTUNE_MASK<< XTALH_CTUNE_POSI);
        reg |= (((*(UINT32 *)param) &XTALH_CTUNE_MASK) << XTALH_CTUNE_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL2, reg);
        break;

    case CMD_SCTRL_GET_XTALH_CTUNE:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
        ret = ((reg >> XTALH_CTUNE_POSI) & XTALH_CTUNE_MASK);
        break;

    case CMD_BLE_RF_BIT_SET:
        reg = REG_READ(SCTRL_CONTROL);
        reg |= BLE_RF_EN_BIT;
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_BLE_RF_BIT_CLR:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(BLE_RF_EN_BIT);
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_RF_HOLD_BIT_SET:
        rf_hold_status |= (*(UINT32 *)param);
        //os_printf("s:%x %x\r\n",rf_hold_status,(*(UINT32 *)param));
        if(rf_hold_status)
        {
            sctrl_rf_wakeup();
        }
        break;

    case CMD_RF_HOLD_BIT_CLR:
        rf_hold_status &= ~(*(UINT32 *)param);
        //os_printf("c:%x %x\r\n",rf_hold_status,(*(UINT32 *)param));
        if(0 == rf_hold_status)
        {
            sctrl_rf_sleep();
        }
        break;

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
	case CMD_BLE_RF_PTA_EN:
        reg = REG_READ(SCTRL_CONTROL);
        reg |= (BLE_RF_PTA_EN_BIT);
        REG_WRITE(SCTRL_CONTROL, reg);
        break;
	case CMD_BLE_RF_PTA_DIS:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(BLE_RF_PTA_EN_BIT);
        REG_WRITE(SCTRL_CONTROL, reg);
        break;
	case CMD_BLE_RF_PTA_GET:
	reg = REG_READ(SCTRL_CONTROL);
	*((UINT32 *)param) = reg & (BLE_RF_PTA_EN_BIT);
	break;
#endif

	case CMD_BLE_RF_BIT_GET:
		reg = REG_READ(SCTRL_CONTROL);
		*((UINT32 *)param) = reg & (BLE_RF_EN_BIT);
		break;

    case CMD_EFUSE_WRITE_BYTE:
        ret = sctrl_write_efuse(param);
        break;

    case CMD_EFUSE_READ_BYTE:
        ret = sctrl_read_efuse(param);
        break;

#if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7238)
    case CMD_QSPI_VDDRAM_VOLTAGE:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(PSRAM_VDDPAD_VOLT_MASK << PSRAM_VDDPAD_VOLT_POSI);
        reg |=(((*(UINT32 *)param) & PSRAM_VDDPAD_VOLT_MASK) << PSRAM_VDDPAD_VOLT_POSI);
        REG_WRITE(SCTRL_CONTROL, reg);
        break;

    case CMD_QSPI_IO_VOLTAGE:
        reg = REG_READ(SCTRL_CONTROL);
        reg &= ~(QSPI_IO_VOLT_MASK << QSPI_IO_VOLT_POSI);
        reg |=(((*(UINT32 *)param) & QSPI_IO_VOLT_MASK) << QSPI_IO_VOLT_POSI);
        REG_WRITE(SCTRL_CONTROL, reg);
        break;
#endif

#endif // (CFG_SOC_NAME != SOC_BK7231)

#if (CFG_SOC_NAME == SOC_BK7221U)
    case CMD_SCTRL_OPEN_DAC_ANALOG:
        //cause increase CFG_USE_STA_PS elect
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL9);
        reg |= EN_AUD_DAC_L | EN_AUD_DAC_R
              | DAC_PA_OUTPUT_EN | DAC_DRIVER_OUTPUT_EN
              | AUD_DAC_DGA_EN;
        sctrl_analog_set(SCTRL_ANALOG_CTRL9, reg);

        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL10);
        reg |= DAC_N_END_OUPT_L | DAC_N_END_OUPT_R;
        sctrl_analog_set(SCTRL_ANALOG_CTRL10, reg);
        break;

    case CMD_SCTRL_CLOSE_DAC_ANALOG:
        //cause reduce CFG_USE_STA_PS elect
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL9);
        reg &= ~(EN_AUD_DAC_L | EN_AUD_DAC_R
              | DAC_PA_OUTPUT_EN | DAC_DRIVER_OUTPUT_EN
              | AUD_DAC_DGA_EN);
        sctrl_analog_set(SCTRL_ANALOG_CTRL9, reg);

        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL10);
        reg &= ~(DAC_N_END_OUPT_L | DAC_N_END_OUPT_R);
        sctrl_analog_set(SCTRL_ANALOG_CTRL10, reg);
        break;

    case CMD_SCTRL_OPEN_ADC_MIC_ANALOG:
        //cause increase CFG_USE_STA_PS elect
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg &= ~(SPI_PWD_AUD_ADC_L | SPI_PWD_AUD_ADC_R);
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_CLOSE_ADC_MIC_ANALOG:
        //cause reduce CFG_USE_STA_PS elect
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg |= (SPI_PWD_AUD_ADC_L | SPI_PWD_AUD_ADC_R);
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_ENALBLE_ADC_LINE_IN:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg |= LINE_IN_EN;
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_DISALBLE_ADC_LINE_IN:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg &= ~LINE_IN_EN;
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_SET_DAC_VOLUME_ANALOG:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg &= ~(AUD_DAC_GAIN_MASK << AUD_DAC_GAIN_POSI);
        reg |= (((*(UINT32 *)param) & AUD_DAC_GAIN_MASK) << AUD_DAC_GAIN_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_SET_LINEIN_VOLUME_ANALOG:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        reg &= ~(LINE_IN_GAIN_MASK << LINE_IN_GAIN_POSI);
        reg |= (((*(UINT32 *)param) & LINE_IN_GAIN_MASK) << LINE_IN_GAIN_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        break;

    case CMD_SCTRL_SET_VOLUME_PORT:
        if((*(UINT32 *)param) == AUDIO_DAC_VOL_DIFF_MODE)
        {
            reg = sctrl_analog_get(SCTRL_ANALOG_CTRL9);
            reg |= (DAC_DIFF_EN);
            sctrl_analog_set(SCTRL_ANALOG_CTRL9, reg);

            reg = sctrl_analog_get(SCTRL_ANALOG_CTRL10);
            reg |= (DAC_N_END_OUPT_L | DAC_N_END_OUPT_R);
            reg &= ~(DAC_VSEL_MASK << DAC_VSEL_POSI);
            reg |= ((0x3 & DAC_VSEL_MASK) << DAC_VSEL_POSI);;
            sctrl_analog_set(SCTRL_ANALOG_CTRL10, reg);
        }
        else if((*(UINT32 *)param) == AUDIO_DAC_VOL_SINGLE_MODE)
        {
            reg = sctrl_analog_get(SCTRL_ANALOG_CTRL9);
            reg &= ~(DAC_DIFF_EN);
            sctrl_analog_set(SCTRL_ANALOG_CTRL9, reg);

            reg = sctrl_analog_get(SCTRL_ANALOG_CTRL10);
            reg &= ~(DAC_N_END_OUPT_L | DAC_N_END_OUPT_R);
            reg &= ~(DAC_VSEL_MASK << DAC_VSEL_POSI);
            reg |= ((0 & DAC_VSEL_MASK) << DAC_VSEL_POSI);;
            sctrl_analog_set(SCTRL_ANALOG_CTRL10, reg);
        }
        break;

    case CMD_SCTRL_SET_AUD_DAC_MUTE:
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
        if((*(UINT32 *)param) == AUDIO_DAC_ANALOG_MUTE)
        {
            reg |= (AUD_DAC_MUTE_EN);
            sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        }
        else if((*(UINT32 *)param) == AUDIO_DAC_ANALOG_UNMUTE)
        {
            reg &= ~(AUD_DAC_MUTE_EN);
            sctrl_analog_set(SCTRL_ANALOG_CTRL8, reg);
        }
        break;
#endif
#if (CFG_SOC_NAME != SOC_BK7231)
	case CMD_SCTRL_SET_ANALOG6:
		reg = sctrl_analog_get(SCTRL_ANALOG_CTRL6);
		reg |= (DPLL_CLK_FOR_AUDIO_EN | DPLL_DIVIDER_CLK_SEL | DPLL_RESET );
		sctrl_analog_set(SCTRL_ANALOG_CTRL6, reg);
		break;
#endif
	case CMD_SCTRL_SET_ANALOG0:
		sctrl_analog_set(SCTRL_ANALOG_CTRL0, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG1:
		sctrl_analog_set(SCTRL_ANALOG_CTRL1, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG2:
		sctrl_analog_set(SCTRL_ANALOG_CTRL2, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG3:
		sctrl_analog_set(SCTRL_ANALOG_CTRL3, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG4:
		sctrl_analog_set(SCTRL_ANALOG_CTRL4, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG5:
		sctrl_analog_set(SCTRL_ANALOG_CTRL5, (*(UINT32 *)param));
		break;
#if (CFG_SOC_NAME == SOC_BK7221U)
	case CMD_SCTRL_SET_ANALOG7:
		sctrl_analog_set(SCTRL_ANALOG_CTRL7, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG8:
		sctrl_analog_set(SCTRL_ANALOG_CTRL8, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG9:
		sctrl_analog_set(SCTRL_ANALOG_CTRL9, (*(UINT32 *)param));
		break;
	case CMD_SCTRL_SET_ANALOG10:
		sctrl_analog_set(SCTRL_ANALOG_CTRL10, (*(UINT32 *)param));
		break;
#endif // (CFG_SOC_NAME == SOC_BK7221U)
	case CMD_SCTRL_GET_ANALOG0:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL0);
		break;
	case CMD_SCTRL_GET_ANALOG1:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL1);
		break;
	case CMD_SCTRL_GET_ANALOG2:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL2);
		break;
	case CMD_SCTRL_GET_ANALOG3:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL3);
		break;
	case CMD_SCTRL_GET_ANALOG4:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL4);
		break;
	case CMD_SCTRL_GET_ANALOG5:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL5);
		break;
#if (CFG_SOC_NAME == SOC_BK7221U)
	case CMD_SCTRL_GET_ANALOG7:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL7);
		break;
	case CMD_SCTRL_GET_ANALOG8:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL8);
		break;
	case CMD_SCTRL_GET_ANALOG9:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL9);
		break;
	case CMD_SCTRL_GET_ANALOG10:
		ret = sctrl_analog_get(SCTRL_ANALOG_CTRL10);
		break;

	case CMD_SCTRL_AUDIO_PLL:
		if((*(UINT32 *)param) == 48000000)						//48MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3B13B13B);
		}
		else if((*(UINT32 *)param) == 48128000)					//48.128MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3B3C05AC);
		}
		else if((*(UINT32 *)param) == 48384000)					//48.384MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3B8CAE8E);
		}
		else if((*(UINT32 *)param) == 49152000)					//49.152MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3C7EA932);
		}
		else if((*(UINT32 *)param) == 49392000)					//49.392MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3CCA4785);
		}
		else if((*(UINT32 *)param) == 50688000)					//50.688MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3E629E7C);
		}
		else if((*(UINT32 *)param) == 50803200)					//50.8032MHz
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3E86EA7A);
		}
		else
		{
			sctrl_analog_set(SCTRL_ANALOG_CTRL5,0x3B13B13B);
		}
		break;

#if CFG_USE_USB_CHARGE
    case CMD_SCTRL_USB_CHARGE_CAL:
        if(1 == ((CHARGE_OPER_PTR)param)->step)
            charger_vlcf_calibration(0);
        else if(2 == ((CHARGE_OPER_PTR)param)->step)
            charger_icp_calibration(0);
        else if(3 == ((CHARGE_OPER_PTR)param)->step)
            charger_vcv_calibration(0);
        else if(4 == ((CHARGE_OPER_PTR)param)->step)
            charger_calib_get(((CHARGE_OPER_PTR)param)->cal);
        break;
    case CMD_SCTRL_USB_CHARGE_START:
        charger_start(param);
        break;
    case CMD_SCTRL_USB_CHARGE_STOP:
        charger_stop((*(UINT32 *)param));
        break;
#endif

#endif // (CFG_SOC_NAME == SOC_BK7221U)
    case CMD_SCTRL_SET_LOW_PWR_CLK:
        reg = REG_READ(SCTRL_LOW_PWR_CLK);
        reg &=~(LPO_CLK_MUX_MASK);
        reg |=((*(UINT32 *)param) << LPO_CLK_MUX_POSI);
        REG_WRITE(SCTRL_LOW_PWR_CLK, reg);
        break;
    case CMD_SCTRL_SET_GADC_SEL:
        #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        reg = sctrl_analog_get(SCTRL_ANALOG_CTRL4);
        reg &= ~(GADC_CAL_SEL_MASK << GADC_CAL_SEL_POSI);
        reg |= (((*(UINT32 *)param) & GADC_CAL_SEL_MASK) << GADC_CAL_SEL_POSI);
        sctrl_analog_set(SCTRL_ANALOG_CTRL4, reg);
        #endif
        break;
    case CMD_SCTRL_SET_VDD_VALUE:
    	reg = REG_READ(SCTRL_DIGTAL_VDD);
    	reg &= (~(DIG_VDD_ACTIVE_MASK << DIG_VDD_ACTIVE_POSI));
        reg |=((*(UINT32 *)param) << DIG_VDD_ACTIVE_POSI);
#if (CFG_SOC_NAME == SOC_BK7238)
		reg &= (~(AON_VDD_ACTIVE_MASK << AON_VDD_ACTIVE_POSI));
		reg |=((*(UINT32 *)param) << AON_VDD_ACTIVE_POSI);
#endif
    	REG_WRITE(SCTRL_DIGTAL_VDD, reg);
        break;
    case CMD_SCTRL_GET_VDD_VALUE:
        reg = REG_READ(SCTRL_DIGTAL_VDD);
        ret = (reg >> DIG_VDD_ACTIVE_POSI) & DIG_VDD_ACTIVE_MASK;
        break;

#if (CFG_SOC_NAME != SOC_BK7231)
    case CMD_GET_SCTRL_RETETION:
        *((UINT32 *)param) = REG_READ(SCTRL_SW_RETENTION);
        break;

    case CMD_SET_SCTRL_RETETION:
        REG_WRITE(SCTRL_SW_RETENTION, *((UINT32 *)param));
        break;
#endif

#if (CFG_SOC_NAME == SOC_BK7221U)
    case CMD_SCTRL_UNCONDITIONAL_MAC_DOWN:
	    sctrl_unconditional_mac_sleep();
        break;

    case CMD_SCTRL_UNCONDITIONAL_MAC_UP:
        sctrl_unconditional_mac_wakeup();
        break;

    case CMD_SCTRL_UNCONDITIONAL_RF_DOWN:
    	sctrl_rf_sleep();
        break;

    case CMD_SCTRL_UNCONDITIONAL_RF_UP:
        sctrl_rf_wakeup();
        break;
#elif (CFG_SOC_NAME == SOC_BK7238)
    case CMD_SCTRL_SKIP_BOOT:
		reg = REG_READ(SCTRL_SLEEP);
		if (*((UINT32 *)param)) {
			reg |= BOOT_SKIP_BIT;
		} else {
			reg &= ~BOOT_SKIP_BIT;
		}
		REG_WRITE(SCTRL_SLEEP, reg);
        break;
#endif
    default:
        ret = SCTRL_FAILURE;
        break;
    }
	GLOBAL_INT_RESTORE();

    return ret;
}

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7231U)
extern void flash_set_clk(UINT8 clk_conf);
void sctrl_overclock(int enable)
{
	UINT32 param;

	GLOBAL_INT_DECLARATION();

	if (enable) {
		GLOBAL_INT_DISABLE();
		 /* already enabled */
		if (sctrl_overclock_refcnt++)
			goto out;

		param = REG_READ(SCTRL_CONTROL);
		param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
		param |= ((MCLK_DIV_2 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
		REG_WRITE(SCTRL_CONTROL, param);
		flash_set_clk(8);
	} else {
		GLOBAL_INT_DISABLE();
		if (sctrl_overclock_refcnt)
			--sctrl_overclock_refcnt;

		/* other API still hold overclock */
		if (sctrl_overclock_refcnt)
			goto out;

		/*config main clk*/
#if !USE_DCO_CLK_POWON
		param = REG_READ(SCTRL_CONTROL);
		param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
#if CFG_SYS_REDUCE_NORMAL_POWER
		param |= ((MCLK_DIV_7 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
#else
		param |= ((MCLK_DIV_5 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
#endif // CFG_SYS_REDUCE_NORMAL_POWER
		REG_WRITE(SCTRL_CONTROL, param);
#endif /*(!USE_DCO_CLK_POWON)*/

#if CFG_USE_STA_PS
		flash_set_clk(9);
#else
		flash_set_clk(5);
#endif
	}
out:
	GLOBAL_INT_RESTORE();
}
#elif (CFG_SOC_NAME == SOC_BK7238)
extern void flash_set_clk(UINT8 clk_conf);
/* save current MCLK before overclock */
static UINT32 sctrl_overclock_div_backup = -1;

void sctrl_overclock(int enable)
{
	UINT32 param;

	GLOBAL_INT_DECLARATION();
	if (enable) {
		GLOBAL_INT_DISABLE();
		 /* already enabled */
		if (sctrl_overclock_refcnt++)
			goto out;

		/* read sys_ctrl register */
		param = REG_READ(SCTRL_CONTROL);
		/* backup previous mclk div */
		sctrl_overclock_div_backup = (param >> MCLK_DIV_POSI) & MCLK_DIV_MASK;

		/* set new mclk div: 480M/(2+1)=160M */
		param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
		param |= ((MCLK_DIV_2 & MCLK_DIV_MASK) << MCLK_DIV_POSI);
		REG_WRITE(SCTRL_CONTROL, param);

		flash_set_clk(8);
		sctrl_ctrl(CMD_BLE_RF_PTA_DIS, NULL);
	} else {
		/*config main clk*/
		GLOBAL_INT_DISABLE();
		if (sctrl_overclock_refcnt)
			--sctrl_overclock_refcnt;

		/* other API still hold overclock */
		if (sctrl_overclock_refcnt)
			goto out;

		/* restore previous mclk */
		if (sctrl_overclock_div_backup != -1) {
			param = REG_READ(SCTRL_CONTROL);
			param &= ~(MCLK_DIV_MASK << MCLK_DIV_POSI);
			param |= ((sctrl_overclock_div_backup & MCLK_DIV_MASK) << MCLK_DIV_POSI);
			REG_WRITE(SCTRL_CONTROL, param);
		}

#if CFG_USE_STA_PS
		flash_set_clk(9);
#else
		flash_set_clk(5);
#endif
		sctrl_ctrl(CMD_BLE_RF_PTA_EN, NULL);
	}
out:
	GLOBAL_INT_RESTORE();
}
#elif (CFG_SOC_NAME == SOC_BK7221U)
void sctrl_overclock(int enable)
{

	GLOBAL_INT_DECLARATION();
	if (enable) {
		GLOBAL_INT_DISABLE();
		 /* already enabled */
		if (sctrl_overclock_refcnt++)
			goto out;

		if(mcu_ps_is_on())
			sctrl_mcu_exit();

	} else {
		/*config main clk*/
		GLOBAL_INT_DISABLE();
		if (sctrl_overclock_refcnt)
			--sctrl_overclock_refcnt;

		/* other API still hold overclock */
		if (sctrl_overclock_refcnt)
			goto out;

		if(mcu_ps_is_on())
			sctrl_mcu_init();
	}
out:
	GLOBAL_INT_RESTORE();
}
#else
void sctrl_overclock(int enable)
{
}
#endif
// EOF
