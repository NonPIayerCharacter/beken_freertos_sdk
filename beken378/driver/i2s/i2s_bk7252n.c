#include "include.h"
#include "arm_arch.h"
#include "typedef.h"

#include "i2s.h"
#include "i2s_pub.h"
#include "gpio_pub.h"
#include "drv_model_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "driver_audio_if_pub.h"
#include "uart_pub.h"
#include "music_msg_pub.h"
#include "sys_config.h"
#include "sys_ctrl.h"
#include "error.h"
#include "rtos_pub.h"
#include "mem_pub.h"
#include "target_util_pub.h"
#include "sys_ctrl_pub.h"
#include "sys_ctrl.h"
#include "icu.h"
#include "gpio.h"
#include "generic.h"

#if CFG_USE_I2S

#define RT_I2S_BIT_DEBUG
#ifdef  RT_I2S_BIT_DEBUG
#define bit_dbg(fmt, ...)   bk_printf(fmt, ##__VA_ARGS__)
#else
#define bit_dbg(fmt, ...)
#endif

#ifndef MAX
#define MAX(x, y) ((x >= y) ? x : y)
#endif

volatile i2s_trans_t i2s_trans;
i2s_level_t i2s_fifo_level;

static UINT32 i2s_ctrl(UINT32 cmd, void *param);
void intc_service_change_handler(UINT8 int_num, FUNCPTR isr);
extern void delay_us(UINT32 ms_count);

static SDD_OPERATIONS i2s_op =
{
    i2s_ctrl
};

static void i2s_active(UINT8 enable)
{
    UINT32 value_ctrl;

    value_ctrl = REG_READ(PCM_CTRL);

    if(enable)
    {
        value_ctrl |= I2S_PCM_EN;
    }
    else
    {
        value_ctrl &= ~(I2S_PCM_EN);
    }

    REG_WRITE(PCM_CTRL, value_ctrl);
}

static void i2s_set_msten(UINT8 enable)
{
    UINT32 value_ctrl;

    value_ctrl = REG_READ(PCM_CTRL);

    if(enable)
    {
        value_ctrl |= MSTEN;
    }
    else
    {
        value_ctrl &= ~(MSTEN);
    }

    REG_WRITE(PCM_CTRL, value_ctrl);
}

static void i2s_set_select_mode(UINT8 val)
{
    UINT32 value;
    val = (val & MODE_SEL_MASK);

    if(val == 3 || val > 7)
    {
        return;
    }

    value = REG_READ(PCM_CTRL);
    value &= ~(MODE_SEL_MASK << MODE_SEL_POSI);
    value |= (val << MODE_SEL_POSI);
    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_lrck(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);
    if(val)
    {
        value |= LRCK_RP;
    }
    else
    {
        value &= ~(LRCK_RP);
    }
    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_sck_inv(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);

    if(val)
    {
        value |= SCK_INV;
    }
    else
    {
        value &= ~(SCK_INV);
    }

    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_lsb_first(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);
    if(val)
    {
        value |= LSB_FIRST;
    }
    else
    {
        value &= ~(LSB_FIRST);
    }
    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_sck_synclen(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);
    value &= ~(SYNCLEN_MASK);
    value |= ((val << SYNCLEN_POSI) & SYNCLEN_MASK);
    REG_WRITE(PCM_CTRL, value);
}

__maybe_unused static void i2s_set_data_length(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);
    value &= ~(DATALEN_MASK << DATALEN_POSI);
    value |= ((val & DATALEN_MASK) << DATALEN_POSI);
    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_pcm_dlen(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CTRL);
    value &= ~(PCM_DLEN_MASK << PCM_DLEN_POSI);
    value |= ((val & PCM_DLEN_MASK) << PCM_DLEN_POSI);
    REG_WRITE(PCM_CTRL, value);
}

static void i2s_set_freq_datawidth(i2s_rate_t *p_rate)
{
    UINT32 bitratio, smpratio, datalen;
    UINT32 value = 0, sys_clk= 0;

    if( (p_rate->freq != 8000) && (p_rate->freq != 16000) &&
        (p_rate->freq != 24000) && (p_rate->freq != 32000) &&(p_rate->freq != 48000)&&
        (p_rate->freq != 11025) && (p_rate->freq != 22050) &&(p_rate->freq != 44100) )
    {
        return;
    }

    /* set lrck div */
    if(p_rate->datawidth == 8)
    {
        datalen = 7;
    }
    else if(p_rate->datawidth == 16)
    {
        datalen = 15;
    }
    else if(p_rate->datawidth == 24)
    {
        datalen = 23;
    }
    else
    {
        datalen = 31;
    }

    /* set system clock */
    if(p_rate->freq == 8000)
    {
        if(p_rate->datawidth == 24)
        {
            sys_clk = 48384000;
        }
        else
        {
            sys_clk = 48128000;
        }
    }
    else if (p_rate->freq == 16000)
    {
        if((p_rate->datawidth == 16) || (p_rate->datawidth == 8))
        {
            sys_clk = 48128000;
        }
        else
        {
            sys_clk = 49152000;
        }
    }
    else if(p_rate->freq == 44100)
    {
        sys_clk = 50803200;
    }
    else
    {
        if(p_rate->datawidth == 24)
        {
            sys_clk = 50688000;
        }
        else
        {
            sys_clk = 49152000;
        }
    }

    /* set bit clock divd */
    /* Fs=Fsck/(2*(SMPRADIO+1)), Fsck=Fsys_clk/(2*BITRADIO) */
    /* bit[7:0]_BITRADIO is unused in slave mode */
    bitratio = MAX(NUMBER_ROUND_DOWN((sys_clk / 2 ), (p_rate->freq * 2 * p_rate->datawidth)), 5);
    smpratio = datalen;
    value = REG_READ(PCM_CTRL);
    value = value & ~(DATALEN_MASK << DATALEN_POSI)
                  & ~(SMPRATIO_MASK << SMPRATIO_POSI)
                  & ~(BITRATIO_MASK << BITRATIO_POSI);
    value = value | ((datalen & DATALEN_MASK) << DATALEN_POSI)
                  | ((smpratio & SMPRATIO_MASK) << SMPRATIO_POSI)
                  | ((bitratio & BITRATIO_MASK) << BITRATIO_POSI);
    REG_WRITE(PCM_CTRL, value);

    bitratio = (bitratio >> 8);
    smpratio = (smpratio >> 5);
    value = REG_READ(PCM_CN);
    value = value & ~(SMPRATIO_H2B_MASK << SMPRATIO_H2B_POSI)
                  & ~(SMPRATIO_H4B_MASK << SMPRATIO_H4B_POSI);
    value = value | ((smpratio & SMPRATIO_H2B_MASK) << SMPRATIO_H2B_POSI)
                  | ((bitratio & SMPRATIO_H4B_MASK) << SMPRATIO_H4B_POSI);
    REG_WRITE(PCM_CN, value);
}

static void i2s_set_rxint_enable(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= RX_INT_EN;
    }
    else
    {
        value &= ~(RX_INT_EN);
    }
    REG_WRITE(PCM_CN, value);
}

/* index=[2,3,4] */
__maybe_unused static void i2s_set_rxint_x_enable(UINT8 val, UINT8 index)
{
    UINT32 value;

    if ((index == 1) || (index > 4))
    {
        return;
    }

    value = REG_READ(PCM_CN_LT2);
    if(val)
    {
        value |= PCM_CN_LT2_RX_N_INT_EN(index);
    }
    else
    {
        value &= ~(PCM_CN_LT2_RX_N_INT_EN(index));
    }
    REG_WRITE(PCM_CN_LT2, value);
}

static void i2s_set_txint_enable(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= TX_INT_EN;
    }
    else
    {
        value &= ~(TX_INT_EN);
    }
    REG_WRITE(PCM_CN, value);
}

/* index=[2,3,4] */
__maybe_unused static void i2s_set_txint_x_enable(UINT8 val, UINT8 index)
{
    UINT32 value;

    if ((index == 1) || (index > 4))
    {
        return;
    }

    value = REG_READ(PCM_CN_LT2);
    if(val)
    {
        value |= PCM_CN_LT2_TX_N_INT_EN(index);
    }
    else
    {
        value &= ~(PCM_CN_LT2_TX_N_INT_EN(index));
    }
    REG_WRITE(PCM_CN_LT2, value);
}

static void i2s_set_rxovr_enable(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= RX_OVF_EN;
    }
    else
    {
        value &= ~(RX_OVF_EN);
    }
    REG_WRITE(PCM_CN, value);
}

/* index=[2,3,4] */
__maybe_unused static void i2s_set_rxovr_x_enable(UINT8 val, UINT8 index)
{
    UINT32 value;

    if ((index == 1) || (index > 4))
    {
        return;
    }

    value = REG_READ(PCM_CN_LT2);
    if(val)
    {
        value |= PCM_CN_LT2_RX_N_OVF_EN(index);
    }
    else
    {
        value &= ~(PCM_CN_LT2_RX_N_OVF_EN(index));
    }
    REG_WRITE(PCM_CN_LT2, value);
}

static void i2s_set_txovr_enable(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= TX_UDF_EN;
    }
    else
    {
        value &= ~(TX_UDF_EN);
    }
    REG_WRITE(PCM_CN, value);
}

/* index=[2,3,4] */
__maybe_unused static void i2s_set_txovr_x_enable(UINT8 val, UINT8 index)
{
    UINT32 value;

    if ((index == 1) || (index > 4))
    {
        return;
    }

    value = REG_READ(PCM_CN_LT2);
    if(val)
    {
        value |= PCM_CN_LT2_TX_N_UDF_EN(index);
    }
    else
    {
        value &= ~(PCM_CN_LT2_TX_N_UDF_EN(index));
    }
    REG_WRITE(PCM_CN_LT2, value);
}

static void i2s_set_rxint_level(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);

    value &= ~(RX_FIFO_LEVEL_MASK << RX_FIFO_LEVEL_POSI) ;
    value |= ((val & RX_FIFO_LEVEL_MASK) << RX_FIFO_LEVEL_POSI);

    REG_WRITE(PCM_CN, value);
}

static void i2s_set_txint_level(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);

    value &= ~(TX_FIFO_LEVEL_MASK << TX_FIFO_LEVEL_POSI) ;
    value |= ((val & TX_FIFO_LEVEL_MASK) << TX_FIFO_LEVEL_POSI);

    REG_WRITE(PCM_CN, value);
}

static void i2s_set_rxfifo_clr(void)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    value |= RX_FIFO_CLR;

    REG_WRITE(PCM_CN, value);
}

static void i2s_set_txfifo_clr(void)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    value |= TX_FIFO_CLR;

    REG_WRITE(PCM_CN, value);
}

__maybe_unused static void i2s_set_lrcom_store(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= LRCOM_STORE;
    }
    else
    {
        value &= ~(LRCOM_STORE);
    }
    REG_WRITE(PCM_CN, value);
}

__maybe_unused static void i2s_set_parallel_enable(UINT8 val)
{
    UINT32 value;

    value = REG_READ(PCM_CN);
    if(val)
    {
        value |= PARALLEL_EN;
    }
    else
    {
        value &= ~(PARALLEL_EN);
    }
    REG_WRITE(PCM_CN, value);
}

static void i2s_icu_configuration(UINT32 enable)
{
    UINT32 param;

    if(enable)
    {
        param = PWD_I2S_PCM_CLK_BIT;
        sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);

        param = (IRQ_I2S_PCM_BIT);
        sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
    }
    else
    {
        param = (IRQ_I2S_PCM_BIT);
        sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);

        param = PWD_I2S_PCM_CLK_BIT;
        sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
    }
}

static void i2s_gpio_configuration(void)
{
    uint32_t val;
    val = GFUNC_MODE_I2S;/*gpio 2-5*/
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);
    val = GFUNC_MODE_I2S_GPIO_21;/*gpio 21*/
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &val);
}

static UINT8 i2s_get_busy(void)
{
    //TODO
    return 0;
}

static void i2s_master_enable(UINT32 enable)
{
    UINT32 value, ultemp;

    ultemp = 1;

    value = REG_READ(PCM_CN);
    value = value | (RX_INT_EN | TX_INT_EN);
    REG_WRITE(PCM_CN, value);

    /* enable i2s unit */
    i2s_ctrl(I2S_CMD_UNIT_ENABLE, (void *) &ultemp);

    /* 1:MASTER   0:SLAVE */
    if(enable)
    {
        i2s_ctrl(I2S_CMD_SET_MSTEN, (void *) &enable);
    }
    else
    {
        i2s_ctrl(I2S_CMD_SET_MSTEN, (void *) &enable);
    }

    bit_dbg("[-ISR-]I2S_DEBUG: pcm_ctrl=0x%X,pcm_cn =0x%08X,pcm_stat =0x%X\r\n",REG_READ(PCM_CTRL),REG_READ(PCM_CN),REG_READ(PCM_STAT));

    i2s_icu_configuration(1);  //enable clock;
}

static void i2s_dma_master_enable(UINT32 enable)
{
    UINT32 value , ultemp;

    ultemp = 1;

    value = REG_READ(PCM_CN);
    value = value | (RX_INT_EN | TX_INT_EN);
    REG_WRITE(PCM_CN, value);

    /* enable i2s unit */
    i2s_ctrl(I2S_CMD_UNIT_ENABLE, (void *) &ultemp);

    /* 1:MASTER   0:SLAVE */
    if(enable)
    {
        i2s_ctrl(I2S_CMD_SET_MSTEN, (void *) &enable);
    }
    else
    {
        i2s_ctrl(I2S_CMD_SET_MSTEN, (void *) &enable);
    }

    bit_dbg("[-DMA-]I2S_DEBUG: pcm_ctrl=0x%X,pcm_cn =0x%08X,pcm_stat =0x%X\r\n",REG_READ(PCM_CTRL),REG_READ(PCM_CN),REG_READ(PCM_STAT));

    //i2s_icu_configuration(!enable);  //enable clock;
    {
        UINT32 param = PWD_I2S_PCM_CLK_BIT;
        sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);
    }
}

static UINT8 i2s_disable_i2s(void)
{
    UINT8  param;
    UINT32 status;

    param = 0;
    i2s_ctrl(I2S_CMD_UNIT_ENABLE, (void *)&param);

    status = REG_READ(PCM_STAT);

    REG_WRITE(PCM_CTRL, 0);
    REG_WRITE(PCM_CN, 0);
    REG_WRITE(PCM_STAT, status);

    i2s_icu_configuration(0);  //disable clock;
    return 0;
}

__maybe_unused static UINT32 i2s_read_rxfifo(UINT32 *data)
{
    UINT32 i2s_sta, i2s_data;

    i2s_sta = REG_READ(PCM_STAT);

    while(1)
    {
        if((i2s_sta & RX_FIFO_RD_READY))
        {
            i2s_data = REG_READ(PCM_DAT0);
            if(data)
            {
                *data = i2s_data;
            }
            return 1;
        }
        delay_us(100);
    }
    return 0;
}

__maybe_unused static void i2s_txfifo_fill(void)
{
    UINT32 i2s_sta;

    i2s_sta = REG_READ(PCM_STAT);

    while((i2s_sta & TX_FIFO_WR_READY))
    {
        REG_WRITE(PCM_DAT0, 0xFF);
    }
}

__maybe_unused static UINT32 i2s_write_txfifo(UINT32 data)
{
    UINT32 value;

    value = REG_READ(PCM_STAT);

    if(value & TX_FIFO_WR_READY)
    {
        REG_WRITE(PCM_STAT, data);
        return 1;
    }

    return 0;
}

void i2s_init(int register_isr)
{
	if (register_isr) {
        intc_service_register(IRQ_I2S_PCM, PRI_IRQ_I2S_PCM, i2s_isr);
	}
    sddev_register_dev(I2S_DEV_NAME, &i2s_op);
	REG_WRITE(I2S_CLK_RST, I2S_CLK_RST_SOFT_RESET_POS);
}

void i2s_exit(void)
{
    sddev_unregister_dev(I2S_DEV_NAME);
}

static void i2s_enable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_I2S_PCM_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
}

static void i2s_disable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_I2S_PCM_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

static UINT32 i2s_ctrl(UINT32 cmd, void *param)
{
    UINT8 ret = I2S_SUCCESS;

    //peri_busy_count_add();

    switch(cmd)
    {
    case I2S_CMD_UNIT_ENABLE:
        i2s_active(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_MSTEN:
        i2s_set_msten(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_SELECT_MODE:
        i2s_set_select_mode(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_LRCK:
        i2s_set_lrck(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_SCK_INV:
        i2s_set_sck_inv(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_LSB_FIRST:
        i2s_set_lsb_first(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_SCK_SYNCLEN:
        i2s_set_sck_synclen(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_PCM_DLEN:
        i2s_set_pcm_dlen(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_FREQ_DATAWIDTH:
        i2s_set_freq_datawidth((i2s_rate_t *)param);
        break;
    case I2S_CMD_SET_RXINT_ENABLE:
        i2s_set_rxint_enable(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_TXINT_ENABLE:
        i2s_set_txint_enable(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_RXOVR_ENABLE:
        i2s_set_rxovr_enable(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_TXOVR_ENABLE:
        i2s_set_txovr_enable(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_RXFIFO_CLR:
        i2s_set_rxfifo_clr();
        break;
    case I2S_CMD_SET_TXFIFO_CLR:
        i2s_set_txfifo_clr();
        break;
    case I2S_CMD_SET_RXINT_LEVEL:
        i2s_set_rxint_level(*(UINT8 *)param);
        break;
    case I2S_CMD_SET_TXINT_LEVEL:
        i2s_set_txint_level(*(UINT8 *)param);
        break;
    case I2S_CMD_GET_BUSY:
        i2s_get_busy();
        break;
    case I2S_CMD_ENABLE_INTERRUPT:
        i2s_enable_interrupt();
        break;
    case I2S_CMD_DISABLE_INTERRUPT:
        i2s_disable_interrupt();
        break;
    case I2S_CMD_MASTER_ENABLE:
        i2s_master_enable(*(UINT32 *)param);
        break;
    case I2S_CMD_DISABLE_I2S:
        i2s_disable_i2s();
        break;
    case I2S_CMD_DMA_MASTER_ENABLE:
        i2s_dma_master_enable(*(UINT32 *)param);
        break;
    case I2S_CMD_DMA_ISR:
        intc_service_change_handler(IRQ_I2S_PCM, (FUNCPTR)param);
        break;

    default:
        ret = I2S_FAILURE;
        break;
    }

   // peri_busy_count_dec();

    return ret;
}

// Todo: solve the todo list before using the i2s_configure() API
UINT32 i2s_configure(UINT32 fifo_level, UINT32 sample_rate, UINT32 bits_per_sample, UINT32 mode)
{
    UINT32 param;
    i2s_rate_t rate;
    rate.datawidth = bits_per_sample;
    rate.freq = sample_rate;
/*
    mode:
    bit29~27:   mode & (0x07 << 27)
                000:I2S
                001:Left Justified
                010:Right Justified
                011:reserve
                100:Short Frame Sync
                101:Long Frame Sync
                110:Normal 2B+D
                111:Delay 2B+D

    bit26:      mode & (0x01 << 26)
                0:LRCK no turn
                1:LRCK turn

    bit25:      mode & (0x01 << 25)
                0:SCK no turn
                1:SCK turn

    bit24:      mode & (0x01 << 24)
                0:MSB first send/receive
                1:LSB first send/receive

    bit23~21:   mode & (0x07 << 21)
                0~7:Sync length only Long Frame Sync effective(vav=1~7 -> bit_clk_cycle=2~8)

    bit15~13:   mode & (0x07 << 13)
                0~7:2B+D's D length in PCM_mode
*/
    /* set work mode */
    param = (mode & (0x07 << 27));
    param = param >> 27;
    i2s_ctrl(I2S_CMD_SET_SELECT_MODE, (void *)&param);// i2s mode set

    /* set lrckrp */
    param = (mode & (0x01 << 26));
    param = param >> 26;
    i2s_ctrl(I2S_CMD_SET_LRCK, (void *)&param);

    /* set sclkinv */
    param = (mode & (0x01 << 25));
    param = param >> 25;
    i2s_ctrl(I2S_CMD_SET_SCK_INV, (void *)&param);

    /* set lsbfirst */
    param = (mode & (0x01 << 24));
    param = param >> 24;
    i2s_ctrl(I2S_CMD_SET_LSB_FIRST, (void *)&param);

    /* set synclen */
    param = (mode & (0x07 << 21));
    param = param >> 21;
    i2s_ctrl(I2S_CMD_SET_SCK_SYNCLEN, (void *)&param);

    /* set pcm_dlen */
    param = (mode & (0x07 << 13));
    param = param >> 13;
    i2s_ctrl(I2S_CMD_SET_PCM_DLEN, (void *)&param);

    /* set txfifo level */
    param = fifo_level;
    i2s_ctrl(I2S_CMD_SET_TXINT_LEVEL, (void *)&param);

    /* set rxfifo level */
    param = fifo_level;
    i2s_ctrl(I2S_CMD_SET_RXINT_LEVEL, (void *)&param);

    /* enable txover int */
    param = 1;
    i2s_ctrl(I2S_CMD_SET_TXOVR_ENABLE, (void *)&param);

    /* enable rxover int */
    param = 1;
    i2s_ctrl(I2S_CMD_SET_RXOVR_ENABLE, (void *)&param);

    i2s_gpio_configuration();  //set gpio

    param = REG_READ(SCTRL_BLOCK_EN_CFG);
    /* Todo To confirmed it bit[17:16] in sys_ctrl_reg0x4b is NULL in address mapping */
    param = (param & (~(0x0FFFUL<<20))) | (BLOCK_EN_WORD_PWD<<20) | (1<<17);
    REG_WRITE(SCTRL_BLOCK_EN_CFG, param);    // audio pll audio enable

    /* set freq_datawidth */
    i2s_ctrl(I2S_CMD_SET_FREQ_DATAWIDTH, (void *)&rate);

    /* clear state */
    REG_WRITE(PCM_STAT, 0x0000003F);

    bit_dbg("[---]I2S_CONF:I2S_REG0X4_PCM_CFG = 0x%x\n", REG_READ(PCM_CTRL));
    bit_dbg("[---]I2S_CONF:I2S_REG0X5_PCM_CN  = 0x%x\n", REG_READ(PCM_CN));

    return I2S_SUCCESS;
}

UINT32 i2s_close(void)
{
    UINT32 param = 0;

    i2s_icu_configuration(0); // close clock;

    i2s_ctrl(I2S_CMD_DISABLE_INTERRUPT, (void *)&param);
    i2s_ctrl(I2S_CMD_UNIT_ENABLE, (void *)&param);

    return I2S_SUCCESS;
}

// Todo: this i2s_transfer() API looks like can not transfer data
UINT32 i2s_transfer(UINT32 *i2s_send_buf , UINT32 *i2s_recv_buf, UINT32 count, UINT32 param )
{
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
    i2s_trans.trans_done = 0;
    i2s_trans.tx_remain_data_cnt = count;
    i2s_trans.rx_remain_data_cnt = count;
    i2s_trans.p_tx_buf =(UINT32 *) i2s_send_buf;
    i2s_trans.p_rx_buf =(UINT32 *) i2s_recv_buf;
    i2s_fifo_level.tx_level = FIFO_LEVEL_24;
    i2s_fifo_level.rx_level = FIFO_LEVEL_24;
    GLOBAL_INT_RESTORE();

    if(param)
    {
        delay_ms(1000);
    }

    /* rxfifo clear enable */
    //i2s_ctrl(I2S_CMD_RXFIFO_CLR_EN, NULL);

    /* trx fifo clean */
    i2s_ctrl(I2S_CMD_SET_RXFIFO_CLR, NULL);
    i2s_ctrl(I2S_CMD_SET_TXFIFO_CLR, NULL);

#ifdef MASTER_MODE_AND_RECEIVE_DATA
    /* master enable */
    i2s_ctrl(I2S_CMD_MASTER_ENABLE,(void*)&param);

    while(!i2s_trans.trans_done)
    {
        i2s_write_txfifo(*i2s_trans.p_tx_buf);
        i2s_trans.p_tx_buf++;
        i2s_trans.tx_remain_data_cnt--;
        if (i2s_trans.tx_remain_data_cnt == 0)
        {
            i2s_trans.trans_done = 1;
        }
    }
#endif

#ifdef SLAVE_MODE_AND_RECEIVE_DATA
    /* slave enable */
    i2s_ctrl(I2S_CMD_SLAVE_ENABLE,(void*)&param);

    while(!i2s_trans.trans_done)
    {
        i2s_read_rxfifo(i2s_trans.p_rx_buf);
        i2s_trans.p_rx_buf++;
        i2s_trans.rx_remain_data_cnt--;
        if (i2s_trans.rx_remain_data_cnt == 0)
        {
            i2s_trans.trans_done = 1;
        }
    }
#endif

    delay_ms(1000);
    i2s_trans.trans_done = 0;

    i2s_ctrl(I2S_CMD_DISABLE_I2S, NULL);

    return i2s_trans.trans_done;
}

void i2s_isr(void)
{
    uint16_t i,rxint,txint,ultemp;
    uint32_t i2s_status;
    volatile uint16_t data_num ;

    i2s_status= REG_READ(PCM_STAT);
    rxint = i2s_status & 0x01;
    txint = i2s_status & 0x02;

    if(txint)
    {
        switch(i2s_fifo_level.tx_level)
        {
            case FIFO_LEVEL_1  : data_num = 1; break;
            case FIFO_LEVEL_8  : data_num = 8; break;
            case FIFO_LEVEL_16  : data_num = 16; break;
            default : data_num = 24; break;
        }

        if(i2s_trans.p_tx_buf == NULL)
        {
            for(i=0; i<data_num; i++)
            {
                REG_WRITE(PCM_DAT0,0xEEEEEEEE);
            }
        }
        else
        {
            for(i = 0; i < data_num; i++)
            {
                REG_WRITE(PCM_DAT0,*i2s_trans.p_tx_buf);
                i2s_trans.p_tx_buf++;
            }
        }
        i2s_status |= 0x2;
    }

    if(rxint)
    {
        switch(i2s_fifo_level.rx_level)
        {
            case FIFO_LEVEL_1  : data_num = 1; break;
            case FIFO_LEVEL_8  : data_num = 8; break;
            case FIFO_LEVEL_16  : data_num = 16; break;
            default : data_num = 24; break;
        }

        if(data_num > i2s_trans.rx_remain_data_cnt)
        {
            data_num = i2s_trans.rx_remain_data_cnt;
        }

        if((i2s_trans.p_rx_buf == NULL) || (i2s_status & TX_FIFO_WR_READY))
        {
            for(i=0; i<data_num; i++)
            {
                ultemp = REG_READ(PCM_DAT0);
                (void) ultemp;
            }
        }
        else
        {
            for(i=0; i<data_num; i++)
            {
                *i2s_trans.p_rx_buf = REG_READ(PCM_DAT0);
                i2s_trans.p_rx_buf++;
            }

            i2s_trans.rx_remain_data_cnt -= data_num;

            if(i2s_trans.rx_remain_data_cnt <=0)
            {
                i = 0;
                i2s_trans.trans_done = 1;
                i2s_ctrl(I2S_CMD_SET_TXINT_ENABLE, (void*)&i);
                i2s_ctrl(I2S_CMD_SET_RXINT_ENABLE, (void*)&i);
            }
        }
        i2s_status |= 0x1;
    }

    REG_WRITE(PCM_STAT,i2s_status);
}
#endif
// eof

