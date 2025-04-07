#include "include.h"
#include "arm_arch.h"

#include "irda_bk7252n.h"
#include "irda_pub_bk7252n.h"

#include "target_util_pub.h"
#include "drv_model_pub.h"
#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "sys_ctrl_pub.h"
#include "bk_timer_pub.h"
#include "rtos_pub.h"

#if (SOC_BK7252N == CFG_SOC_NAME)

static UINT32 irda_ctrl(UINT32 cmd, void *param);
static SDD_OPERATIONS irda_op = {
    irda_ctrl
};
//global
volatile UINT8 rx_done_flag=0;
volatile UINT8 tx_done_flag=0;

extern void delay_us(UINT32 ms_count);

static void irda_active(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_TX_IRDA_PWD;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_TX_IRDA_PWD;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_tx_start(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_TX_START;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_TX_START;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_tx_initial_level(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_TX_INITIAL_LV;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_TX_INITIAL_LV;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_txenable(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_TX_EN;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_TX_EN;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_rx_initial_level(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_RX_INITIAL_LV;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_RX_INITIAL_LV;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_rxenable(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    if(enable)
    {
        value |= IRDA_REG0X4_CONFIG_RX_EN;
    }
    else
    {
        value &= ~IRDA_REG0X4_CONFIG_RX_EN;
    }
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_clk(UINT16 clk)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    value &= ~(IRDA_REG0X4_CONFIG_CLK_FREQ_IN_MASK << IRDA_REG0X4_CONFIG_CLK_FREQ_IN_POS);
    value |= (clk << IRDA_REG0X4_CONFIG_CLK_FREQ_IN_POS);
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_txdata_num(UINT16 val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X4_CONFIG);
    value &= ~(IRDA_REG0X4_CONFIG_TX_DATA_NUM_MASK << IRDA_REG0X4_CONFIG_TX_DATA_NUM_POS);
    value |= (val << IRDA_REG0X4_CONFIG_TX_DATA_NUM_POS);
    REG_WRITE(IRDA_REG0X4_CONFIG, value);
}

static void irda_set_rx_timeout_cnt(UINT16 cnt)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X5_INT);
    value &= ~(IRDA_REG0X5_INT_RX_TIMEOUT_CNT_MASK << IRDA_REG0X5_INT_RX_TIMEOUT_CNT_POS);
    value |= (cnt << IRDA_REG0X5_INT_RX_TIMEOUT_CNT_POS);
    REG_WRITE(IRDA_REG0X5_INT, value);
}

static void irda_set_rx_fifo_threshold(UINT16 val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X5_INT);
    value &= ~(IRDA_REG0X5_INT_RX_FIFO_THR_MASK << IRDA_REG0X5_INT_RX_FIFO_THR_POS);
    value |= (val << IRDA_REG0X5_INT_RX_FIFO_THR_POS);
    REG_WRITE(IRDA_REG0X5_INT, value);
}

static void irda_set_tx_fifo_threshold(UINT16 val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X5_INT);
    value &= ~(IRDA_REG0X5_INT_TX_FIFO_THR_MASK << IRDA_REG0X5_INT_TX_FIFO_THR_POS);
    value |= (val << IRDA_REG0X5_INT_TX_FIFO_THR_POS);
    REG_WRITE(IRDA_REG0X5_INT, value);
}

static UINT32 irda_get_rxdate_num(void)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X6_FIFO);
    value = (value >> IRDA_REG0X6_FIFO_RXDATA_NUM_POS) & IRDA_REG0X6_FIFO_RXDATA_NUM_MASK;

    return value;
}

static UINT16 irda_get_fifo_data_rx(void)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X7_DATA);
    value = (value >> IRDA_REG0X7_DATA_FIFO_DATA_RX_POS) & IRDA_REG0X7_DATA_FIFO_DATA_RX_MASK;

    return (UINT16)(value);
}

static void irda_set_fifo_data_tx(UINT16 val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X7_DATA);
    value &= ~(IRDA_REG0X7_DATA_FIFO_DATA_TX_MASK << IRDA_REG0X7_DATA_FIFO_DATA_TX_POS);
    value |= (val << IRDA_REG0X7_DATA_FIFO_DATA_TX_POS);
    REG_WRITE(IRDA_REG0X7_DATA, value);
}

static UINT16 rx_data_array[1024];

__maybe_unused static UINT16 * irda_read_data(void)
{
    UINT16 rx_data;
    UINT8 rx_num;

    UINT32 rx_data_index = 0;

    if (rx_done_flag == 0)
    {
        rx_num =irda_get_rxdate_num();

        for (rx_data_index = 0; rx_data_index < rx_num; rx_data_index++) {
            rx_data = irda_get_fifo_data_rx();
            rx_data_array[rx_data_index] = rx_data;
        }
        return rx_data_array;
    }
    else
    {
        return 0;
    }
}

static void irda_set_rx_need_rd_mask(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X8_INT_MASK);
    if(enable)
    {
        value |= IRDA_REG0X8_INT_MASK_RX_NEED_RD_MASK;
    }
    else
    {
        value &= ~IRDA_REG0X8_INT_MASK_RX_NEED_RD_MASK;
    }
    REG_WRITE(IRDA_REG0X8_INT_MASK, value);
}

__maybe_unused static void irda_set_tx_need_wr_mask(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X8_INT_MASK);
    if(enable)
    {
        value |= IRDA_REG0X8_INT_MASK_TX_NEED_WR_MASK;
    }
    else
    {
        value &= ~IRDA_REG0X8_INT_MASK_TX_NEED_WR_MASK;
    }
    REG_WRITE(IRDA_REG0X8_INT_MASK, value);
}

static void irda_set_tx_done_mask(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X8_INT_MASK);
    if(enable)
    {
        value |= IRDA_REG0X8_INT_MASK_TX_DONE_MASK;
    }
    else
    {
        value &= ~IRDA_REG0X8_INT_MASK_TX_DONE_MASK;
    }
    REG_WRITE(IRDA_REG0X8_INT_MASK, value);
}

static void irda_set_rx_timeout_mask(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0X8_INT_MASK);
    if(enable)
    {
        value |= IRDA_REG0X8_INT_MASK_RX_TIMEOUT_MASK;
    }
    else
    {
        value &= ~IRDA_REG0X8_INT_MASK_RX_TIMEOUT_MASK;
    }
    REG_WRITE(IRDA_REG0X8_INT_MASK, value);
}

static void irda_set_carrier_polarity(UINT16 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XA_CONFIG2);
    if(enable)
    {
        value |= IRDA_REG0XA_CONFIG2_CARRIER_POLARITY;
    }
    else
    {
        value &= ~IRDA_REG0XA_CONFIG2_CARRIER_POLARITY;
    }
    REG_WRITE(IRDA_REG0XA_CONFIG2, value);
}

static void irda_set_softreset(UINT32 enable)
{
    UINT32 reg_val = REG_READ(IRDA_REG0X2_CLK_RST);
    if(enable)
        reg_val |= IRDA_REG0X2_CLK_RST_SOFT_RESET;
    else
        reg_val &= ~IRDA_REG0X2_CLK_RST_SOFT_RESET;
    REG_WRITE(IRDA_REG0X2_CLK_RST, reg_val);
}

static void irda_set_carrier_duty(UINT16 duty_val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XA_CONFIG2);
    value &= ~(IRDA_REG0XA_CONFIG2_CARRIER_DUTY_MASK << IRDA_REG0XA_CONFIG2_CARRIER_DUTY_DOS);
    value |= (duty_val << IRDA_REG0XA_CONFIG2_CARRIER_DUTY_DOS);
    REG_WRITE(IRDA_REG0XA_CONFIG2, value);
}

static void irda_set_carrier_period(UINT16 period_val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XA_CONFIG2);
    value &= ~(IRDA_REG0XA_CONFIG2_CARRIER_PERIOD_MASK << IRDA_REG0XA_CONFIG2_CARRIER_PERIOD_POS);
    value |= (period_val << IRDA_REG0XA_CONFIG2_CARRIER_PERIOD_POS);
    REG_WRITE(IRDA_REG0XA_CONFIG2, value);
}

static void irda_set_rx_start_thr(UINT16 val)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XA_CONFIG2);
    value &= ~(IRDA_REG0XA_CONFIG2_RX_START_THR_MASK << IRDA_REG0XA_CONFIG2_RX_START_THR_POS);
    value |= (val << IRDA_REG0XA_CONFIG2_RX_START_THR_POS);
    REG_WRITE(IRDA_REG0XA_CONFIG2, value);
}

static void irda_set_glitch_enable(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XB_CONFIG3);
    if(enable)
    {
        value |= IRDA_REG0XB_CONFIG3_GLITCH_ENABLE;
    }
    else
    {
        value &= ~IRDA_REG0XB_CONFIG3_GLITCH_ENABLE;
    }
    REG_WRITE(IRDA_REG0XB_CONFIG3, value);
}

static void irda_set_rfu2(UINT16 rfu2)
{
    UINT32 value;

    value = REG_READ(IRDA_REG0XB_CONFIG3);
    value &= ~(0x1fff << 3);
    value |= rfu2;
    REG_WRITE(IRDA_REG0XB_CONFIG3, value);
}

void irda_tx_initial(void)
{
    UINT32 param;

    param = PCLK_POSI_IRDA;
    sddev_control(ICU_DEV_NAME,CMD_CONF_PCLK_26M,&param);//irda clk:26M

    param = PWD_IRDA_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);//clk power up

    param = (IRQ_IRDA_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);

    param = GFUNC_MODE_IRDA;
    sddev_control(GPIO_DEV_NAME,CMD_GPIO_ENABLE_SECOND,&param);//gpio config

    irda_set_softreset(1);
    irda_set_clk(26);
    irda_set_carrier_duty(10);
    irda_set_carrier_period(26);
    irda_txenable(1);
    irda_set_tx_done_mask(1);
    irda_set_rx_timeout_mask(1);
}

void irda_rx_initial(void)
{
    UINT32 param;

    param = PCLK_POSI_IRDA;
    sddev_control(ICU_DEV_NAME,CMD_CONF_PCLK_26M,&param);//irda clk:26M

    param = PWD_IRDA_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);//clk power up

    param = (IRQ_IRDA_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);

    param = GFUNC_MODE_IRDA;
    sddev_control(GPIO_DEV_NAME,CMD_GPIO_ENABLE_SECOND,&param);//gpio config

    irda_set_softreset(1);
    irda_set_clk(26);
    irda_set_rx_timeout_cnt(8000);
    irda_set_rx_start_thr(1000);
    irda_set_glitch_enable(1);//gltch
    irda_set_rfu2(5);
    //clrf_IRDA_Reg0x0_rx_initial_level;
    irda_set_rx_fifo_threshold(20);
    irda_rxenable(1);
    irda_set_tx_done_mask(1);//FOR THR TEST
    irda_set_rx_need_rd_mask(1);//FOR THR TEST
    irda_set_rx_timeout_mask(1);
}

__maybe_unused void bk_irda_driver_init(void)
{
    UINT32 reg_val = 0x0;

    irda_init();
    reg_val = REG_READ(IRDA_REG0X2_CLK_RST);
    reg_val &= ~IRDA_REG0X2_CLK_RST_SOFT_RESET;
    delay_us(200);
    reg_val = REG_READ(IRDA_REG0X2_CLK_RST);
    reg_val |= IRDA_REG0X2_CLK_RST_SOFT_RESET;
}

__maybe_unused void bk_irda_driver_deinit(void)
{
    irda_exit();
}

void irda_tx_start(void)
{
    irda_set_tx_start(0);
    delay_us(200);
    irda_set_tx_start(1);
    delay_us(200);
}

void irda_test(UINT8 pPara)
{
    UINT8 id = pPara;
    UINT8 i;
    UINT8 tx_num, rx_num;
    UINT16 send_para[]={1023,0,1023,0,1023,0};
    UINT16* p_send_para;
    irda_set_tx_initial_level(0);
    irda_set_rx_initial_level(0);
    irda_set_carrier_polarity(0);
    if(id == 0)
    {
        irda_tx_initial();
        tx_num = sizeof(send_para)/sizeof(UINT16);
        p_send_para = send_para;
        for (i=0; i<tx_num;i++)
        {
            irda_set_fifo_data_tx(p_send_para[i]);
        }
        irda_set_txdata_num(tx_num);
        irda_set_tx_fifo_threshold(1);
        //enable tx
        irda_tx_start();
        while(tx_done_flag==0)
        {
            delay_us(2000);
        }
        tx_done_flag=0;
        delay_us(2000);
    }
    else if(id == 1)
    {
        irda_rx_initial();
        rx_done_flag = 0;

        while (rx_done_flag == 0)
        {
            os_null_printf(" irda rxing now..\r\n");
            delay_us(2000);
        }
        rx_num = irda_get_rxdate_num();
        for (i=0; i < rx_num; i++)
        {
            os_printf("irda rx data[%d]=%d.\r\n", i, rx_data_array[i]);
        }
    }
}

static void trng_active(UINT8 enable)
{
    UINT32 value;

    value = REG_READ(TRNG_CTRL);
    if(enable)
    {
        value |= TRNG_EN;
    }
    else
    {
        value &= ~TRNG_EN;
    }
    REG_WRITE(TRNG_CTRL, value);
}

void trng_read_delay(UINT32 time)
{
    volatile UINT32 i = 0;
    for( i=0; i<time; i++){
        ;
    }
}

static UINT32 trng_get_random(void)
{
    UINT32 value;

    trng_active(1);

    /* The first set of 100 trng value read upon power-up is identical to the
        * next set of 100 trng value read upon subsequent power-up, when using the
        * delay function:rtos_delay_milliseconds(1) or trng_read_delay(100); Incresing
        * the delay time from trng_read_delay(100) to trng_read_delay(170000)
        * the comparasion involves reading data after powering down and up, occasionally
        * the first one or two or three trng value are the same, so a few of such trng
        * trng values are discarded after powering up. refer to the following function:irda_init
        */
    #if CONFIG_TRNG_DELAY_ONCE
    static bool random_delay_flag = false;
    if(random_delay_flag == false){
        rtos_delay_milliseconds(1);
        random_delay_flag = true;
    }
    #else
    trng_read_delay(170000);
    #endif
    value = REG_READ(TRNG_DATA);

    trng_active(0);

    return value;
}

static UINT32 irda_ctrl(UINT32 cmd, void *param)
{
    UINT32 ret = IRDA_SUCCESS;

    switch(cmd)
    {
        case IRDA_CMD_ACTIVE:
            irda_active(*(UINT8 *)param);
            break;
        case IRDA_CMD_SET_CLK:
            irda_set_clk(*(UINT16 *)param);
            break;
        case TRNG_CMD_GET:
            *(UINT32 *)param = trng_get_random();
            break;
        default:
            ret = IRDA_FAILURE;
            break;
    }

    return ret;
}

void irda_init(void)
{
    intc_service_register(IRQ_IRDA, PRI_IRQ_IRDA, irda_isr);
    sddev_register_dev(IRDA_DEV_NAME, &irda_op);

    irda_active(1);

#if CONFIG_TRNG_PATCH
    uint32_t discard_cnt = 4, i;
    for(i = 0; i < discard_cnt; i ++)
    {
        trng_get_random();
    }
#endif
}

void irda_exit(void)
{
    sddev_unregister_dev(IRDA_DEV_NAME);
}

void irda_isr(void)
{
    UINT32 irda_int_status;
    irda_int_status = REG_READ(IRDA_REG0X9_INT_STA);
    if (irda_int_status & IRDA_REG0X9_INT_STA_TX_DONE_STA)
    {
        tx_done_flag = 1;
    }
    if (irda_int_status & IRDA_REG0X9_INT_STA_RX_DONE_STA)
    {
        irda_read_data();
        rx_done_flag = 1;
    }
    if (irda_int_status & IRDA_REG0X9_INT_STA_RX_NEED_ED_STA)
    {
        irda_set_rx_need_rd_mask(0);
    }
    REG_WRITE(IRDA_REG0X9_INT_STA, irda_int_status);
}
#endif
// eof

