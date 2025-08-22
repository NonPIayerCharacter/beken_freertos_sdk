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

#include "include.h"
#include "arm_arch.h"
#include "flash_bypass.h"
#include "gpio_pub.h"
#include "gpio.h"
#include "icu_pub.h"
#include "flash_pub.h"
#include "spi_pub.h"
#include "sys_ctrl.h"
#if (CFG_SOC_NAME == SOC_BK7221U)
#include "spi.h"
#elif((CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N) || (CFG_SOC_NAME == SOC_BK7231N))
#include "spi_bk7231n.h"
#include "mem_pub.h"
#include "target_util_pub.h"
#endif
#include "icu.h"
#include "flash.h"

#if ((CFG_SOC_NAME == SOC_BK7221U))
extern void spi_gpio_configuration(void);
extern void bk_spi_configure(UINT32 rate, UINT32 mode);

int flash_bypass_wr_sr_cb(uint32_t larg, uint32_t rarg)
{
    volatile int32_t delay_count, j;
    uint8_t sr_width = (uint8_t)larg;
    uint16_t sr_val = (uint16_t)rarg;
    uint32_t intc_enable_reg, reg;
    char *text_ptr, temp_buf = 0;
    uint32_t i, reg_ctrl, reg_dat;
    uint32_t reg_stat, reg_sctrl;
    int exceptional_flag = 0;
    UINT32 ret, mode, is_spi_mode;

    if (0 == flash_is_support_0x50h_cmd()
            || (sr_width > 2))
        return exceptional_flag;

    /*step 0, disable interrupt*/
    intc_enable_reg = REG_READ(ICU_GLOBAL_INT_EN);
    reg = intc_enable_reg & (~(GINTR_FIQ_EN | GINTR_IRQ_EN));
    REG_WRITE(ICU_GLOBAL_INT_EN, reg);

    /*step 1, save spi register configuration*/
    reg_ctrl = REG_READ(SPI_CTRL);
    reg_stat = REG_READ(SPI_STAT);
    reg_dat  = REG_READ(SPI_DAT);
    reg_sctrl = REG_READ(SPI_SLAVE_CTRL);

    /*step 2, resident cache*/
    REG_WRITE(SPI_CTRL, 0);
    do {
        text_ptr = (char *)flash_bypass_wr_sr_cb;
        for (i = 0; i < CURRENT_ROUTINE_TEXT_SIZE; i ++)
            temp_buf += text_ptr[i];

        REG_WRITE(SPI_STAT, temp_buf);
    } while (0);

    /*step 3, config spi master*/
    /*     3.1 clear spi fifo content*/
    reg = REG_READ(SPI_STAT);
    while ((reg & RXFIFO_EMPTY) == 0) {
        REG_READ(SPI_DAT);
        reg = REG_READ(SPI_STAT);
    }

    /*     3.2 disable spi block*/
    reg = REG_READ(ICU_PERI_CLK_PWD);
    reg |= (PWD_SPI_CLK_BIT);
    REG_WRITE(ICU_PERI_CLK_PWD, reg);

    /*     3.3 clear spi status*/
    REG_WRITE(SPI_CTRL, 0);
    reg = REG_READ(SPI_STAT);
    REG_WRITE(SPI_STAT, reg);

    reg = REG_READ(SPI_SLAVE_CTRL);
    REG_WRITE(SPI_SLAVE_CTRL, reg);

    /*     3.4 save the previous setting status*/
    ret = gpio_get_config(GPIO14, &mode);
    is_spi_mode = 0;
    if ((GPIO_SUCCESS == ret) && (PERIAL_MODE_2 == mode))
        is_spi_mode = 1;

    /*     3.5 set the spi master mode*/
    bk_spi_configure(SPI_DEF_CLK_HZ, SPI_DEF_MODE);

    /*step 4, gpio(14/15/16/17) are set as high-impedance state or input state ,
              for spi mux with them*/
    gpio_config(GPIO14, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO15, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO16, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO17, GMODE_SET_HIGH_IMPENDANCE);

    /*step 5, switch flash interface to spi:sctrl_flash_select_spi_controller()
     *        Pay attention to prefetch instruction destination, the text can not
     *        fetch from flash space after this timepoint.
     */
    reg = REG_READ(SCTRL_CONTROL);
    reg |= FLASH_SPI_MUX_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);
    while (1) { /* double check the bit:FLASH_SPI_MUX_BIT. The bit cannot be set successfully, if cpu is fetching instruction*/
        reg = REG_READ(SCTRL_CONTROL);
        if (reg & FLASH_SPI_MUX_BIT)
            break;
    }

    /*step 6, write enable for volatile status register: 50H*/
    /*      6.1:take cs*/
    reg = REG_READ(SPI_CTRL);
    reg &= ~(NSSMD_MASK << NSSMD_POSI);
    reg |= (SPI_VAL_TAKE_CS << NSSMD_POSI);
    REG_WRITE(SPI_CTRL, reg);

    /*      6.2:write tx fifo*/
    reg = REG_READ(SPI_STAT);
    if ((reg & TXFIFO_FULL) == 0)
        REG_WRITE(SPI_DAT, FLASH_CMD_WR_EN_VSR);
    else {
        exceptional_flag = -1;
        goto wr_exceptional;
    }

    /*      6.3:waiting for TXFIFO_EMPTY interrupt*/
    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & TXFIFO_EMPTY)
            break;
    }

    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & SPIBUSY)
            break;
    }

    /*delay 1us--5us. the bits of 2 clock do not send over, if the spi status is TXFIFO_EMPTY*/
    for (delay_count = 0; delay_count < 1; delay_count ++) {
        for (j = 0; j < 4; j ++)
            ;
    }

    /*      6.4:release cs*/
    reg = REG_READ(SPI_CTRL);
    reg &= ~(NSSMD_MASK << NSSMD_POSI);
    reg |= (SPI_VAL_RELEASE_CS << NSSMD_POSI);
    REG_WRITE(SPI_CTRL, reg);

    /*step 7, write flash command:01H and sr*/
    /*      7.1:take cs*/
    reg = REG_READ(SPI_CTRL);
    reg &= ~(NSSMD_MASK << NSSMD_POSI);
    reg |= (SPI_VAL_TAKE_CS << NSSMD_POSI);
    REG_WRITE(SPI_CTRL, reg);

    /*      7.2:write tx fifo*/
    reg = REG_READ(SPI_STAT);
    if ((reg & TXFIFO_FULL) == 0) {
        REG_WRITE(SPI_DAT, FLASH_CMD_WR_SR);
        for (int i = 0; i < sr_width; i ++)
            REG_WRITE(SPI_DAT, (sr_val >> (i * 8)) & 0xff);
    } else {
        exceptional_flag = -2;
        goto wr_exceptional;
    }

    /*      7.3:waiting for TXFIFO_EMPTY interrupt*/
    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & TXFIFO_EMPTY)
            break;
    }

    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & SPIBUSY)
            break;
    }

    /*delay 1us--5us. the bits of 2 clock do not send over, if the spi status is TXFIFO_EMPTY*/
    for (delay_count = 0; delay_count < 1; delay_count ++) {
        for (j = 0; j < 4; j ++)
            ;
    }

    /*      7.4:release cs*/
    reg = REG_READ(SPI_CTRL);
    reg &= ~(NSSMD_MASK << NSSMD_POSI);
    reg |= (SPI_VAL_RELEASE_CS << NSSMD_POSI);
    REG_WRITE(SPI_CTRL, reg);

    exceptional_flag = 1;

wr_exceptional:
    /*step 8, flash interface select: flash controller-->sctrl_flash_select_flash_controller()*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(FLASH_SPI_MUX_BIT);
    REG_WRITE(SCTRL_CONTROL, reg);

    /*step 9, gpio(14/15/16/17) second function*/
    if (is_spi_mode)
        spi_gpio_configuration();

    /*step 10, restore spi register configuration*/
    REG_WRITE(SPI_CTRL, reg_ctrl);
    REG_WRITE(SPI_STAT, reg_stat);
    REG_WRITE(SPI_DAT, reg_dat);
    REG_WRITE(SPI_SLAVE_CTRL, reg_sctrl);

    /*step 11, enable interrupt*/
    REG_WRITE(ICU_GLOBAL_INT_EN, intc_enable_reg);

    if (exceptional_flag < 0)
        os_printf("fb_wr_sr failed %d\r\n", exceptional_flag);

    return exceptional_flag;
}
#elif ((CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N) || (CFG_SOC_NAME == SOC_BK7231N))
int flash_bypass_op_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf, uint32_t rx_len)
{
    uint32_t intc_enable_reg, reg;
    uint32_t reg_ctrl, reg_dat;
    uint32_t reg_stat, reg_config, reg_icu_clk, reg_icu_pwd;
    uint32_t gpio14_status, gpio15_status, gpio16_status, gpio17_status;
    uint32_t gpio14_15_cfg, gpio16_17_cfg;
    int exceptional_flag = 0, use_less = 0, k = 0, wait = 0;

    reg_config = REG_READ(SPI_CONFIG);
    reg_ctrl = REG_READ(SPI_CTRL);
    if ((reg_ctrl & SPIEN) && (reg_config & (SPI_TX_EN|SPI_RX_EN)))
    {
        return -3;
    }

    if(LINE_MODE_FOUR == flash_get_line_mode())
    {
        flash_set_line_mode(LINE_MODE_TWO);
    }

    /*step 0, disable interrupt*/
    intc_enable_reg = REG_READ(ICU_GLOBAL_INT_EN);
    reg = intc_enable_reg & (~(GINTR_FIQ_EN | GINTR_IRQ_EN));
    REG_WRITE(ICU_GLOBAL_INT_EN, reg);

    /*step 1, save spi register configuration*/
    reg_ctrl = REG_READ(SPI_CTRL);
    reg_config = REG_READ(SPI_CONFIG);
    reg_stat = REG_READ(SPI_STAT);
    reg_dat  = REG_READ(SPI_DAT);

    /*step 2, resident cache*/
    // link this function to itcm on bk7238 and bk7252n

    /*step 3, config spi master*/
    /*     3.1 clear spi fifo content*/
    reg = REG_READ(SPI_STAT);
    while (reg & RXFIFO_RD_READ) {
        REG_READ(SPI_DAT);
        reg = REG_READ(SPI_STAT);
    }

    /*     3.2 disable spi block, and backup */
    reg_icu_pwd = reg = REG_READ(ICU_PERI_CLK_PWD);
    reg |= (PWD_SPI_CLK_BIT);
    REG_WRITE(ICU_PERI_CLK_PWD, reg);

    /*     3.3 clear spi status*/
    REG_WRITE(SPI_CTRL, 0);
    REG_WRITE(SPI_CONFIG, 0);
    reg = REG_READ(SPI_STAT);
    REG_WRITE(SPI_STAT, reg);

    /*     3.4 save the previous setting status*/
    gpio14_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO14*4);
    gpio15_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO15*4);
    gpio16_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO16*4);
    gpio17_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO17*4);
    gpio14_15_cfg = REG_READ(REG_GPIO_FUNC_CFG);
    gpio16_17_cfg = REG_READ(REG_GPIO_FUNC_CFG_2);

    /*     3.5 set the spi master mode*/
    // open clock, and backup
    reg = reg_icu_clk = REG_READ(ICU_PWM_CLK_MUX);
    reg |= PCLK_POSI_SPI;
    REG_WRITE(ICU_PWM_CLK_MUX, reg);
    // select 26M
    reg = REG_READ(ICU_PERI_CLK_PWD);
    reg &= ~(PWD_SPI_CLK_BIT);
    REG_WRITE(ICU_PERI_CLK_PWD, reg);

    // set to spi config directly
    reg = 0xC00100; // spien  msten  spi_clk=1---13M
    REG_WRITE(SPI_CTRL, reg);

    /*step 4, gpio(14/15/16/17) are set as high-impedance state or input state , for spi mux with them*/
    gpio_config(GPIO14, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO15, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO16, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO17, GMODE_SET_HIGH_IMPENDANCE);

    /*step 5, switch flash interface to spi:sctrl_flash_select_spi_controller()
    *         Pay attention to prefetch instruction destination, the text can not
    *         fetch from flash space after this timepoint.
    */
    reg = REG_READ(SCTRL_CONTROL);
    reg |= FLASH_SPI_MUX_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);
    while (1) { /* double check the bit:FLASH_SPI_MUX_BIT. The bit cannot be set successfully, if cpu is fetching instruction*/
        reg = REG_READ(SCTRL_CONTROL);
        if (reg & FLASH_SPI_MUX_BIT)
            break;
    }

    /*step 7, write flash command:01H and sr*/
    /*      7.1:take cs*/
    reg = REG_READ(SPI_CONFIG);
    reg &= ~(0xffffU << SPI_TX_TRAHS_LEN_POSI);
    reg &= ~(0xffffU << SPI_RX_TRAHS_LEN_POSI);
    k = (tx_len + rx_len);
    reg |= ((k & 0xffffU) << SPI_TX_TRAHS_LEN_POSI);
    reg |= ((k & 0xffffU) << SPI_RX_TRAHS_LEN_POSI);
    reg |= (SPI_TX_EN| SPI_RX_EN | SPI_TX_FINISH_EN);
    REG_WRITE(SPI_CONFIG, reg);

    /*      7.2:write tx fifo*/
    // write tx first
    use_less = 0;
    k = 0;
    wait = 0;
    for (int i = 0; i < tx_len;) {
        reg = REG_READ(SPI_STAT);
        if ((reg & TXFIFO_WR_READ) == 0) {
            for(int j=0; j<500; j++);
            wait++;
            if(wait > 100) {
                exceptional_flag = -1;
                goto wr_exceptional;
            }
        } else {
            wait = 0;
            REG_WRITE(SPI_DAT, tx_buf[i]);
            i++;
        }

        reg = REG_READ(SPI_STAT);
        if ((reg & RXFIFO_RD_READ)) {
            if(use_less < tx_len) {
                REG_READ(SPI_DAT);
                use_less++;
            }
            else if(k < rx_len) {
                rx_buf[k] = REG_READ(SPI_DAT);
                k++;
            }
            else {
                REG_READ(SPI_DAT);
            }
        }
    }

    // read rx, send 0xff
    wait = 0;
    for (int i = 0; i < rx_len; )
    {
        reg = REG_READ(SPI_STAT);
        if ((reg & TXFIFO_WR_READ) == 0) {
            for(int j=0; j<500; j++);
            wait++;
            if(wait > 100)
            {
                exceptional_flag = -2;
                goto wr_exceptional;
            }
        }
        else {
            wait = 0;
            REG_WRITE(SPI_DAT, 0xff);
            i++;
        }

        reg = REG_READ(SPI_STAT);
        if ((reg & RXFIFO_RD_READ)) {
            if(use_less < tx_len) {
                REG_READ(SPI_DAT);
                use_less++;
            }
            else if(k < rx_len) {
                rx_buf[k] = REG_READ(SPI_DAT);
                k++;
            }
            else {
                REG_READ(SPI_DAT);
            }
        }
    }

    /*      7.3:waiting for TXFIFO_EMPTY interrupt*/
    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & TX_FINISH_INT)
            break;
        else if ((reg & RXFIFO_RD_READ)) {
            if(use_less < tx_len) {
                REG_READ(SPI_DAT);
                use_less++;
            }
            else if(k < rx_len) {
                rx_buf[k] = REG_READ(SPI_DAT);
                k++;
            }
        }
    }

    wait = 0;
    while(k < rx_len) {
        reg = REG_READ(SPI_STAT);
        if ((reg & RXFIFO_RD_READ)) {
            if(use_less < tx_len) {
                REG_READ(SPI_DAT);
                use_less++;
            }
            else if(k < rx_len) {
                rx_buf[k] = REG_READ(SPI_DAT);
                k++;
            }
            wait = 0;
        } else {
            for(int j=0; j<500; j++);
            wait++;
            if(wait > 100) {
                break;
            }
        }
    }
    exceptional_flag = k;

    /*      7.4:release cs*/
    reg = REG_READ(SPI_CONFIG);
    reg &= ~(0xffffU << SPI_TX_TRAHS_LEN_POSI);
    //reg &= ~(0xffffU << SPI_RX_TRAHS_LEN_POSI);
    reg &= ~(SPI_TX_EN | SPI_TX_FINISH_EN);
    REG_WRITE(SPI_CONFIG, reg);

wr_exceptional:
    reg = 0;
    REG_WRITE(SPI_CONFIG, reg);

    // cler stat and fifo
    reg = REG_READ(SPI_STAT);
    while (reg & RXFIFO_RD_READ) {
        REG_READ(SPI_DAT);
        reg = REG_READ(SPI_STAT);
    }
    reg = REG_READ(SPI_STAT);
    REG_WRITE(SPI_STAT, reg);

    /*step 8, flash interface select: flash controller-->sctrl_flash_select_flash_controller()*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(FLASH_SPI_MUX_BIT);
    REG_WRITE(SCTRL_CONTROL, reg);
    // recover icu and powerdown bits for spi
    REG_WRITE(ICU_PWM_CLK_MUX, reg_icu_clk);
    REG_WRITE(ICU_PERI_CLK_PWD, reg_icu_pwd);

    /*step 9, gpio(14/15/16/17) second function*/
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO14*4, gpio14_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO15*4, gpio15_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO16*4, gpio16_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO17*4, gpio17_status);
    REG_WRITE(REG_GPIO_FUNC_CFG, gpio14_15_cfg);
    REG_WRITE(REG_GPIO_FUNC_CFG_2, gpio16_17_cfg);

    /*step 10, restore spi register configuration*/
    REG_WRITE(SPI_CTRL, reg_ctrl);
    REG_WRITE(SPI_CONFIG, reg_config);
    REG_WRITE(SPI_STAT, reg_stat);
    REG_WRITE(SPI_DAT, reg_dat);

    /*step 11, enable interrupt*/
    REG_WRITE(ICU_GLOBAL_INT_EN, intc_enable_reg);

    if(LINE_MODE_FOUR == flash_get_line_mode())
    {
        flash_set_line_mode(LINE_MODE_FOUR);
    }

    return exceptional_flag;
}

int flash_bypass_op_write(uint8_t *op_code, uint8_t *tx_buf, uint32_t tx_len)
{
    uint32_t intc_enable_reg, reg;
    uint32_t reg_ctrl, reg_dat;
    uint32_t reg_stat, reg_config, reg_icu_clk, reg_icu_pwd;
    uint32_t gpio14_status, gpio15_status, gpio16_status, gpio17_status;
    uint32_t gpio14_15_cfg, gpio16_17_cfg;
    int exceptional_flag = 0;

    reg_config = REG_READ(SPI_CONFIG);
    reg_ctrl = REG_READ(SPI_CTRL);
    if ((reg_ctrl & SPIEN) && (reg_config & (SPI_TX_EN|SPI_RX_EN)))
    {
        return -3;
    }

    if(LINE_MODE_FOUR == flash_get_line_mode())
    {
        flash_set_line_mode(LINE_MODE_TWO);
    }

    /*step 0, disable interrupt*/
    intc_enable_reg = REG_READ(ICU_GLOBAL_INT_EN);
    reg = intc_enable_reg & (~(GINTR_FIQ_EN | GINTR_IRQ_EN));
    REG_WRITE(ICU_GLOBAL_INT_EN, reg);

    /*step 1, save spi register configuration*/
    reg_ctrl = REG_READ(SPI_CTRL);
    reg_config = REG_READ(SPI_CONFIG);
    reg_stat = REG_READ(SPI_STAT);
    reg_dat  = REG_READ(SPI_DAT);

    /*step 2, resident cache*/
    // link this function to itcm on bk7238 and bk7252n

    /*step 3, config spi master*/
    /*     3.1 clear spi fifo content*/
    reg = REG_READ(SPI_STAT);
    while (reg & RXFIFO_RD_READ) {
        REG_READ(SPI_DAT);
        reg = REG_READ(SPI_STAT);
    }

    /*     3.2 disable spi block, and backup */
    reg_icu_pwd = reg = REG_READ(ICU_PERI_CLK_PWD);
    reg |= (PWD_SPI_CLK_BIT);
    REG_WRITE(ICU_PERI_CLK_PWD, reg);

    /*     3.3 clear spi status*/
    REG_WRITE(SPI_CTRL, 0);
    REG_WRITE(SPI_CONFIG, 0);
    reg = REG_READ(SPI_STAT);
    REG_WRITE(SPI_STAT, reg);

    /*     3.4 save the previous setting status*/
    gpio14_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO14*4);
    gpio15_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO15*4);
    gpio16_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO16*4);
    gpio17_status = REG_READ(REG_GPIO_CFG_BASE_ADDR + GPIO17*4);
    gpio14_15_cfg = REG_READ(REG_GPIO_FUNC_CFG);
    gpio16_17_cfg = REG_READ(REG_GPIO_FUNC_CFG_2);

    /*     3.5 set the spi master mode*/
    // open clock, and backup
    reg = reg_icu_clk = REG_READ(ICU_PWM_CLK_MUX);
    reg |= PCLK_POSI_SPI;
    REG_WRITE(ICU_PWM_CLK_MUX, reg);
    // select 26M
    reg = REG_READ(ICU_PERI_CLK_PWD);
    reg &= ~(PWD_SPI_CLK_BIT);
    REG_WRITE(ICU_PERI_CLK_PWD, reg);

    // set to spi config directly
    reg = 0xC00100; // spien  msten  spi_clk=1---13M
    REG_WRITE(SPI_CTRL, reg);

    /*step 4, gpio(14/15/16/17) are set as high-impedance state or input state ,
    			for spi mux with them*/
    gpio_config(GPIO14, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO15, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO16, GMODE_SET_HIGH_IMPENDANCE);
    gpio_config(GPIO17, GMODE_SET_HIGH_IMPENDANCE);

    /*step 5, switch flash interface to spi:sctrl_flash_select_spi_controller()
    	*        Pay attention to prefetch instruction destination, the text can not
    	*        fetch from flash space after this timepoint.
    	*/
    reg = REG_READ(SCTRL_CONTROL);
    reg |= FLASH_SPI_MUX_BIT;
    REG_WRITE(SCTRL_CONTROL, reg);
    while (1) { /* double check the bit:FLASH_SPI_MUX_BIT. The bit cannot be set successfully, if cpu is fetching instruction*/
        reg = REG_READ(SCTRL_CONTROL);
        if (reg & FLASH_SPI_MUX_BIT)
            break;
    }

    if(op_code != NULL)
    {
        /*step 6, write enable for volatile status register: 50H*/
        /*      6.1:take cs*/
        reg = REG_READ(SPI_CONFIG);
        reg &= ~(0xffff << SPI_TX_TRAHS_LEN_POSI);
        reg |= (1 << SPI_TX_TRAHS_LEN_POSI);
        reg |= (SPI_TX_EN | SPI_TX_FINISH_EN);
        REG_WRITE(SPI_CONFIG, reg);

        /*      6.2:write tx fifo*/
        reg = REG_READ(SPI_STAT);
        if (reg & TXFIFO_WR_READ)
            REG_WRITE(SPI_DAT, *op_code);
        else {
            exceptional_flag = -1;
            goto wr_exceptional;
        }

        /*      6.3:waiting for TXFIFO_EMPTY interrupt*/
        while (1) {
            reg = REG_READ(SPI_STAT);
            if (reg & TX_FINISH_INT)
                break;
        }

        /*      6.4:release cs*/
        reg = REG_READ(SPI_CONFIG);
        reg &= ~(0xffff << SPI_TX_TRAHS_LEN_POSI);
        reg &= ~(SPI_TX_EN | SPI_TX_FINISH_EN);
        REG_WRITE(SPI_CONFIG, reg);

        // cler stat and fifo
        reg = REG_READ(SPI_STAT);
        while (reg & RXFIFO_RD_READ) {
            REG_READ(SPI_DAT);
            reg = REG_READ(SPI_STAT);
        }
        reg = REG_READ(SPI_STAT);
        REG_WRITE(SPI_STAT, reg);
    }

    /*step 7, write flash command:01H and sr*/
    /*      7.1:take cs*/
    reg = REG_READ(SPI_CONFIG);
    reg &= ~(0xffffU << SPI_TX_TRAHS_LEN_POSI);
    reg &= ~(0xffffU << SPI_RX_TRAHS_LEN_POSI);
    reg |= ((tx_len & 0xffffU) << SPI_TX_TRAHS_LEN_POSI);
    reg |= (SPI_TX_EN | SPI_TX_FINISH_EN);
    REG_WRITE(SPI_CONFIG, reg);

    /*      7.2:write tx fifo*/
    // write tx first
    for (int i = 0, wait = 0; i < tx_len; ) {
        reg = REG_READ(SPI_STAT);
        if ((reg & TXFIFO_WR_READ) == 0) {
            for(int j=0; j<500; j++);
            wait++;
            if(wait > 100) {
                exceptional_flag = -2;
                goto wr_exceptional;
            }
        } else {
            wait = 0;
            REG_WRITE(SPI_DAT, tx_buf[i]);
            i++;
        }
    }

    /*      7.3:waiting for TXFIFO_EMPTY interrupt*/
    while (1) {
        reg = REG_READ(SPI_STAT);
        if (reg & TX_FINISH_INT)
            break;
    }

    /*      7.4:release cs*/
    reg = REG_READ(SPI_CONFIG);
    reg &= ~(0xffffU << SPI_TX_TRAHS_LEN_POSI);
    reg &= ~(0xffffU << SPI_RX_TRAHS_LEN_POSI);
    reg &= ~(SPI_TX_EN | SPI_RX_EN | SPI_TX_FINISH_EN);
    REG_WRITE(SPI_CONFIG, reg);

    // cler stat and fifo
    reg = REG_READ(SPI_STAT);
    while (reg & RXFIFO_RD_READ) {
        REG_READ(SPI_DAT);
        reg = REG_READ(SPI_STAT);
    }
    reg = REG_READ(SPI_STAT);
    REG_WRITE(SPI_STAT, reg);
    exceptional_flag = 0;

wr_exceptional:
    /*step 8, flash interface select: flash controller-->sctrl_flash_select_flash_controller()*/
    reg = REG_READ(SCTRL_CONTROL);
    reg &= ~(FLASH_SPI_MUX_BIT);
    REG_WRITE(SCTRL_CONTROL, reg);
    // recover icu and powerdown bits for spi
    REG_WRITE(ICU_PWM_CLK_MUX, reg_icu_clk);
    REG_WRITE(ICU_PERI_CLK_PWD, reg_icu_pwd);

    /*step 9, gpio(14/15/16/17) second function*/
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO14*4, gpio14_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO15*4, gpio15_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO16*4, gpio16_status);
    REG_WRITE(REG_GPIO_CFG_BASE_ADDR + GPIO17*4, gpio17_status);
    REG_WRITE(REG_GPIO_FUNC_CFG, gpio14_15_cfg);
    REG_WRITE(REG_GPIO_FUNC_CFG_2, gpio16_17_cfg);

    /*step 10, restore spi register configuration*/
    REG_WRITE(SPI_CTRL, reg_ctrl);
    REG_WRITE(SPI_CONFIG, reg_config);
    REG_WRITE(SPI_STAT, reg_stat);
    REG_WRITE(SPI_DAT, reg_dat);

    /*step 11, enable interrupt*/
    REG_WRITE(ICU_GLOBAL_INT_EN, intc_enable_reg);

    if(LINE_MODE_FOUR == flash_get_line_mode())
    {
        flash_set_line_mode(LINE_MODE_FOUR);
    }

    return exceptional_flag;
}

int flash_bypass_wr_sr_cb(uint32_t larg, uint32_t rarg)
{
    uint8_t sr_width = (uint8_t)larg;
    uint8_t sr_val[3];
    int ret = 0;

    if (0 == flash_is_support_0x50h_cmd() || (sr_width > 2))
        return 0;

    uint8_t op_code = FLASH_CMD_WR_EN_VSR;
    sr_val[0] = FLASH_CMD_WR_SR;
    sr_val[1] = rarg & 0xff;
    sr_val[2] = ((rarg>>8) & 0xff);

    ret = flash_bypass_op_write(&op_code, sr_val, sr_width+1);
    if(ret == 0)// success
    {
        ret = 1;
    }

    return ret;
}

#if CFG_FLASH_BYPASS_OTP
bk_err_t flash_bypass_wait_wip_clear(void);
bk_err_t flash_bypass_id_read(void);

static const flash_bypass_otp_feature_t flash_bypass_otp_feature[] =
{
    {0xC84015,  8, 0, 1024,  256, 3, 0}, //gd_25q16c
    {0xC84016, 12, 1, 1024, 1024, 3, 1}, //gd_25q32c
    {0xC86515, 12, 1, 1024,  256, 1, 0}, //gd_25w16e
    {0xC86516, 12, 1, 1024,  256, 3, 1}, //gd_25wq32e
    {0xEB6015,  8, 0, 1024,  256, 3, 0}, //zg_th25q16b
    {0xCD6014, 12, 1, 1024,  256, 3, 1}, //zg_th25q80HB
    {0xCD7115, 12, 1, 1024,  256, 3, 1}, //zg_th25q16uc
    {0x854215, 12, 1, 1024, 1024, 3, 1}, //py_p25q16
    {0x856015, 12, 1,  512,  256, 3, 1}, //py_p25q16SH(2022)
    {0x852015, 12, 1, 1024,  256, 3, 1}, //py_p25q16HB
    {0x852016, 12, 1, 1024,  256, 3, 1}, //py_p25q32HB
    {0x000000, 12, 1, 1024,  256, 3, 1}, //default
};

uint32_t g_flash_id = 0x0;
static flash_bypass_otp_feature_t otp_feat = {0};

static void flash_bypass_otp_init(void)
{
    if (g_flash_id == 0) {
        flash_bypass_id_read();
    }
}

void flash_bypass_otp_get_current_config(void)
{
    int i;

    for(i = 0; i < (sizeof(flash_bypass_otp_feature) / sizeof(flash_bypass_otp_feature_t) - 1); i++)
    {
        if(g_flash_id == flash_bypass_otp_feature[i].flash_id)
        {
            otp_feat = flash_bypass_otp_feature[i];
            break;
        }
    }

    if(i == (sizeof(flash_bypass_otp_feature) / sizeof(flash_bypass_otp_feature_t) - 1))
    {
        otp_feat = flash_bypass_otp_feature[i];
        os_printf("don't config this flash, choose default flash_bypass_otp config\r\n");
    }
}

static uint32_t otp_idx_to_addr(uint8_t idx)
{
    uint8_t otp_addr_idx_offset = 0;

    otp_addr_idx_offset = otp_feat.otp_addr_idx_offset;

    return (idx << otp_addr_idx_offset);
}

uint32_t flash_bypass_sta_reg_read(void)
{
    bk_err_t ret = BK_OK;
    uint32_t sta_reg = 0;
    bk_err_t ret1 = BK_OK, ret2 = BK_OK;
    uint8_t  retry_cycles = 0;

    uint8_t tx_buf[1]  = {0};
    uint8_t tx_len     = 1;
    uint8_t rx_buf[1]  = {0};
    uint8_t rx_len     = 1;
    uint8_t rx_buf1[1] = {0};
    uint8_t rx_buf2[1] = {0};

    uint8_t sta_reg_read_cmd[] = {
        CMD_FLASH_STA_REG_BIT07_BIT00_READ,
        CMD_FLASH_STA_REG_BIT15_BIT08_READ
    };
    for (uint8_t i = 0; i < sizeof(sta_reg_read_cmd); i++) {
        tx_buf[0] = sta_reg_read_cmd[i];
read_sta_cycle:
        retry_cycles++;
        ret1 = flash_bypass_op_read(tx_buf, tx_len, rx_buf1, rx_len);
        ret2 = flash_bypass_op_read(tx_buf, tx_len, rx_buf2, rx_len);
        if ((ret1 < 0) || (ret2 < 0)) {
            goto read_sta_cycle;
        }
        if (rx_buf1[0] != rx_buf2[0]) {
            goto read_sta_cycle;
        } else {
            rx_buf[0] = rx_buf2[0];
        }
        sta_reg |= (rx_buf[0] << (i * 8));
        if (retry_cycles == 0xff) {
            goto err_exit;
        }
    }
    ret = sta_reg;
    bk_printf("sta_reg = 0x%x\n", sta_reg);
    goto exit;

err_exit:
    bk_printf("%s failed.\n", __func__);
    ret = BK_FAIL;
exit:
    return ret;
}

static bk_err_t flash_bypass_otp_read(flash_bypass_otp_ctrl_t *param, bool printf_on)
{
    bk_err_t ret = BK_OK;
    flash_bypass_otp_ctrl_t *read_cfg = param;
    uint8_t  idx    = read_cfg->otp_idx;
    uint32_t offset = read_cfg->addr_offset;
    uint32_t length = read_cfg->read_len;
    uint32_t addr   = otp_idx_to_addr(idx) + offset;

    uint8_t tx_buf[5] = {0};
    uint8_t tx_len    = 5;
    tx_buf[0] = CMD_FLASH_OTP_READ;
    tx_buf[1] = (addr >> 16) & 0xff;
    tx_buf[2] = (addr >>  8) & 0xff;
    tx_buf[3] = (addr >>  0) & 0xff;
    tx_buf[4] = 0x00;

    uint8_t *rx_buf = (uint8_t *)os_malloc(length);
    if (!rx_buf) {
        goto malloc_err;
    }
    uint32_t rx_len = length;
    uint8_t *err_buf = (uint8_t *)os_malloc(length);
    if (!err_buf) {
        goto malloc_err;
    }
    os_memset(err_buf, 0x00, rx_len);

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }
    for (uint8_t i = 0; i < 3; i++) {
        ret = flash_bypass_op_read(tx_buf, tx_len, rx_buf, rx_len);
        if (ret < 0) {
            goto err_exit;
        }
        ret = flash_bypass_wait_wip_clear();
        if (ret == BK_FAIL) {
            goto err_exit;
        }
        if (os_memcmp(err_buf, rx_buf, rx_len) != 0) {
            break;
        }
    }

    os_memcpy(read_cfg->read_buf, rx_buf, rx_len);

    if (printf_on == 1) {
        for (uint32_t i = 0; i < rx_len; i++) {
            bk_printf("%02x ", rx_buf[i]);
            if ((i + 1) % 16 == 0) {
                bk_printf("\n");
            }
        }
        if (rx_len % 16 != 0) {
            bk_printf("\n");
        }
    }

    ret = BK_OK;
    goto exit;

malloc_err:
    bk_printf("%s malloc failed.\n", __func__);
err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    if (rx_buf) {
        os_free(rx_buf);
    }
    if (err_buf) {
        os_free(err_buf);
    }
    return ret;
}

static bk_err_t flash_bypass_otp_write(flash_bypass_otp_ctrl_t *param)
{
    bk_err_t ret = BK_OK;
    uint8_t op_code = CMD_FLASH_WR_EN;
    flash_bypass_otp_ctrl_t *write_cfg = param;
    uint8_t  idx    = write_cfg->otp_idx;
    uint32_t offset = write_cfg->addr_offset;
    uint32_t length = write_cfg->write_len;
    uint32_t addr   = otp_idx_to_addr(idx) + offset;

    uint32_t tx_len    = 4 + length;
    uint8_t *tx_buf = (uint8_t *)os_malloc(tx_len);
    if (!tx_buf) {
        goto malloc_err;
    }
    tx_buf[0] = CMD_FLASH_OTP_WRITE;
    tx_buf[1] = (addr >> 16) & 0xff;
    tx_buf[2] = (addr >>  8) & 0xff;
    tx_buf[3] = (addr >>  0) & 0xff;

    os_memcpy(tx_buf + 4, write_cfg->write_buf, length);

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }
    ret = flash_bypass_op_write(&op_code, tx_buf, tx_len);
    if (ret < 0) {
        goto err_exit;
    }
    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = BK_OK;
    goto exit;

malloc_err:
    bk_printf("%s malloc failed.\n", __func__);
err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    if (tx_buf) {
        os_free(tx_buf);
    }
    return ret;
}

static bk_err_t flash_bypass_otp_earse(flash_bypass_otp_ctrl_t *param)
{
    bk_err_t ret = BK_OK;
    uint8_t op_code = CMD_FLASH_WR_EN;
    flash_bypass_otp_ctrl_t *earse_cfg = param;
    uint8_t  idx    = earse_cfg->otp_idx;
    uint32_t addr   = otp_idx_to_addr(idx);

    uint8_t tx_buf[4] = {0};
    uint8_t tx_len    = 4;
    tx_buf[0] = CMD_FLASH_OTP_EARSE;
    tx_buf[1] = (addr >> 16) & 0xff;
    tx_buf[2] = (addr >>  8) & 0xff;
    tx_buf[3] = (addr >>  0) & 0xff;

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }
    ret = flash_bypass_op_write(&op_code, tx_buf, tx_len);
    if (ret < 0) {
        goto err_exit;
    }
    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = BK_OK;
    goto exit;

err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    return ret;
}

static bk_err_t flash_bypass_set_otp_lock(flash_bypass_otp_ctrl_t *param)
{
    bk_err_t ret = BK_OK;
    uint8_t op_code = CMD_FLASH_WR_EN;
    flash_bypass_otp_ctrl_t *lock_cfg = param;
    uint8_t  idx    = lock_cfg->otp_idx;

    uint32_t sta_reg  = 0;
    uint8_t tx_buf[3] = {0};
    uint8_t tx_len    = 3;

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = flash_bypass_sta_reg_read();
    if (ret == BK_FAIL) {
        goto err_exit;
    } else {
        sta_reg = ret;
    }

    if (otp_feat.otp_lock_regular_flag) {
        sta_reg |= (1 << (10 + idx));
    } else {
        sta_reg |= (1 << 10);
    }

    tx_buf[0] = CMD_FLASH_STA_REG_BIT07_BIT00_WRITE;
    tx_buf[1] = sta_reg & 0xff;
    tx_buf[2] = (sta_reg >> 8) & 0xff;
    ret = flash_bypass_op_write(&op_code, tx_buf, tx_len);
    if (ret < 0) {
        goto err_exit;
    }

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = BK_OK;
    goto exit;

err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    return ret;
}

bk_err_t flash_bypass_get_otp_lock(flash_bypass_otp_ctrl_t *param)
{
    bk_err_t ret = BK_OK;
    flash_bypass_otp_ctrl_t *lock_cfg = param;
    uint8_t  idx = lock_cfg->otp_idx;

    uint32_t sta_reg  = 0;

    ret = flash_bypass_sta_reg_read();
    if (ret == BK_FAIL) {
        goto err_exit;
    } else {
        sta_reg = ret;
    }

    if (otp_feat.otp_lock_regular_flag) {
        sta_reg |= (1 << (10 + idx));
    } else {
        sta_reg |= (1 << 10);
    }

    if (ret != 0) {
        bk_printf("otp_idx:%d locked.\n", idx);
    } else {
        bk_printf("otp_idx:%d unlocked.\n", idx);
    }

    ret = BK_OK;
    goto exit;

err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    return ret;
}

bk_err_t flash_bypass_otp_operation(flash_bypass_otp_cmd_t cmd, flash_bypass_otp_ctrl_t *param)
{
    bk_err_t ret = BK_OK;
    flash_bypass_otp_ctrl_t *otp_cfg = param;
    uint32_t read_len   = otp_cfg->read_len;
    uint32_t write_len  = otp_cfg->write_len;
    uint32_t offset     = otp_cfg->addr_offset;
    uint32_t otp_idx    = otp_cfg->otp_idx;
    uint32_t op_len_max = 0;
    uint32_t wr_len_max = 0;
    uint32_t idx_max    = 0;
    uint32_t idx_min    = 0;

    GLOBAL_INT_DECLARATION();
    GLOBAL_INT_DISABLE();

    flash_bypass_otp_init();

    op_len_max = otp_feat.op_len_max;
    wr_len_max = otp_feat.wr_len_max;
    idx_max    = otp_feat.idx_max;
    idx_min    = otp_feat.idx_min;

    if ((otp_idx < idx_min) || (otp_idx > idx_max)) {
        bk_printf("[ERROR] otp_idx = %d out of range [%d, %d]\n", otp_idx, idx_min, idx_max);
        return 0;
    }

    switch (cmd) {
    case CMD_OTP_READ:
        if (read_len + offset > op_len_max) {
            bk_printf("otp_read_length out of range %d\n", op_len_max);
            break;
        }
        ret = flash_bypass_otp_read(otp_cfg, 1);
        break;

    case CMD_OTP_EARSE:
        ret = flash_bypass_otp_earse(otp_cfg);
        break;

    case CMD_OTP_WRITE:
        if (write_len + offset > op_len_max) {
            bk_printf("otp_write_length out of range %d\n", op_len_max);
            break;
        }
        for (uint8_t i = 0; i < ((write_len / OTP_WRITE_LEN_MAX) + 1); i++) {
            if ((write_len - (i * OTP_WRITE_LEN_MAX)) <= OTP_WRITE_LEN_MAX) {
                otp_cfg->write_len   = write_len - (i * OTP_WRITE_LEN_MAX);
                otp_cfg->addr_offset = offset + (i * OTP_WRITE_LEN_MAX);
            } else {
                otp_cfg->write_len   = wr_len_max;
                otp_cfg->addr_offset = offset + (i * wr_len_max);
            }
            bk_printf("offset = 0x%x, length = 0x%x\n", otp_cfg->addr_offset, otp_cfg->write_len);
            ret = flash_bypass_otp_write(otp_cfg);
        }
        break;

    case CMD_OTP_SET_LOCK:
        ret = flash_bypass_set_otp_lock(otp_cfg);
        break;

    case CMD_OTP_GET_LOCK:
        ret = flash_bypass_get_otp_lock(otp_cfg);
        break;

    default:
        break;
    }

    GLOBAL_INT_RESTORE();

// exit:
    return ret;
}

bk_err_t flash_bypass_id_read(void)
{
    bk_err_t ret = BK_OK;

    uint8_t tx_buf[1] = {CMD_FLASH_RDID};
    uint8_t tx_len    = 1;
    uint8_t rx_buf[3] = {0};
    uint8_t rx_len    = 3;

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = flash_bypass_op_read(tx_buf, tx_len, rx_buf, rx_len);
    if (ret >= 0) {
        g_flash_id = ((rx_buf[0] << 16) | (rx_buf[1] << 8) | (rx_buf[2] << 0));
        bk_printf("flash_id = 0x%x\n", g_flash_id);
    } else {
        goto err_exit;
    }

    ret = flash_bypass_wait_wip_clear();
    if (ret == BK_FAIL) {
        goto err_exit;
    }

    ret = BK_OK;
    goto exit;

err_exit:
    ret = BK_FAIL;
    bk_printf("%s failed.\n", __func__);
exit:
    return ret;
}

bk_err_t flash_bypass_wait_wip_clear(void)
{
    bk_err_t ret = BK_OK;
    uint32_t sta_reg = 0;
    uint8_t cycles = 0;

    for (cycles = 0; cycles < 0xFF; cycles++) {
        if (g_flash_id == 0x852015) {
            delay_ms(50);
        } else {
            delay_ms(5);
        }
        ret = flash_bypass_sta_reg_read();
        if (ret != BK_FAIL) {
            sta_reg = ret;
        } else {
            goto err_exit;
        }

        if ((sta_reg & FLASH_STA_REG_WIP_BIT) != true) {
            ret = BK_OK;
            goto exit;
        }
    }

err_exit:
    bk_printf("%s failed.\n", __func__);
    ret = BK_FAIL;
exit:
    return ret;
}
#endif
#endif

#if ((CFG_SOC_NAME == SOC_BK7221U) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7231N))
uint32_t flash_bypass_operate_sr_init(void)
{
    flash_register_bypass_cb(flash_bypass_wr_sr_cb);
    return 0;
}
#endif
// eof

