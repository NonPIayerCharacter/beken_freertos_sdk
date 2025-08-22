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
#if CFG_USE_I2C1
#include "arm_arch.h"

#include "i2c1.h"
#include "i2c_pub.h"

#include "intc_pub.h"
#include "icu_pub.h"
#include "gpio_pub.h"
#include "icu.h"

#include "drv_model_pub.h"
#include "mem_pub.h"
#include "common.h"
#include "rtos_pub.h"


static DD_OPERATIONS i2c1_op =
{
    i2c1_open,
    i2c1_close,
    i2c1_read,
    i2c1_write,
    i2c1_ctrl
};

__maybe_unused static void i2c1_set_slave_addr(UINT32 addr);
__maybe_unused static void i2c1_clk_source_set_26M(void);
static volatile I2C1_MSG_ST *gi2c1 ;

static void i2c1_set_idle_cr(UINT32 idle_cr)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    reg_val = (reg_val & ~(I2C1_IDLE_CR_MASK << I2C1_IDLE_CR_POSI))
              | ((idle_cr & I2C1_IDLE_CR_MASK) << I2C1_IDLE_CR_POSI);

    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_scl_cr(UINT32 scl_cr)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    reg_val = (reg_val & ~(I2C1_SCL_CR_MASK << I2C1_SCL_CR_POSI))
              | ((scl_cr & I2C1_SCL_CR_MASK) << I2C1_SCL_CR_POSI);

    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_freq_div(UINT32 div)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    reg_val = (reg_val & ~(I2C1_FREQ_DIV_MASK << I2C1_FREQ_DIV_POSI))
              | ((div & I2C1_FREQ_DIV_MASK) << I2C1_FREQ_DIV_POSI);

    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_slave_addr(UINT32 addr)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    reg_val = (reg_val & ~(I2C1_SLV_ADDR_MASK << I2C1_SLV_ADDR_POSI))
              | ((addr & I2C1_SLV_ADDR_MASK) << I2C1_SLV_ADDR_POSI);
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_smbus_cs(UINT32 cs)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    reg_val = (reg_val & ~(I2C1_SMB_CS_MASK << I2C1_SMB_CS_POSI))
              | ((cs & I2C1_SMB_CS_MASK) << I2C1_SMB_CS_POSI);
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_timeout_en(UINT32 timeout)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    if (timeout)
    {
        reg_val |= I2C1_SMB_TOE;
    }
    else
    {
        reg_val &= ~I2C1_SMB_TOE;
    }
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_free_detect(UINT32 free)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    if (free)
    {
        reg_val |= I2C1_SMB_FTE;
    }
    else
    {
        reg_val &= ~I2C1_SMB_FTE;
    }
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_salve_en(UINT32 en)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    if (en)
    {
        reg_val |= I2C1_INH;
    }
    else
    {
        reg_val &= ~I2C1_INH;
    }
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

static void i2c1_set_smbus_en(UINT32 en)
{
    UINT32 reg_val = REG_READ(REG_I2C1_CONFIG);

    if (en)
    {
        reg_val |= I2C1_ENSMB;
    }
    else
    {
        reg_val &= ~I2C1_ENSMB;
    }
    REG_WRITE(REG_I2C1_CONFIG, reg_val);
}

////////////////////////////////////////////////////////////////////////////////
static void i2c1_clk_source_set_26M(void)
{
    UINT32 param;
    param = PCLK_POSI_I2C1;
    sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, &param);
}

static void i2c1_gpio_config(void)
{
    UINT32 param;

    param = GFUNC_MODE_I2C1;
    sddev_control(GPIO_DEV_NAME, CMD_GPIO_ENABLE_SECOND, &param);
}

static void i2c1_power_up(void)
{
    UINT32 param;

    //set 26M clock
    param = PCLK_POSI_I2C1;
    sddev_control(ICU_DEV_NAME, CMD_CONF_PCLK_26M, &param);

    param = PWD_I2C1_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_UP, &param);
}

static void i2c1_power_down(void)
{
    UINT32 param;
    param = PWD_I2C1_CLK_BIT;
    sddev_control(ICU_DEV_NAME, CMD_CLK_PWR_DOWN, &param);
}

static void i2c1_enable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_I2C1_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_ENABLE, &param);
}

static void i2c1_disable_interrupt(void)
{
    UINT32 param;
    param = (IRQ_I2C1_BIT);
    sddev_control(ICU_DEV_NAME, CMD_ICU_INT_DISABLE, &param);
}

static void i2c1_send_addr(void)
{
    unsigned char tx_data;

    if ((gi2c1->WkMode & I2C1_MSG_WORK_MODE_AL_BIT) == 0)		//7 bit address
    {
        tx_data = 0;
        tx_data |= (gi2c1->SendAddr << 1);
        tx_data &= ~0x01;

        if (gi2c1->WkMode == 0x01)		//master read,without inner address
        {
            tx_data |= 0x01;
        }

        REG_WRITE(REG_I2C1_DAT, tx_data);

        I2C1_PRT("send address:0x%x\r\n", tx_data);

        //modify addr flag for software state machine
        gi2c1->AddrFlag ++;

        if ((gi2c1->WkMode == 0x00) || (gi2c1->WkMode == 0x01))		//master read/write 7bit without inner address
        {
            gi2c1->AddrFlag |= 0x13;

            I2C1_PRT("add_flag=%x\r\n", gi2c1->AddrFlag);
        }
    }
    else								//10 bit address
    {
        tx_data = 0xF0;         		// tx the first address byte with a WRITE in RW bit
        tx_data |= (gi2c1->SendAddr >> 7) & 0x06;
        REG_WRITE(REG_I2C1_DAT, tx_data);

        I2C1_PRT("send address:0x%x\r\n", tx_data);

        gi2c1->AddrFlag ++;
    }
}

static void i2c1_send_start(void)
{
    unsigned int cfg_data;
    unsigned int int_mode;

    if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_MS_BIT)		//slave mode
    {
        return;
    }

    if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_RW_BIT)      // read mode
    {
        if (gi2c1->AllDataNum > 12)
        {
            int_mode = 0x00;
        }
        else if (gi2c1->AllDataNum  > 8)
        {
            int_mode = 0x01;
        }
        else if (gi2c1->AllDataNum  > 4)
        {
            int_mode = 0x02;
        }
        else
        {
            int_mode = 0x03;
        }
    }
    else      										// write mode
    {
        if (gi2c1->AllDataNum < 4 || (gi2c1->WkMode & I2C1_MSG_WORK_MODE_MS_BIT))
        {
            int_mode = 0x00;
        }
        else
        {
            int_mode = 0x01;
        }
    }

    //write address into REG_I2C1_DATA
    i2c1_send_addr();

    // set start bit and mode int
    cfg_data = REG_READ(REG_I2C1_STA);
    cfg_data = cfg_data & 0xFFFBUL;
    cfg_data |= (int_mode << 6);
    cfg_data |= I2C1_SMBUS_STA;

    REG_WRITE(REG_I2C1_STA, cfg_data);
}

static void i2c1_get_message(I2C1_MSG_ST *i2c1_config)
{
    gi2c1 = i2c1_config;

    I2C1_PRT("gi2c1.WkMode = 0x%x\r\n",   gi2c1->WkMode);
    I2C1_PRT("gi2c1.SalveID= 0x%x\r\n",   gi2c1->Slave_addr);
    I2C1_PRT("gi2c1.SendAddr = 0x%x\r\n", gi2c1->SendAddr);
    I2C1_PRT("gi2c1.ack_check = %d\r\n",  gi2c1->ack_check);
    I2C1_PRT("gi2c1.AddrFlag = %d\r\n",   gi2c1->AddrFlag);
    I2C1_PRT("gi2c1.CurrentNum = %d\r\n", gi2c1->CurrentNum);
    I2C1_PRT("gi2c1.InnerAddr = %d\r\n",  gi2c1->InnerAddr);
    I2C1_PRT("gi2c1.TransDone = %d\r\n",  gi2c1->TransDone);
    I2C1_PRT("gi2c1.AllDataNum = %d\r\n", gi2c1->AllDataNum);
}

static UINT8  i2c1_get_busy(void)
{
    UINT32 value;

    value = REG_READ(REG_I2C1_STA);

    if (value & I2C1_BUSY)
    {
        I2C1_DEBUG_PRINTF("busy1: value=0x%x\r\n", value);
        return 1;
    }

    if (gi2c1->TransDone == 0)
    {
        I2C1_DEBUG_PRINTF("busy2:\r\n");
        return 1;
    }
    return 0;
}

static void i2c1_set_io_output(UINT32 enable)
{
    UINT32 value;

    value = REG_READ(REG_I2C1_SMB_EXT_CN);

    if (enable)
    {
        value |= (0x3);
    }
    else
    {
        value &= ~(0x3);
    }

    REG_WRITE(REG_I2C1_SMB_EXT_CN, value);
}

static void i2c1_isr(void)
{
    UINT32 i2c1_stat, i2c1_config;
    UINT32 work_mode, ack, sta, sto, si;
    volatile UINT8 fifo_empty_num = 0;
    volatile UINT8 data_num = 0;
    UINT8 i, uctemp, remain_data_cnt;

    i2c1_stat = REG_READ(REG_I2C1_STA);
    si = i2c1_stat & I2C1_SMBUS_SI;

    I2C1_DEBUG_PRINTF("i2c1_stat=0x%x\r\n", i2c1_stat);
    I2C1_DEBUG_PRINTF("A\r\n");

    if (!si)
    {
        if (i2c1_stat & I2C1_SCL_TIMEOUT)				// SCL low level over time interrupt
        {
            I2C1_EPRT("i\r\n");
            i2c1_set_smbus_en(0);						// must close-->open when SCL Low
            i2c1_set_smbus_en(1);
        }

        if (i2c1_stat & I2C1_ARB_LOST)
        {
            i2c1_stat |= I2C1_ARB_LOST;
            REG_WRITE(REG_I2C1_STA, i2c1_stat);
        }

        return;
    }

    i2c1_config =  REG_READ(REG_I2C1_CONFIG);
    REG_WRITE(REG_I2C1_CONFIG, i2c1_config & (~(I2C1_SMB_CS_MASK << I2C1_SMB_CS_POSI)));			//fix bug

    I2C1_DEBUG_PRINTF("i2c1_config=0x%x\r\n", i2c1_config);

    ack = i2c1_stat & I2C1_SMBUS_ACK;
    sto = i2c1_stat & I2C1_SMBUS_STOP;
    sta = i2c1_stat & I2C1_SMBUS_STA;

    work_mode = gi2c1->WkMode & 0x03;
    remain_data_cnt = gi2c1->AllDataNum - gi2c1->CurrentNum;

    switch (work_mode)
    {
    case 0x00:										//master write
    {
        I2C1_DEBUG_PRINTF("B\r\n");

        i2c1_stat &= ~I2C1_SMBUS_STA;
        ack = REG_READ(REG_I2C1_STA)& I2C1_SMBUS_ACK;

        if (gi2c1->ack_check && !ack)
        {
            i2c1_stat |= I2C1_SMBUS_STOP;			//send stop
            gi2c1->TransDone = 1;

            break;
        }

        uctemp = gi2c1->AddrFlag;

        if (uctemp & 0x10)     // all address bytes has been tx, now tx data
        {
            I2C1_DEBUG_PRINTF("F\r\n");

            if (remain_data_cnt == 0)  // all data bytes has been tx, now send stop
            {
                I2C1_DEBUG_PRINTF("H\r\n");
                i2c1_stat |= I2C1_SMBUS_STOP;		//send stop
                gi2c1->TransDone = 1;

                break;
            }

            switch (i2c1_stat & 0x00C0)
            {
            case 0x00:
                fifo_empty_num = 16;
                break;
            case 0x40:
                fifo_empty_num = 12;
                break;
            case 0x80:
                fifo_empty_num = 8;
                break;
            case 0xC0:
                fifo_empty_num = 4;
                break;
            default  :
                fifo_empty_num = 0;
                break;

            }

            if (remain_data_cnt < fifo_empty_num)
            {

                I2C1_DEBUG_PRINTF("I\r\n");
                data_num = remain_data_cnt;
            }
            else
            {

                I2C1_DEBUG_PRINTF("J\r\n");
                data_num = fifo_empty_num;
            }

            for (i = 0; i < data_num; i++)
            {
                I2C1_DEBUG_PRINTF("K\r\n");

                REG_WRITE(REG_I2C1_DAT, gi2c1->pData[gi2c1->CurrentNum]);
                gi2c1->CurrentNum ++;
                remain_data_cnt --;
            }

            if (remain_data_cnt < fifo_empty_num)
            {

                I2C1_DEBUG_PRINTF("L\r\n");
                i2c1_stat &= ~(I2C1_SMBUS_STA | I2C1_INT_MODE_MASK);
            }
            break;
        }

        if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_AL_BIT)		//10bit address
        {
            I2C1_DEBUG_PRINTF("M\r\n");
            if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_IA_BIT)	//with inner address
            {
                if ((uctemp & 0x08) == 0)
                {
                    if ((uctemp & 0x03) == 0x00)
                    {
                        I2C1_DEBUG_PRINTF("tx error!!!\r\n");
                        break;
                    }
                    else if ((uctemp & 0x03) == 0x01)
                    {

                        I2C1_DEBUG_PRINTF("O\r\n");
                        gi2c1->SendAddr &= 0xFF;
                        REG_WRITE(REG_I2C1_DAT, gi2c1->SendAddr);
                        gi2c1->AddrFlag++;
                    }
                    else
                    {
                        I2C1_DEBUG_PRINTF("P\r\n");
                        REG_WRITE(REG_I2C1_DAT, gi2c1->InnerAddr);
                        gi2c1->AddrFlag |= 0x1B;
                    }
                }
                else
                {
                    gi2c1->AddrFlag |= 0x13;
                }
            }
            else 												//without inner address
            {

                I2C1_DEBUG_PRINTF("N\r\n");
                if ((uctemp & 0x03) == 0x00)
                {
                    I2C1_DEBUG_PRINTF("tx error!!!\r\n");
                    break;
                }
                else if ((uctemp & 0x03) == 0x01)
                {
                    I2C1_DEBUG_PRINTF("O\r\n");
                    gi2c1->SendAddr &= 0xFF;
                    REG_WRITE(REG_I2C1_DAT, gi2c1->SendAddr);
                    gi2c1->AddrFlag |= 0x13;
                }
                else
                {
                    gi2c1->AddrFlag |= 0x13;
                }
            }
        }
        else 												//7 bit address
        {

            I2C1_DEBUG_PRINTF("M'\r\n");
            if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_IA_BIT)	//with inner address
            {

                I2C1_DEBUG_PRINTF("N\r\n");

                if ((uctemp & 0x08) == 0)
                {
                    I2C1_DEBUG_PRINTF("O\r\n");
                    REG_WRITE(REG_I2C1_DAT, gi2c1->InnerAddr); 	//write inner address

                    gi2c1->AddrFlag |= 0x13;
                }
                else
                {
                    I2C1_DEBUG_PRINTF("P\r\n");
                    gi2c1->AddrFlag |= 0x13;
                }
            }
            else
            {
                gi2c1->AddrFlag |= 0x13;
            }
        }
        break;
    }

    case 0x01:												  //master read
    {
        i2c1_stat &= ~I2C1_SMBUS_STA;
        I2C1_DEBUG_PRINTF("C\r\n");

        if (sta && gi2c1->ack_check && !ack)
        {
            i2c1_stat = i2c1_stat | I2C1_SMBUS_STOP;			//send stop
            gi2c1->TransDone = 1;

            break;
        }

        uctemp = gi2c1->AddrFlag;

        if (uctemp & 0x10)      									// all address bytes has been tx, now tx data
        {
            I2C1_DEBUG_PRINTF("D\r\n");

            if (sta)
            {
                i2c1_stat = i2c1_stat | I2C1_SMBUS_ACK; 		//sen ack
                break;
            }

            switch (i2c1_stat & 0x00C0)
            {
            case 0x00:
                data_num = 12;
                break;
            case 0x40:
                data_num = 8;
                break;
            case 0x80:
                data_num = 4;
                break;
            case 0xC0:
                data_num = 1;
                break;
            default  :
                data_num = 0;
                break;
            }

            for (i = 0; i < data_num; i++)
            {
                I2C1_DEBUG_PRINTF("E\r\n");

                gi2c1->pData[gi2c1->CurrentNum] = REG_READ(REG_I2C1_DAT);
                gi2c1->CurrentNum ++;
                remain_data_cnt --;
            }

            if (remain_data_cnt == 0)  // all data bytes has been tx, now send stop
            {
                I2C1_DEBUG_PRINTF("F\r\n");

                i2c1_stat = (i2c1_stat & (~(I2C1_SMBUS_ACK | I2C1_SMBUS_STA))) | I2C1_SMBUS_STOP ;		//send no ack/stop
                gi2c1->TransDone = 1;

            }
            else if (remain_data_cnt < data_num)
            {
                i2c1_stat = i2c1_stat | (I2C1_SMBUS_ACK | I2C1_INT_MODE_MASK);
            }
            else
            {
                i2c1_stat = i2c1_stat | I2C1_SMBUS_ACK ;
            }
            break;
        }
        if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_AL_BIT)		//10bit address
        {

            I2C1_DEBUG_PRINTF("G\r\n");

            if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_IA_BIT)	//with inner address
            {
                I2C1_DEBUG_PRINTF("F\r\n");

                if ((uctemp & 0x08) == 0)
                {
                    if ((uctemp & 0x03) == 0x00)
                    {
                        I2C1_DEBUG_PRINTF("tx error!!!\r\n");

                        break;
                    }
                    else if ((uctemp & 0x03) == 0x01)
                    {
                        REG_WRITE(REG_I2C1_DAT, gi2c1->SendAddr & 0xFF);
                        gi2c1->AddrFlag++;
                    }
                    else
                    {
                        REG_WRITE(REG_I2C1_DAT, gi2c1->InnerAddr);
                        gi2c1->AddrFlag |= 0x08;
                    }
                }
                else
                {
                    if ((uctemp & 0x03) == 0x02)
                    {
                        i2c1_stat |= I2C1_SMBUS_STA;
                        REG_WRITE(REG_I2C1_DAT, ((gi2c1->SendAddr >> 7) & 0x06) | 0xF1);
                        gi2c1->AddrFlag |= 0x13;
                    }
                    else
                    {
                        I2C1_DEBUG_PRINTF("read error1!!!\r\n");
                        break;
                    }
                }
            }
            else 												//without inner address
            {

                if ((uctemp & 0x03) == 0x00)
                {
                    I2C1_DEBUG_PRINTF("read error1!!!\r\n");
                }
                else if ((uctemp & 0x03) == 0x01)
                {
                    REG_WRITE(REG_I2C1_DAT, gi2c1->SendAddr & 0xFF);

                    gi2c1->AddrFlag ++;
                }
                else if ((uctemp & 0x03) == 0x02)
                {
                    i2c1_stat |= I2C1_SMBUS_STA;
                    REG_WRITE(REG_I2C1_DAT, ((gi2c1->SendAddr >> 7) & 0x06) | 0xF1);

                    gi2c1->AddrFlag |= 0x13;
                }
                else
                {
                    I2C1_DEBUG_PRINTF("read error1!!!\r\n");
                    break;
                }

            }
        }
        else												// 7 bite address
        {
            I2C1_DEBUG_PRINTF("H\r\n");

            if (gi2c1->WkMode & I2C1_MSG_WORK_MODE_IA_BIT)	//with inner address
            {
                I2C1_DEBUG_PRINTF("D\r\n");

                if ((uctemp & 0x08) == 0)
                {
                    REG_WRITE(REG_I2C1_DAT, gi2c1->InnerAddr); 	//write inner address

                    gi2c1->AddrFlag = (gi2c1->AddrFlag & ~0x0B) | 0x0A;
                }
                else
                {
                    i2c1_stat |= I2C1_SMBUS_STA;
                    REG_WRITE(REG_I2C1_DAT, (gi2c1->SendAddr << 1) | 0x01);

                    gi2c1->AddrFlag |= 0x13;
                }
            }
            else 												//without inner address
            {
                gi2c1->AddrFlag |= 0x13;
            }

        }
        break;

    }
    case 0x02:      // slave write
    {
        I2C1_DEBUG_PRINTF("d\r\n");

        if (sto)    // detect a STOP
        {

            I2C1_DEBUG_PRINTF("k\r\n");

            gi2c1->TransDone = 1;
            break;
        }


        if (i2c1_stat & I2C1_ADDR_MATCH)      // match address byte
        {

            I2C1_DEBUG_PRINTF("l\r\n");

            i2c1_stat |= I2C1_SMBUS_ACK;     // send ACK for address byte
        }

        if ((i2c1_stat & I2C1_TX_MODE) == 0)      // read mode
        {
            break;
        }


        if (i2c1_stat & I2C1_SMBUS_ACK)      // detect an ACK
        {

            I2C1_DEBUG_PRINTF("i\r\n");

            REG_WRITE(REG_I2C1_DAT, gi2c1->pData[gi2c1->CurrentNum]);
            gi2c1->CurrentNum ++;
            remain_data_cnt --;

            if (remain_data_cnt == 0)
            {
                I2C1_DEBUG_PRINTF("j\r\n");

                // TODO
            }
        }
        break;
    }
    case 0x03:											// slave read
    {
        I2C1_DEBUG_PRINTF("e\r\n");

        if (sto)										// detect a STOP
        {
            gi2c1->TransDone = 1;
            break;
        }

        if (i2c1_stat & I2C1_ADDR_MATCH)	  // match address byte
        {

            i2c1_stat |= I2C1_SMBUS_ACK;	   // send ACK
            break;
        }

        switch (i2c1_stat & I2C1_INT_MODE_MASK)
        {
        case 0x0000:
            data_num = 12;
            break;
        case 0x0040:
            data_num = 8;
            break;
        case 0x0080:
            data_num = 4;
            break;
        case 0x00C0:
            data_num = 1;
            break;
        default    :
            data_num = 0;
            break;
        }

        for (i = 0; i < data_num; i ++)
        {

            gi2c1->pData[gi2c1->CurrentNum] = REG_READ(REG_I2C1_DAT);
            gi2c1->CurrentNum ++;
            remain_data_cnt --;
        }

        if (remain_data_cnt == 0)
        {

            i2c1_stat &= ~I2C1_SMBUS_ACK; 	// send NACK
            gi2c1->TransDone = 1;
        }
        else if (remain_data_cnt < data_num)
        {

            // send ACK, set int_mode
            i2c1_stat = i2c1_stat | I2C1_SMBUS_ACK | I2C1_INT_MODE_MASK;
        }
        else
        {

            i2c1_stat |= I2C1_SMBUS_ACK;	   // send ACK
        }
        break;
    }

    default:
        break;
    }

    REG_WRITE(REG_I2C1_CONFIG, i2c1_config);
    REG_WRITE(REG_I2C1_STA, i2c1_stat & (~I2C1_SMBUS_SI)); 		//clear si

}

static void i2c1_software_init(void)
{
    ddev_register_dev(I2C1_DEV_NAME, &i2c1_op);
}

static void i2c1_hardware_init(void)
{
    //UINT32 reg;
    /* register interrupt */
    intc_service_register(IRQ_I2C1, PRI_IRQ_I2C1, i2c1_isr);

    /* clear all setting */
    REG_WRITE(REG_I2C1_CONFIG, 0);
    REG_WRITE(REG_I2C1_STA, 0);
}

void i2c1_init(void)
{
    gi2c1 = os_malloc(sizeof(I2C1_MSG_ST));
    os_memset((unsigned char *)gi2c1, 0, sizeof(I2C1_MSG_ST));

    i2c1_software_init();
    i2c1_hardware_init();
}

void i2c1_exit(void)
{
    /* clear all setting */
    REG_WRITE(REG_I2C1_CONFIG, 0);
    REG_WRITE(REG_I2C1_STA, 0);

    ddev_unregister_dev(I2C1_DEV_NAME);
}

static UINT32 i2c1_open(UINT32 op_flag)
{
    UINT32 reg, reg1;

    if(op_flag == 0)
    {
        op_flag = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)// master
                   & (~I2C2_MSG_WORK_MODE_AL_BIT))// 7bit address
                  | (I2C2_MSG_WORK_MODE_IA_BIT); // wi
    }

    reg = REG_READ(REG_I2C1_CONFIG);
    reg1 = REG_READ(REG_I2C1_STA);

    if (reg & I2C1_ENSMB)
    {
        reg1 |=  I2C1_SMBUS_STOP;
        reg   = 0;
    }

    REG_WRITE(REG_I2C1_CONFIG, reg);
    REG_WRITE(REG_I2C1_STA, reg1);

    if (op_flag >> 4)
    {
        i2c1_set_freq_div(op_flag);
    }
    else
    {
        I2C1_PRT("i2c1 set default baud_rate:%d\r\n", I2C_DEFAULT_BAUD);
        i2c1_set_freq_div(I2C_CLK_DIVID(I2C_DEFAULT_BAUD));
    }

    i2c1_set_idle_cr(0x3);
    i2c1_set_scl_cr(0x4);
    i2c1_set_smbus_cs(0x3);
    i2c1_set_timeout_en(1);
    i2c1_set_free_detect(1);
    i2c1_set_salve_en(0);  // enable/disable i2c slave

    i2c1_enable_interrupt();
    i2c1_power_up();
    i2c1_gpio_config();

    i2c1_set_smbus_en(1);

    reg = REG_READ(REG_I2C1_CONFIG);

    I2C1_DEBUG_PRINTF("REG_I2C1_CONFIG=0x%lx\r\n", reg);

    gi2c1->WkMode = op_flag & 0x0F;

    return I2C1_SUCCESS;
}

static UINT32 i2c1_close(void)
{
    i2c1_set_smbus_en(0);

    i2c1_disable_interrupt();
    i2c1_power_down();

    return I2C1_SUCCESS;
}

static UINT32 i2c1_read(char *user_buf, UINT32 count, UINT32 op_flag)
{
    UINT32 reg, start_tick, cur_tick;
    I2C_OP_PTR i2c_op;
    GLOBAL_INT_DECLARATION();

    i2c_op = (I2C_OP_PTR)op_flag;

    if (gi2c1->TransDone != 0)
    {
        return 0;
    }

    GLOBAL_INT_DISABLE();
    gi2c1->AddrFlag   = 0;
    gi2c1->TransDone  = 0;
    gi2c1->ack_check  = 1;
    gi2c1->CurrentNum = 0;
    gi2c1->AllDataNum = count;
    gi2c1->pData      = (UINT8 *)user_buf;
    gi2c1->InnerAddr  = i2c_op->op_addr;
    gi2c1->SendAddr   = i2c_op->salve_id;
    gi2c1->Slave_addr = i2c_op->slave_addr;
    gi2c1->WkMode     |= (I2C1_MSG_WORK_MODE_RW_BIT);   //READ
    gi2c1->ErrorNO = 0;


    I2C1_PRT("gi2c1.WkMode = 0x%x\r\n",   gi2c1->WkMode);
    I2C1_PRT("gi2c1.SalveID= 0x%x\r\n",   gi2c1->Slave_addr);
    I2C1_PRT("gi2c1.SendAddr = 0x%x\r\n", gi2c1->SendAddr);
    I2C1_PRT("gi2c1.ack_check = %d\r\n",  gi2c1->ack_check);
    I2C1_PRT("gi2c1.AddrFlag = %d\r\n",   gi2c1->AddrFlag);
    I2C1_PRT("gi2c1.CurrentNum = %d\r\n", gi2c1->CurrentNum);
    I2C1_PRT("gi2c1.InnerAddr = %d\r\n",  gi2c1->InnerAddr);
    I2C1_PRT("gi2c1.TransDone = %d\r\n",  gi2c1->TransDone);
    I2C1_PRT("gi2c1.AllDataNum = %d\r\n", gi2c1->AllDataNum);

    //write salve address
    reg = REG_READ(REG_I2C1_CONFIG);
    reg &= (~(0x3FF << I2C1_SLV_ADDR_POSI));
    reg = reg | (((gi2c1->Slave_addr) & 0x3FF) << I2C1_SLV_ADDR_POSI);
    REG_WRITE(REG_I2C1_CONFIG, reg);

    reg = REG_READ(REG_I2C1_CONFIG);
    I2C1_PRT("i2c1_config= %lx\r\n", reg);

    reg = REG_READ(REG_I2C1_STA);
    I2C1_PRT("i2c1_stat= %lx\r\n", reg);

    i2c1_send_start();
    GLOBAL_INT_RESTORE();

    start_tick = rtos_get_time();
    while(1)
    {
        if (i2c1_get_busy() == 1)
        {
            UINT32 past = 0;
            cur_tick = rtos_get_time();
            past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
            if ((past) > I2C_READ_WAIT_TIMEOUT)
            {
                gi2c1->ErrorNO = 1;
                break;
            }
            //rtos_delay_milliseconds(10);
        }
        else
        {
            break;
        }
    }

    GLOBAL_INT_DISABLE();
    gi2c1->TransDone = 0;
    GLOBAL_INT_RESTORE();

    return gi2c1->ErrorNO;
}

static UINT32 i2c1_write(char *user_buf, UINT32 count, UINT32 op_flag)
{
    UINT32 start_tick, cur_tick;
    I2C_OP_PTR i2c_op;
    GLOBAL_INT_DECLARATION();

    if (gi2c1->TransDone != 0)
    {
        return 0;
    }

    GLOBAL_INT_DISABLE();
    i2c_op = (I2C_OP_PTR)op_flag;

    //os_printf("i2c1_write\r\n");
    gi2c1->AddrFlag   = 0;
    gi2c1->TransDone  = 0;
    gi2c1->ack_check  = 1;
    gi2c1->CurrentNum = 0;
    gi2c1->AllDataNum = count;
    gi2c1->pData      = (UINT8 *)user_buf;
    gi2c1->InnerAddr  = i2c_op->op_addr;
    gi2c1->SendAddr   = i2c_op->salve_id;
    gi2c1->Slave_addr = i2c_op->slave_addr;
    gi2c1->WkMode     &= (~I2C1_MSG_WORK_MODE_RW_BIT);   //WRITE
    gi2c1->ErrorNO    = 0;

    I2C1_PRT("gi2c1.WkMode = 0x%x\r\n",   gi2c1->WkMode);
    I2C1_PRT("gi2c1.SalveID= 0x%x\r\n",   gi2c1->Slave_addr);
    I2C1_PRT("gi2c1.SendAddr = 0x%x\r\n", gi2c1->SendAddr);
    I2C1_PRT("gi2c1.ack_check = %d\r\n",  gi2c1->ack_check);
    I2C1_PRT("gi2c1.AddrFlag = %d\r\n",   gi2c1->AddrFlag);
    I2C1_PRT("gi2c1.CurrentNum = %d\r\n", gi2c1->CurrentNum);
    I2C1_PRT("gi2c1.InnerAddr = %d\r\n",  gi2c1->InnerAddr);
    I2C1_PRT("gi2c1.TransDone = %d\r\n",  gi2c1->TransDone);
    I2C1_PRT("gi2c1.AllDataNum = %d\r\n", gi2c1->AllDataNum);

    // salve address---as slave
    i2c1_set_slave_addr(gi2c1->Slave_addr);

    i2c1_send_start();
    GLOBAL_INT_RESTORE();

    start_tick = rtos_get_time();
    while(1)
    {
        if (i2c1_get_busy() == 1)
        {
            UINT32 past = 0;
            cur_tick = rtos_get_time();
            past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
            if ((past) > I2C_WRITE_WAIT_TIMEOUT)
            {
                gi2c1->ErrorNO = 1;
                break;
            }
            //rtos_delay_milliseconds(10);
        }
        else
        {
            break;
        }
    }

    GLOBAL_INT_DISABLE();
    gi2c1->TransDone = 0;
    GLOBAL_INT_RESTORE();

    return gi2c1->ErrorNO;
}

static UINT32 i2c1_ctrl(UINT32 cmd, void *param)
{
    UINT32 ret = I2C1_SUCCESS;

    switch (cmd)
    {
    case I2C1_CMD_SET_IDLE_CR:
        i2c1_set_idle_cr(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_SCL_CR:
        i2c1_set_scl_cr(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_FREQ_DIV:
        i2c1_set_freq_div(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_SMBUS_CS:
        i2c1_set_smbus_cs(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_TIMEOUT_EN:
        i2c1_set_timeout_en(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_FREE_DETECT:
        i2c1_set_free_detect(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_SLAVE_EN:
        i2c1_set_salve_en(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_SMBUS_EN:
        i2c1_set_smbus_en(*((UINT32 *)param));
        break;
    case I2C1_CMD_SET_POWER_UP:
        i2c1_power_up();
        break;
    case I2C1_CMD_SET_POWER_DOWN:
        i2c1_power_down();
        break;
    case I2C1_CMD_GET_MESSAGE:
        i2c1_get_message(param);
        break;
    case I2C1_CMD_SET_IO_OUTPUT:
        i2c1_set_io_output(*((UINT32 *)param));
        break;
    default:
        ret = I2C1_FAILURE;
        break;
    }

    return ret;
}

#endif

