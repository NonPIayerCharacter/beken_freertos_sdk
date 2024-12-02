#include "include.h"
#include "arm_arch.h"
#include "sys_config.h"
#include "flash.h"
#include "sys_ctrl.h"
#include "flash_pub.h"
#include "drv_model_pub.h"
#include <string.h>
#include "uart_pub.h"
#include "sys_ctrl_pub.h"
#include "mcu_ps_pub.h"
#include "mem_pub.h"
#include "ate_app.h"
#include "flash_bypass.h"

static const flash_config_t flash_config[] =
{
    {0x1C7016, 1, 0x400000, 2,  0, 2, 0x1F, 0x1F, 0x00, 0x16, 0x01B, 0, 0, 0xA5, 0x01}, //en_25qh32b
    {0x1C7015, 1, 0x200000, 2,  0, 2, 0x1F, 0x1F, 0x00, 0x0d, 0x0d,  0, 0, 0xA5, 0x01}, //en_25qh16b
    {0x1C4116, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x101, 9, 1, 0xA5, 0x01}, //en_25qe32a(2P)
    {0x1C6116, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x101, 9, 1, 0xA5, 0x01}, //en_25qw32a(2P)
    {0x0B4014, 2, 0x100000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0C, 0x101, 9, 1, 0xA0, 0x01}, //xtx_25f08b
    {0x0B4015, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //xtx_25f16b
    {0x0B4016, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x101, 9, 1, 0xA0, 0x01}, //xtx_25f32b
    {0x0B4017, 2, 0x800000, 2, 14, 2, 0x1F, 0x05, 0x00, 0x0E, 0x109, 9, 1, 0xA0, 0x01}, //xtx_25f64b
    {0x0E4016, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x101, 9, 1, 0xA0, 0x01}, //xtx_FT25H32
    {0xC84015, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //gd_25q16c
    {0xC84016, 1, 0x400000, 2,  0, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x00E, 0, 0, 0xA0, 0x01}, //gd_25q32c
    {0xC86515, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //gd_25w16e
    {0xEF4016, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x00, 0x101, 9, 1, 0xA0, 0x01}, //w_25q32(bfj)
    {0x204016, 2, 0x400000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0E, 0x101, 9, 1, 0xA0, 0x01}, //xmc_25qh32b
    {0xC22315, 1, 0x200000, 2,  0, 2, 0x0F, 0x0F, 0x00, 0x0A, 0x00E, 6, 1, 0xA5, 0x01}, //mx_25v16b
    {0xEB6015, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //zg_th25q16b
    {0xCD6014, 2, 0x100000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0C, 0x101, 9, 1, 0xA0, 0x01}, //zg_th25q80HB
    {0x854215, 1, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0C, 0x101, 9, 1, 0xA0, 0x01}, //py_p25q16
    {0x856015, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //py_p25q16SH(2022)
    {0x852015, 2, 0x200000, 2, 14, 2, 0x1F, 0x1F, 0x00, 0x0D, 0x101, 9, 1, 0xA0, 0x01}, //py_p25q16HB
    {0x000000, 2, 0x400000, 2,  0, 2, 0x1F, 0x00, 0x00, 0x00, 0x000, 0, 0, 0x00, 0x01}, //default
};

static const flash_config_t *flash_current_config = NULL;
static FUNC_2PARAM_CB flash_wr_sr_bypass_method_cd = NULL;
static UINT32 flash_id;
static DD_OPERATIONS flash_op =
{
    NULL,
    NULL,
    flash_read,
    flash_write,
    flash_ctrl
};

static void flash_get_current_flash_config(void)
{
    int i;
	
    for(i = 0; i < (sizeof(flash_config) / sizeof(flash_config_t) - 1); i++)
    {
        if(flash_id == flash_config[i].flash_id)
        {
            flash_current_config = &flash_config[i];
            break;
        }
    }
	
    if(i == (sizeof(flash_config) / sizeof(flash_config_t) - 1))
    {
        flash_current_config = &flash_config[i];
        os_printf("don't config this flash, choose default config\r\n");
    }
}

void flash_disable_crc(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(CRC_EN);
	
    REG_WRITE(REG_FLASH_CONF, value);
}

void flash_enable_crc(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value |= CRC_EN;
	
    REG_WRITE(REG_FLASH_CONF, value);
}

void flash_set_clk(UINT8 clk_conf)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(FLASH_CLK_CONF_MASK << FLASH_CLK_CONF_POSI);
    value |= (clk_conf << FLASH_CLK_CONF_POSI);
	
    REG_WRITE(REG_FLASH_CONF, value);
}

__maybe_unused static void flash_enable_cpu_data_wr(void);
static void flash_enable_cpu_data_wr(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value |= (CPU_DATA_WR_MASK << CPU_DATA_WR_POSI);
    REG_WRITE(REG_FLASH_CONF, value);
}

void flash_disable_cpu_data_wr(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value &= (~(CPU_DATA_WR_MASK << CPU_DATA_WR_POSI));
	
    REG_WRITE(REG_FLASH_CONF, value);
}

static void flash_write_enable(void)
{
    UINT32 value;

    value = (FLASH_OPCODE_WREN << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    REG_WRITE(REG_FLASH_OPERATE_SW, value);

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

static void flash_write_disable(void)
{
    UINT32 value;

    value = (FLASH_OPCODE_WRDI << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    REG_WRITE(REG_FLASH_OPERATE_SW, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

static UINT16 flash_read_sr(UINT8 sr_width)
{
	UINT16 sr;
    UINT32 value;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = (FLASH_OPCODE_RDSR << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    REG_WRITE(REG_FLASH_OPERATE_SW, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
    sr = value & 0x00FF;

	if(sr_width == 2)
	{
	    value = (FLASH_OPCODE_RDSR2 << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
	    REG_WRITE(REG_FLASH_OPERATE_SW, value);
	    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

	    value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
	    sr |= (value & 0x00FF) << 8;
	}

    return sr;
}

static void flash_write_sr(UINT8 sr_width,  UINT16 val)
{
    UINT32 value;

	if(flash_wr_sr_bypass_method_cd)
	{
		int ret = flash_wr_sr_bypass_method_cd((uint32_t)sr_width, (uint32_t)val);
		if(ret == 1)
		{
			// if return 1, means writen sr by volatile successed
			return;
		}
	}

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DECLARATION();

    GLOBAL_INT_DISABLE();
#endif
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(WRSR_DATA_MASK << WRSR_DATA_POSI);

    value |= (val << WRSR_DATA_POSI);
    
    REG_WRITE(REG_FLASH_CONF, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    if(sr_width == 1)
    {
        value = (FLASH_OPCODE_WRSR << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
        REG_WRITE(REG_FLASH_OPERATE_SW, value);
    }
    else if(sr_width == 2)
    {
        value = (FLASH_OPCODE_WRSR2 << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
        REG_WRITE(REG_FLASH_OPERATE_SW, value);
    }

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_RESTORE();
#endif
}

static UINT8 flash_read_qe(void)
{
    UINT8 temp;
    UINT32 value;

    if(1 == flash_current_config->sr_size)
    {
        value = (FLASH_OPCODE_RDSR << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
        REG_WRITE(REG_FLASH_OPERATE_SW, value);
    }
    else
    {
        value = (FLASH_OPCODE_RDSR2 << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
        REG_WRITE(REG_FLASH_OPERATE_SW, value);
    }

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
    temp = (value & 0xFF);
    return temp;
}

static void flash_set_qe(void)
{
    UINT32 value, param;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    param = flash_read_sr(flash_current_config->sr_size);
    if((param & (flash_current_config->qe_bit << flash_current_config->qe_bit_post)))
    {
        return ;
    }
    value = REG_READ(REG_FLASH_CONF);
    value &= ~(WRSR_DATA_MASK << WRSR_DATA_POSI);
    value |= (((flash_current_config->qe_bit << flash_current_config->qe_bit_post)
        | param) << WRSR_DATA_POSI);
    REG_WRITE(REG_FLASH_CONF, value);

    value = REG_READ(REG_FLASH_OPERATE_SW);
    
    if(1 == flash_current_config->sr_size)
    {
        value = (value & (ADDR_SW_REG_MASK << ADDR_SW_REG_POSI))
                | (FLASH_OPCODE_WRSR << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    }
    else
    {
        value = (value & (ADDR_SW_REG_MASK << ADDR_SW_REG_POSI))
                | (FLASH_OPCODE_WRSR2 << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    }

    REG_WRITE(REG_FLASH_OPERATE_SW, value);

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

static void flash_set_qwfr(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(MODEL_SEL_MASK << MODEL_SEL_POSI);
    value |= (flash_current_config->mode_sel << MODEL_SEL_POSI);
    REG_WRITE(REG_FLASH_CONF, value);
}

static void flash_clr_qwfr(void)
{
    UINT32 value;

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(MODEL_SEL_MASK << MODEL_SEL_POSI);
    REG_WRITE(REG_FLASH_CONF, value);

    value = REG_READ(REG_FLASH_OPERATE_SW);
    value = ((0 << ADDR_SW_REG_POSI)
             | (FLASH_OPCODE_CRMR << OP_TYPE_SW_POSI)
             | OP_SW
             | (value & WP_VALUE));
    REG_WRITE(REG_FLASH_OPERATE_SW, value);

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

static void flash_set_wsr(UINT16 data)
{
    UINT32 value;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = REG_READ(REG_FLASH_CONF);
    value &= ~(WRSR_DATA_MASK << WRSR_DATA_POSI);
    value |= (data << WRSR_DATA_POSI);
    REG_WRITE(REG_FLASH_CONF, value);

    value = REG_READ(REG_FLASH_OPERATE_SW);
    value = (value & (ADDR_SW_REG_MASK << ADDR_SW_REG_POSI))
            | (FLASH_OPCODE_WRSR2 << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    REG_WRITE(REG_FLASH_OPERATE_SW, value);

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

UINT8 flash_support_wide_voltage(void)
{
    return (flash_id == 0xC86515);
}

UINT8 flash_get_line_mode(void)
{
    return flash_current_config->line_mode;
}

void flash_set_line_mode(UINT8 mode)
{
    UINT32 value;
    
    if(1 == mode)
    {
        flash_clr_qwfr();
    }
    if(2 == mode)
    {
        flash_clr_qwfr();
        value = REG_READ(REG_FLASH_CONF);
        value &= ~(MODEL_SEL_MASK << MODEL_SEL_POSI);
        value |= ((MODE_DUAL & MODEL_SEL_MASK) << MODEL_SEL_POSI);
        REG_WRITE(REG_FLASH_CONF, value);
    }
    else if(4 == mode)
    {
        flash_clr_qwfr();
        value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
        value &= ~(M_VALUE_MASK << M_VALUE_POST);
        value |= (flash_current_config->m_value<< M_VALUE_POST);
        
        REG_WRITE(REG_FLASH_SR_DATA_CRC_CNT, value);

        value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);

        if(1 == flash_current_config->qe_bit)
        {
            flash_set_qe();
        }

        flash_set_qwfr();
    }
}

UINT32 flash_get_id(void)
{
    UINT32 value;

    value = (FLASH_OPCODE_RDID << OP_TYPE_SW_POSI) | OP_SW | WP_VALUE;
    REG_WRITE(REG_FLASH_OPERATE_SW, value);

    while (REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    flash_id = REG_READ(REG_FLASH_RDID_DATA_FLASH);
    return flash_id;
}

static UINT32 flash_get_size(void)
{
	if (NULL == flash_current_config)
	{
		return 0x200000;
	}

	return flash_current_config->flash_size;
}

UINT32 flash_is_xtx_type(void)
{
	if((0x0B4014 == flash_id) || (0x0B4015 == flash_id)
			|| (0x0B4016 == flash_id)
			|| (0x0B4017 == flash_id)
			|| (0x0E4016 == flash_id))
	{
		return 1;
	}
	// puya flash
	else if((0x854215 == flash_id) || (0x856015 == flash_id) || (0x852015 == flash_id))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

UINT32 flash_is_support_0x50h_cmd(void)
{
	return flash_is_xtx_type();
}

UINT32 flash_register_bypass_cb(FUNC_2PARAM_CB cb)
{
	flash_wr_sr_bypass_method_cd = cb;

	return 0;
}

static UINT32 flash_read_mid(void)
{
    UINT32 value;
    UINT32 flash_id;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    value = REG_READ(REG_FLASH_OPERATE_SW);
    value = ((value & (ADDR_SW_REG_MASK << ADDR_SW_REG_POSI))
             | (FLASH_OPCODE_RDID << OP_TYPE_SW_POSI)
             | OP_SW
             | (value & WP_VALUE));
    REG_WRITE(REG_FLASH_OPERATE_SW, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);

    flash_id = REG_READ(REG_FLASH_RDID_DATA_FLASH);

    return flash_id;
}

PROTECT_TYPE get_flash_protect(void)
{
	UINT16 sr_value, cmp, param, value, type;

	sr_value = flash_read_sr(flash_current_config->sr_size);
	param = (sr_value >> flash_current_config->protect_post) & flash_current_config->protect_mask;
	cmp = (sr_value >> flash_current_config->cmp_post) & 0x01;
	value = (cmp << 8) | param;
	
	if(value == flash_current_config->protect_all)
	{
		type = FLASH_PROTECT_ALL;
	}
	else if(value == flash_current_config->protect_none)
	{
		type = FLASH_PROTECT_NONE;
	}
	else if(value == flash_current_config->protect_half)
	{
		type = FLASH_PROTECT_HALF;
	}
	else if(value == flash_current_config->unprotect_last_block)
	{
		type = FLASH_UNPROTECT_LAST_BLOCK;
	}
	else
	{
		type = -1;
	}

	return type;
}

static void set_flash_protect(PROTECT_TYPE type)
{
    UINT32 param, value, cmp;
	
	switch (type)
	{
		case FLASH_PROTECT_NONE:
            param = flash_current_config->protect_none & 0xff;
            cmp = (flash_current_config->protect_none >> 8) & 0xff;
            break;
            
		case FLASH_PROTECT_ALL:
			param = flash_current_config->protect_all & 0xff;
			cmp = (flash_current_config->protect_all >> 8) & 0xff;
			break;

        case FLASH_PROTECT_HALF:
			param = flash_current_config->protect_half & 0xff;
			cmp = (flash_current_config->protect_half >> 8) & 0xff;
			break;

        case FLASH_UNPROTECT_LAST_BLOCK:
			param = flash_current_config->unprotect_last_block& 0xff;
			cmp = (flash_current_config->unprotect_last_block >> 8) & 0xff;
			break;
			
		default:
			param = flash_current_config->protect_all & 0xff;
            cmp = (flash_current_config->protect_all >> 8) & 0xff;
			break;
	}
    
    value = flash_read_sr(flash_current_config->sr_size);

	if(((param << flash_current_config->protect_post) != 
        (value & (flash_current_config->protect_mask << flash_current_config->protect_post)))
        || ((cmp << flash_current_config->cmp_post) !=
        (value & (0x01 << flash_current_config->cmp_post))))
	{
        value = (value & (~(flash_current_config->protect_mask 
			            << flash_current_config->protect_post))) 
			            | (param << flash_current_config->protect_post);
		value &= ~(1 << flash_current_config->cmp_post);
		value |= ((cmp & 0x01) << flash_current_config->cmp_post);
		
		os_printf("--write status reg:%x,%x--\r\n", value, flash_current_config->sr_size);
		flash_write_sr(flash_current_config->sr_size, value);
	}
}

static void flash_erase_sector(UINT32 address)
{
    UINT32 value;
    UINT32 erase_addr = address & 0xFFF000;
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DECLARATION();
#endif

    if(erase_addr >= flash_current_config->flash_size)
    {
        bk_printf("Erase error:invalid address0x%x\r\n", erase_addr);
        return;
    }

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DISABLE();
#endif
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    value = REG_READ(REG_FLASH_OPERATE_SW);
    value = ((erase_addr << ADDR_SW_REG_POSI)
             | (FLASH_OPCODE_SE << OP_TYPE_SW_POSI)
             | OP_SW
             | (value & WP_VALUE));
    REG_WRITE(REG_FLASH_OPERATE_SW, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_RESTORE();
#endif
}

static void flash_set_hpm(void)
{
    UINT32 value;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    value = REG_READ(REG_FLASH_OPERATE_SW);
    value = ((0x0 << ADDR_SW_REG_POSI)
             | (FLASH_OPCODE_HPM << OP_TYPE_SW_POSI)
             | (OP_SW)
             | (value & WP_VALUE));
    REG_WRITE(REG_FLASH_OPERATE_SW, value);
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
}

static void flash_read_data(UINT8 *buffer, UINT32 address, UINT32 len)
{
    UINT32 i, reg_value;
    UINT32 addr = address & (~0x1F);
    UINT32 buf[8];
    UINT8 *pb = (UINT8 *)&buf[0];
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DECLARATION();
#endif

    if(len == 0)
    {
        return;
    }

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DISABLE();
#endif
    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    while(len)
    {
        reg_value = REG_READ(REG_FLASH_OPERATE_SW);
        reg_value = ((addr << ADDR_SW_REG_POSI)
                     | (FLASH_OPCODE_READ << OP_TYPE_SW_POSI)
                     | OP_SW
                     | (reg_value & WP_VALUE));
        REG_WRITE(REG_FLASH_OPERATE_SW, reg_value);
        while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
        addr += 32;

        for(i = 0; i < 8; i++)
        {
            buf[i] = REG_READ(REG_FLASH_DATA_FLASH_SW);
        }

        for(i = address % 32; i < 32; i++)
        {
            *buffer++ = pb[i];
            address++;
            len--;
            if(len == 0)
            {
                break;
            }
        }
    }
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_RESTORE();
#endif
}

#if (CFG_SOC_NAME == SOC_BK7238)
static int flash_read_opt_data(UINT8 group, UINT8 *buffer, UINT32 address, UINT32 len)
{
    UINT32 i, reg_value, ret_len;
    UINT32 addr = address & (~0x1F);
    UINT32 buf[8];
    UINT8 *pb = (UINT8 *)&buf[0];

    if (len == 0)
        return 0;

    if (address >= 1024)
        return -1;

    if (buffer == NULL)
        return -2;

    if ((flash_id == 0x854215) || (flash_id == 0x852015) || (flash_id == 0x856015)) {
        /*puty slot id is 1~3*/
        if ((group < 1) || (group > 3))
            return -3;
    } else if (flash_id == 0xC86515) {
        /*gd slot id is 0~1*/
        if (group > 1)
            return -3;
    } else
        return -4;

    if ((address + len) > 1024) {
        len = 1024 - address;
    }
    ret_len = len;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    while (len) {
        reg_value = REG_READ(REG_FLASH_OPERATE_SW);
        reg_value = ((((group << 12) | ((addr & 0x3FF) << 0)) << ADDR_SW_REG_POSI)
                     | (FLASH_OPCODE_RDSCR << OP_TYPE_SW_POSI)
                     | OP_SW
                     | (reg_value & WP_VALUE));
        REG_WRITE(REG_FLASH_OPERATE_SW, reg_value);
        while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
        addr += 32;

        for (i = 0; i < 8; i++) {
            buf[i] = REG_READ(REG_FLASH_DATA_FLASH_SW);
        }

        for (i = address % 32; i < 32; i++) {
            *buffer++ = pb[i];
            address++;
            len--;
            if (len == 0) {
                break;
            }
        }
    }

    return ret_len;
}

static UINT32 flash_read_otp(flash_otp_t *param)
{
	UINT8 slot = 0, slot_max = 0;
	UINT32 read_size, slot_len, read_len, read_addr;
	int ret;

	if (param == NULL || param->buf == NULL) return 0;

	slot = (param->addr / 1024);
	if ((flash_id == 0x854215) || (flash_id == 0x852015) || (flash_id == 0x856015)) {
		if (slot > 2) return 0;
		slot++;
		slot_max = 3; /*1~3*/
	} else if (flash_id == 0xC86515) {
		if (slot > 1) return 0;
		slot_max = 1; /*0~1*/
	} else {
		return 0;
	}

	read_len = 0;
	while (param->len) {
		read_addr = (param->addr % 1024);
		slot_len = (1024 - read_addr);
		read_size = param->len > slot_len ? slot_len : param->len;
		ret = flash_read_opt_data(slot, param->buf, read_addr, read_size);
		if (ret > 0) {
			param->buf += read_size;
			param->addr += read_size;
			param->len -= read_size;
			read_len += read_size;
		} else {
			break;
		}

		if (++slot > slot_max) break;
	}

	return read_len;
}

static int flash_read_uid(flash_otp_t *param)
{
    UINT8 tx_buf[5] = {0x4B, 0x00, 0x00, 0x00, 0x00};
    UINT8 rx_buf[16] = {0};
    int ret = 0;

    if((param == NULL) || (param->buf == NULL))
        return 0;

    ret = flash_bypass_op_read(tx_buf, 5, rx_buf, 16);
    if(ret > 0)
    {
        int cp_len = (int)param->len;
        if(cp_len > ret)
            cp_len = ret;
        os_memcpy(param->buf, rx_buf, cp_len);
        return cp_len;
    }
    return ret;
}
#endif

static void flash_write_data(UINT8 *buffer, UINT32 address, UINT32 len)
{
    UINT32 i, reg_value;
    UINT32 addr = address & (~0x1F);
    UINT32 buf[8];
    UINT8 *pb = (UINT8 *)&buf[0];
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
    GLOBAL_INT_DECLARATION();
#endif

    if((addr >= flash_current_config->flash_size)
        || (len > flash_current_config->flash_size)
        || ((addr + len) > flash_current_config->flash_size))
    {
        bk_printf("Write error[addr:0x%x len:0x%x]\r\n", addr, len);
        return;
    }

    if(address % 32)
    {
        flash_read_data(pb, addr, 32);
    }
    else
    {
        os_memset(pb, 0xFF, 32);
    }

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
    while(len)
    {
        if(len < 32)
        {
            flash_read_data(pb, addr, 32);
            while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
        }
        for (i = address % 32; i < 32; i++)
        {
            pb[i] = *buffer++;
            address++;
            len--;
            if (len == 0)
                break;
        }

#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        GLOBAL_INT_DISABLE();
#endif
        for (i = 0; i < 8; i++)
        {
            REG_WRITE(REG_FLASH_DATA_SW_FLASH, buf[i]);
        }

        reg_value = REG_READ(REG_FLASH_OPERATE_SW);
        reg_value = ((addr << ADDR_SW_REG_POSI)
                     | (FLASH_OPCODE_PP << OP_TYPE_SW_POSI)
                     | OP_SW
                     | (reg_value & WP_VALUE));
        REG_WRITE(REG_FLASH_OPERATE_SW, reg_value);
        while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
#if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7238)
        GLOBAL_INT_RESTORE();
#endif
        addr += 32;
        os_memset(pb, 0xFF, 32);
    }
}

#if (CFG_SOC_NAME == SOC_BK7238)
static void flash_page_write_data(UINT8 *buffer, UINT32 address, UINT32 len)
{
	UINT32 i, j, cnt, mod, reg_value;
	GLOBAL_INT_DECLARATION();

	if ((address >= flash_current_config->flash_size)
		|| (len > flash_current_config->flash_size)
		|| ((address + len) > flash_current_config->flash_size)) {
		bk_printf("Write error[addr:0x%x len:0x%x]\r\n", address, len);
		return;
	}

	if (address % 256) {
		cnt = 256 - (address % 256);
		flash_write_data(buffer, address, cnt);
		len -= cnt;
		address += cnt;
		buffer += cnt;
	}

	cnt = len / 256;
	mod = len % 256;

	if(cnt)
	{
		while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
		/*enable page write*/
		reg_value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
		reg_value |= PAGE_WRITE_EN;
		REG_WRITE(REG_FLASH_SR_DATA_CRC_CNT, reg_value);

		for (i = 0; i < cnt; i++) {
			/*clear memory address*/
			REG_WRITE(REG_FLASH_PW_CONF, FLASH_PW_MEM_CLR);
			GLOBAL_INT_DISABLE();
			for (j = 0; j < 256; j++) {
				REG_WRITE(REG_FLASH_PW_CONF, (buffer[j] & FLASH_PW_MEM_DATA_MASK) << FLASH_PW_MEM_DATA_POSI);
			}
			reg_value = REG_READ(REG_FLASH_OPERATE_SW);
			reg_value = ((address << ADDR_SW_REG_POSI)
						 | (FLASH_OPCODE_PP << OP_TYPE_SW_POSI)
						 | OP_SW
						 | (reg_value & WP_VALUE));
			REG_WRITE(REG_FLASH_OPERATE_SW, reg_value);
			while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
			GLOBAL_INT_RESTORE();
			buffer += 256;
			address += 256;
		}

		/*disable page write*/
		reg_value = REG_READ(REG_FLASH_SR_DATA_CRC_CNT);
		reg_value &= ~PAGE_WRITE_EN;
		REG_WRITE(REG_FLASH_SR_DATA_CRC_CNT, reg_value);
	}

	if (mod) {
		flash_write_data(buffer, address, mod);
	}
}
#endif

void flash_protection_op(UINT8 mode, PROTECT_TYPE type)
{
	set_flash_protect(type);
}

void flash_init(void)
{
    UINT32 id;

    while(REG_READ(REG_FLASH_OPERATE_SW) & BUSY_SW);
	
    id = flash_get_id();
    FLASH_PRT("[Flash]id:0x%x\r\n", id);
    flash_get_current_flash_config();
	
	set_flash_protect(FLASH_UNPROTECT_LAST_BLOCK);

	#if (0 == CFG_JTAG_ENABLE)
	flash_disable_cpu_data_wr();
	#endif
	
    flash_set_line_mode(flash_current_config->line_mode);
    flash_enable_crc();
	
#if (CFG_SOC_NAME == SOC_BK7238)
    flash_set_clk(9);  // dco/2=60M
#else
    flash_set_clk(5);  // 60M
#endif

    ddev_register_dev(FLASH_DEV_NAME, &flash_op);
    
    os_printf("[Flash]init over\r\n");
}

void flash_exit(void)
{
    ddev_unregister_dev(FLASH_DEV_NAME);
}

UINT32 flash_read(char *user_buf, UINT32 count, UINT32 address)
{
    peri_busy_count_add();

    flash_read_data((UINT8 *)user_buf, address, count);

    peri_busy_count_dec();

    return FLASH_SUCCESS;
}

UINT32 flash_write(char *user_buf, UINT32 count, UINT32 address)
{
    peri_busy_count_add();

    if(4 == flash_current_config->line_mode)
    {
        flash_set_line_mode(LINE_MODE_TWO);
    }

#if (CFG_SOC_NAME == SOC_BK7238)
    if (count > 256)
    {
        flash_page_write_data((UINT8 *)user_buf, address, count);
    }
    else
#endif
    {
        flash_write_data((UINT8 *)user_buf, address, count);
    }

    if(4 == flash_current_config->line_mode)
    {
        flash_set_line_mode(LINE_MODE_FOUR);
    }
    peri_busy_count_dec();

    return FLASH_SUCCESS;
}


UINT32 flash_ctrl(UINT32 cmd, void *parm)
{
    UINT8 clk;
    UINT16 wsr;
    UINT32 address;
    UINT32 reg;
    UINT32 ret = FLASH_SUCCESS;
    flash_otp_t *otp_cfg;
    peri_busy_count_add();
    
    if(4 == flash_current_config->line_mode)
    {
        flash_set_line_mode(LINE_MODE_TWO);
    }
        
    switch(cmd)
    {
    case CMD_FLASH_SET_CLK:
        clk = (*(UINT8 *)parm);
        flash_set_clk(clk);
        break;

    case CMD_FLASH_SET_DPLL:
        sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_SET_FLASH_DPLL, 0);
        

        reg = REG_READ(REG_FLASH_CONF);
        reg &= ~(FLASH_CLK_CONF_MASK << FLASH_CLK_CONF_POSI);
        reg = reg | (5 << FLASH_CLK_CONF_POSI);
        REG_WRITE(REG_FLASH_CONF, reg);
        break;

    case CMD_FLASH_SET_DCO:
        sddev_control(SCTRL_DEV_NAME, CMD_SCTRL_SET_FLASH_DCO, 0);
        
        reg = REG_READ(REG_FLASH_CONF);
        reg &= ~(FLASH_CLK_CONF_MASK << FLASH_CLK_CONF_POSI);
        if (get_ate_mode_state()) {
#if (CFG_SOC_NAME == SOC_BK7238)
            reg = reg | (9 << FLASH_CLK_CONF_POSI);
#else
            reg = reg | (0xB << FLASH_CLK_CONF_POSI);
#endif
        } else {
            reg = reg | (9 << FLASH_CLK_CONF_POSI);
        }
        REG_WRITE(REG_FLASH_CONF, reg);
        break;

    case CMD_FLASH_WRITE_ENABLE:
        flash_write_enable();
        break;

    case CMD_FLASH_WRITE_DISABLE:
        flash_write_disable();
        break;

    case CMD_FLASH_READ_SR:
        (*(UINT16 *)parm) = flash_read_sr(2);
        break;

    case CMD_FLASH_WRITE_SR:
        flash_write_sr(*(unsigned long *)parm & 0x00FF, ((*(unsigned long *)parm) >> 8) & 0x00FFFF);
        break;

    case CMD_FLASH_READ_QE:
        (*(UINT8 *)parm) = flash_read_qe();
        break;

    case CMD_FLASH_SET_QE:
        if(flash_current_config->qe_bit)
        {
            flash_set_qe();
        }
        break;

    case CMD_FLASH_SET_QWFR:
        flash_set_qwfr();
        break;

    case CMD_FLASH_CLR_QWFR:
        flash_clr_qwfr();
        break;

    case CMD_FLASH_SET_WSR:
        wsr = (*(UINT16 *)parm);
        flash_set_wsr(wsr);
        break;

    case CMD_FLASH_GET_ID:
        (*(UINT32 *)parm) = flash_get_id();
        break;

    case CMD_FLASH_GET_SIZE:
        (*(UINT32 *)parm) = flash_get_size();
        break;

    case CMD_FLASH_READ_MID:
        (*(UINT32 *)parm) = flash_read_mid();
        break;

    case CMD_FLASH_GET_PROTECT:
        (*(UINT32 *)parm) = get_flash_protect();
        break;

    case CMD_FLASH_ERASE_SECTOR:
        address = (*(UINT32 *)parm);
        flash_erase_sector(address);
        break;

    case CMD_FLASH_SET_HPM:
        flash_set_hpm();
        break;

    case CMD_FLASH_SET_PROTECT:
        reg =  (*(UINT32 *)parm);
        flash_protection_op(FLASH_XTX_16M_SR_WRITE_DISABLE, reg);
        break;

    case CMD_FLASH_READ_OTP:
        #if (CFG_SOC_NAME == SOC_BK7238)
        otp_cfg = (flash_otp_t *)parm;
        ret = flash_read_otp(otp_cfg);
        #else
        otp_cfg = otp_cfg;
        ret = FLASH_FAILURE;
        #endif
        break;

    case CMD_FLASH_GET_UID:
        #if (CFG_SOC_NAME == SOC_BK7238)
        otp_cfg = (flash_otp_t *)parm;
        ret = flash_read_uid(otp_cfg);
        #else
        otp_cfg = otp_cfg;
        ret = FLASH_FAILURE;
        #endif
        break;

    default:
        ret = FLASH_FAILURE;
        break;
    }
    
    if(4 == flash_current_config->line_mode)
    {        
        flash_set_line_mode(LINE_MODE_FOUR);
    }

    peri_busy_count_dec();
    return ret;
}
// eof

