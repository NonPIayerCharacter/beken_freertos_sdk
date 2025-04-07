/**
 ******************************************************************************
 * @file    BkDriverI2C.c
 * @brief   This file provides all the headers of I2C operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2024 BEKEN Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */


#include "BkDriverI2c.h"
#include "drv_model_pub.h"
#include "i2c_pub.h"
#include "include.h"
#include "error.h"
#include "rtos_pub.h"
#include "mem_pub.h"

#if CFG_USE_I2C2

/************************************************************************
1. It is necessary to pull up resistors on the clock and data bus of i2c;
2. I2C_RETRY_TIME represents the read and write attempt time, which is adjusted for user testing
3. The timeout parameter represents the maximum time spent on reading and writing data;
4. Set the slave i2c address in the read and write interface of the slave;
For specific usage methods, please refer to wlan_cli c
*************************************************************************/

#define I2C_RETRY_TIME	     3000    //i2c write or read retry time

uint32 bk_i2c_open(bk_i2c_device_t i2cdevice)
{
    unsigned int oflag = 0;
    unsigned int status;
    DD_HANDLE i2c_hdl;
    uint32 Baudrate = 0;

    if(i2cdevice.ms_mode == I2C_MASTER_MODE){
        oflag = oflag & (~I2C2_MSG_WORK_MODE_MS_BIT);  //master
    }else{
        oflag = oflag | (I2C2_MSG_WORK_MODE_MS_BIT);    // slave
    }

    if(i2cdevice.address_width == I2C_ADDRESS_WIDTH_7BIT){
        oflag = oflag & (~I2C2_MSG_WORK_MODE_AL_BIT);  //7bit addr
    }else{
        oflag = oflag | (I2C2_MSG_WORK_MODE_AL_BIT);    // 10bit addr
    }

    if(i2cdevice.address_mode == I2C_ADDRESS_WITH_INNER){
        oflag = oflag | (I2C2_MSG_WORK_MODE_IA_BIT);     // with inner address
    }else{
        oflag = oflag & (~I2C2_MSG_WORK_MODE_IA_BIT);    // without inner address
    }
    Baudrate = i2cdevice.baudrate;

    oflag = oflag | (Baudrate << 4);     // set baudrate
    i2c_hdl = ddev_open(I2C2_DEV_NAME, &status, oflag);
    return i2c_hdl;
}

OSStatus bk_i2c_close(uint32 i2c_hdl)
{
    return ddev_close(i2c_hdl);
}

void bk_i2c_set_slave_addr(uint32 slave_addr)
{
    extern void i2c_set_slave_addr(UINT32 addr);
    return i2c_set_slave_addr(slave_addr);
}

extern beken_semaphore_t i2c_transdone_sema;

OSStatus bk_i2c_master_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint8_t salve_id, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_master write ... \r\n");
    if((tx_buffer == NULL)||(data_length == 0)){
        os_printf("tx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    i2c2_op.salve_id    = salve_id;       //send slave address

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c slave no ack \r\n");
            break;
        }
        status = ddev_write(i2c_hdl, (char *)tx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_master write done,status [%d] \r\n",status);

    return status;
}

OSStatus bk_i2c_master_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint8_t salve_id, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_master read ... \r\n");
    if((rx_buffer == NULL)||(data_length == 0)){
        os_printf("rx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    i2c2_op.salve_id   = salve_id;      //send slave address

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c slave no ack \r\n");
            break;
        }
        status = ddev_read(i2c_hdl, (char *)rx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_master read done ,status [%d]\r\n",status);

    return status;
}

OSStatus bk_i2c_slave_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_slave write ... \r\n");
    if((tx_buffer == NULL)||(data_length == 0)){
        os_printf("tx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c2 bus is busy \r\n");
            break;
        }
        status = ddev_write(i2c_hdl, (char *)tx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_slave write done ,status [%d]\r\n",status);

    return status;
}

OSStatus bk_i2c_slave_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_slave read ... \r\n");
    if((rx_buffer == NULL)||(data_length == 0)){
        os_printf("rx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c2 bus is busy \r\n");
            break;
        }
        status = ddev_read(i2c_hdl, (char *)rx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_slave read done,status [%d] \r\n",status);

    return status;
}

OSStatus bk_i2c_memory_write(uint32 i2c_hdl, const char *tx_buffer, uint32 data_length, uint8_t salve_id, uint8_t inner_addr, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_memory write ... \r\n");
    if((tx_buffer == NULL)||(data_length == 0)){
        os_printf("tx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    i2c2_op.op_addr     = inner_addr;
    i2c2_op.salve_id    = salve_id;       //send slave address

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c slave no ack \r\n");
            break;
        }
        status = ddev_write(i2c_hdl, (char *)tx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_memory write done,status [%d] \r\n",status);

    return status;
}

OSStatus bk_i2c_memory_read(uint32 i2c_hdl, char *rx_buffer, uint32 data_length, uint8_t salve_id, uint8_t inner_addr, uint32 timeout)
{
    unsigned int status = DRV_FAILURE;
    I2C_OP_ST i2c2_op;
    uint32 past, start_tick, cur_tick;

    os_printf("i2c_memory read ... \r\n");
    if((rx_buffer == NULL)||(data_length == 0)){
        os_printf("rx buffer is null \r\n");
        return DRV_FAILURE;
    }
    if(DD_HANDLE_UNVALID == i2c_hdl){
        os_printf("i2c_hdl DD_HANDLE_UNVALID \r\n");
        return DD_HANDLE_UNVALID;
    }

    i2c2_op.op_addr    = inner_addr;
    i2c2_op.salve_id   = salve_id;      //send slave address

    start_tick = rtos_get_time();
    do {
        cur_tick = rtos_get_time();
        past = (cur_tick >= start_tick) ? (cur_tick - start_tick) : (0xffffffffu - start_tick + cur_tick);
        if ((past) > I2C_RETRY_TIME){
            os_printf("i2c slave no ack \r\n");
            break;
        }
        status = ddev_read(i2c_hdl, (char *)rx_buffer, data_length, (unsigned long)&i2c2_op);
    } while (status != DRV_SUCCESS);

    if(i2c_transdone_sema != NULL){
        rtos_get_semaphore(&i2c_transdone_sema,timeout);
    }

    os_printf("i2c_memory read done ,status [%d]\r\n",status);

    return status;
}
// eof

#endif
