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
#include <stdlib.h>

#include "stdarg.h"
#include "mem_pub.h"
#include "sys_rtos.h"
#include "rtos_pub.h"
#include "error.h"
#include "sys_ctrl_pub.h"
#include "wlan_cli_pub.h"
#include "arm_mcu_pub.h"
#include "BkDriverPwm.h"
#include "ble_pub.h"
#include "sensor.h"
#include "spi_pub.h"
#include "i2c_pub.h"
#include "BkDriverTimer.h"
#include "BkDriverPwm.h"
#include "saradc_intf.h"
#include "drv_model_pub.h"
#include "BkDriverGpio.h"

#if CFG_USE_SPI_MST_FLASH
#include "spi_flash.h"
#endif

#if CFG_PERIPHERAL_TEST
#include "bk_err.h"
#include "bk_log.h"

#define TAG			 	"perit"

#define CFG_LOG_LEVEL			BK_LOG_DEBUG

#define PERI_LOGI				BK_LOGI
#define PERI_LOGW				BK_LOGW
#define PERI_LOGD				BK_LOGD

INT32 os_strcmp(const char *s1, const char *s2);


#define I2C_TEST_LEGNTH				32
#define I2C_TEST_EEPROM_LEGNTH			8

#if CFG_USE_I2C1
static void i2c1_test_eeprom(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    int i;
    DD_HANDLE i2c_hdl;
    unsigned int status;
    unsigned int oflag;
    I2C_OP_ST i2c1_op;
    I2C1_MSG_ST i2c_msg_config;

    os_printf(" i2c1_test_eeprom start  \r\n");

    i2c_msg_config.pData = (UINT8 *)os_malloc(I2C_TEST_EEPROM_LEGNTH);
    if (i2c_msg_config.pData == NULL) {
        os_printf("malloc fail\r\n");
        goto exit;
    }

    oflag   = (0 & (~I2C1_MSG_WORK_MODE_MS_BIT)     // master
               & (~I2C1_MSG_WORK_MODE_AL_BIT))    // 7bit address
              | (I2C1_MSG_WORK_MODE_IA_BIT);     // with inner address

    i2c_hdl = ddev_open(I2C1_DEV_NAME, &status, oflag);

    if (os_strcmp(argv[1], "write_eeprom") == 0) {
        os_printf("eeprom write\r\n");

        for (i = 0; i < I2C_TEST_EEPROM_LEGNTH; i++)
            i2c_msg_config.pData[i] = (i << 2) + 0x10 ;

        i2c1_op.op_addr    = 0x08;
        i2c1_op.salve_id   = 0x50;      //send slave address
        i2c1_op.slave_addr = 0x73;      //slave: as slave address

        do {
            status = ddev_write(i2c_hdl, (char *)i2c_msg_config.pData, I2C_TEST_EEPROM_LEGNTH, (unsigned long)&i2c1_op);
        } while (status != 0);
    }
    if (os_strcmp(argv[1], "read_eeprom") == 0) {
        os_printf("eeprom read\r\n");

        i2c1_op.op_addr    = 0x08;
        i2c1_op.salve_id   = 0x50;      //send slave address
        i2c1_op.slave_addr = 0x73;      //slave: as slave address

        do {
            status = ddev_read(i2c_hdl, (char *)i2c_msg_config.pData, I2C_TEST_EEPROM_LEGNTH, (unsigned long)&i2c1_op);
        } while (status != 0);
    }

    for (i = 0; i < I2C_TEST_EEPROM_LEGNTH; i++)
        os_printf("pData[%d]=0x%x\r\n", i, i2c_msg_config.pData[i]);

    ddev_close(i2c_hdl);

    os_printf(" i2c2 test over\r\n");

exit:

    if (NULL != i2c_msg_config.pData) {
        os_free(i2c_msg_config.pData);
        i2c_msg_config.pData = NULL;
    }
}
#endif

#if CFG_USE_I2C2
static void i2c2_test_eeprom(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{

    int i;
    DD_HANDLE i2c_hdl;
    unsigned int status;
    unsigned int oflag;
    I2C_OP_ST i2c2_op;
    I2C2_MSG_ST i2c_msg_config;

    os_printf(" i2c2_test_eeprom start  \r\n");

    i2c_msg_config.pData = (UINT8 *)os_malloc(I2C_TEST_EEPROM_LEGNTH);
    if (i2c_msg_config.pData == NULL) {
        os_printf("malloc fail\r\n");
        goto exit;
    }

    oflag   = (0 & (~I2C2_MSG_WORK_MODE_MS_BIT)     // master
               & (~I2C2_MSG_WORK_MODE_AL_BIT))    // 7bit address
              | (I2C2_MSG_WORK_MODE_IA_BIT);     // with inner address

    i2c_hdl = ddev_open(I2C2_DEV_NAME, &status, oflag);

    if (os_strcmp(argv[1], "write_eeprom") == 0) {
        os_printf("eeprom write\r\n");

        for (i = 0; i < I2C_TEST_EEPROM_LEGNTH; i++)
            i2c_msg_config.pData[i] = (i << 2) + 0x10 ;

        i2c2_op.op_addr     = 0x08;
        i2c2_op.salve_id    = 0x50;     //send slave address
        i2c2_op.slave_addr  = 0x73;     //slave: as slave address

        do {
            status = ddev_write(i2c_hdl, (char *)i2c_msg_config.pData, I2C_TEST_EEPROM_LEGNTH, (unsigned long)&i2c2_op);
        } while (status != 0);
    }
    if (os_strcmp(argv[1], "read_eeprom") == 0) {
        os_printf("eeprom read\r\n");

        i2c2_op.op_addr    = 0x08;
        i2c2_op.salve_id   = 0x50;      //send slave address
        i2c2_op.slave_addr = 0x73;      //slave: as slave address

        do {
            status = ddev_read(i2c_hdl, (char *)i2c_msg_config.pData, I2C_TEST_EEPROM_LEGNTH, (unsigned long)&i2c2_op);
        } while (status != 0);
    }

    for (i = 0; i < I2C_TEST_EEPROM_LEGNTH; i++)
        os_printf("pData[%d]=0x%x\r\n", i, i2c_msg_config.pData[i]);
    os_free(i2c_msg_config.pData);

    ddev_close(i2c_hdl);
    os_printf(" i2c2 test over\r\n");

exit:

    if (NULL != i2c_msg_config.pData) {
        os_free(i2c_msg_config.pData);
        i2c_msg_config.pData = NULL;
    }
}
#endif

#if (CFG_SUPPORT_SPI_TEST && CFG_USE_SPI_MST_FLASH)
#define SPI_BAUDRATE_26M       (26 * 1000 * 1000)
#define SPI_BAUDRATE           (2 * 1000 * 1000)
#define SPI_BAUDRATE_5M        (5* 1000 * 1000)

#define SPI_TX_BUF_LEN         (1024)
#define SPI_RX_BUF_LEN         (1024)

#define CMD_READ_STATUS_S7_0      0x05
#define CMD_READ_STATUS_S15_8     0x35
#define FLASH_STATUS_WIP_BIT      (1 << 0)
#define FLASH_STATUS_WEL_BIT      (1 << 1)
#define FLASH_STATUS_PRETECT_MASK (0x3F << 2)
#define CMD_WRITE_STATUS          0x01
#define ERASE_MODE_ALL            0x01
#define ERASE_MODE_BLOCK_64K      0x02
#define ERASE_MODE_BLOCK_32K      0x03
#define ERASE_MODE_SECTOR         0x04

#define CMD_ERASE_ALL             0xc7  // 0x60
#define CMD_ERASE_BLK_64K         0xD8
#define CMD_ERASE_BLK_32K         0x52
#define CMD_ERASE_SECTOR          0x20
#define CMD_READ_DATA             0x03
#define CMD_PAGE_PROG             0x02
#define CMD_WRITE_ENABLE          0x06

#if (CFG_SOC_NAME == SOC_BK7271)
int spi_channel;
#endif


beken_thread_t  test_spi_handle = NULL;
void test_spi_main( beken_thread_arg_t arg )
{
    bk_printf("test_spi_main\r\n");
    int ret = spi_flash_init(0);
    if(ret == 0) {
        uint32_t id = spi_flash_read_id();
        bk_printf("spi_flash_init %dhz ok, flash_id:0x%x\r\n", 0, id);
    } else {
        bk_printf("spi_flash_init %dhz failed!\r\n", 0);
    }
    while (1)
    {
        rtos_delay_milliseconds(1000);
    }
    test_spi_handle = NULL;
    rtos_delete_thread(NULL);
}

void test_spi_thread(void)
{
    OSStatus ret;
    if(test_spi_handle == NULL)
    {
        ret = rtos_create_thread(&test_spi_handle,
                            2,
                            "test_spi",
                            (beken_thread_function_t)test_spi_main,
                            4096,
                            NULL);
        bk_printf("test_spi %d\r\n", ret);
    }
}


void gspi_test(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    struct spi_message  msg;
    UINT32 max_hz;
    UINT32 mode = 0;

    SPI_CFG_ST cfg;
    cfg.u.value = 0;
    mode = mode;

    #if (CFG_SOC_NAME == SOC_BK7271)
    spi_channel = 2;
    #endif

    if (os_strcmp(argv[1], "master") == 0)
    {
        cfg.u.slave = 0;
    
        if (argc < 2)
            max_hz = SPI_BAUDRATE;
        else
            max_hz = atoi(argv[2]);

        #if (CFG_USE_SPI_MASTER) 
        bk_spi_master_init(max_hz, cfg.u.value);
        #endif
    }
    else if (os_strcmp(argv[1], "slave") == 0)
    {
        cfg.u.slave = 1;
        max_hz = SPI_BAUDRATE;

        #if (CFG_USE_SPI_SLAVE) 
        bk_spi_slave_init(max_hz, cfg.u.value);
        #endif
    }
    #if CFG_USE_SPI_DMA_SLAVE
    else if (os_strcmp(argv[1], "slave_dma_rx") == 0)
    {
        UINT8 *buf;
        int rx_len, ret;

        max_hz = SPI_BAUDRATE;
        cfg.u.slave = 1;
        cfg.u.dma = 1;
        bk_spi_slave_dma_init(max_hz, cfg.u.value);

        if (argc < 2)
            rx_len = SPI_RX_BUF_LEN;
        else
            rx_len = atoi(argv[2]);

        bk_printf("spi dma rx: rx_len:%d\n", rx_len);

        buf = os_malloc(rx_len * sizeof(UINT8));
        if (!buf) {
            bk_printf("spi test malloc buf fail\r\n");
            return ;
        }

        os_memset(buf, 0, rx_len);

        msg.send_buf = NULL;
        msg.send_len = 0;
        msg.recv_buf = buf;
        msg.recv_len = rx_len;

        ret = bk_spi_slave_dma_xfer(&msg);
        if (ret < 0)
            bk_printf("spi dma recv error%d\r\n", ret);
        else {
            for (int i = 0; i < rx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");
            os_free(buf);
        }
        bk_spi_slave_dma_deinit();
    }
    else if ((os_strcmp(argv[1], "slave_dma_tx") == 0))
    {
        UINT8 *buf;
        int tx_len, ret;

        if (argc < 2)
            tx_len = SPI_RX_BUF_LEN;
        else
            tx_len = atoi(argv[2]);

        max_hz = SPI_BAUDRATE;
        cfg.u.slave = 1;
        cfg.u.dma = 1;
        bk_spi_slave_dma_init(max_hz, cfg.u.value);

        bk_printf("spi dma tx: tx_len:%d\n", tx_len);

        buf = os_malloc(tx_len * sizeof(UINT8));
        if (!buf) {
            bk_printf("spi test malloc buf fail\r\n");
            return ;
        }

        os_memset(buf, 0, tx_len);

        for (int i = 0; i < tx_len; i++)
            buf[i] = i & 0xFF;

        msg.send_buf = buf;
        msg.send_len = tx_len;
        msg.recv_buf = NULL;
        msg.recv_len = 0;

        ret = bk_spi_slave_dma_xfer(&msg);
        if (ret < 0)
            bk_printf("spi dma send error%d\r\n", ret);
        else {
            for (int i = 0; i < tx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");
            os_free(buf);
        }
        bk_spi_slave_dma_deinit(); 
    }
    #endif
    #if CFG_USE_SPI_DMA_MASTER
    else if ((os_strcmp(argv[1], "master_dma_tx") == 0))
    {
        UINT8 *buf;
        int tx_len, ret;

        if (argc < 2)
            tx_len = SPI_RX_BUF_LEN;
        else
            tx_len = atoi(argv[2]);

        max_hz = atoi(argv[3]);//SPI_BAUDRATE;

        bk_printf("spi master  dma tx: tx_len:%d max_hz:%d\r\n", tx_len, max_hz);

        buf = os_malloc(tx_len * sizeof(UINT8));
        if (!buf) {
            bk_printf("spi test malloc buf fail\r\n");
            return ;
        }

        os_memset(buf, 0, tx_len);
        for (int i = 0; i < tx_len; i++)
            buf[i] = i & 0xFF;

        msg.send_buf = buf;
        msg.send_len = tx_len;
        msg.recv_buf = NULL;
        msg.recv_len = 0;

        cfg.u.slave = 0;
        cfg.u.dma = 1;
        bk_spi_master_dma_init(max_hz, cfg.u.value);

        ret = bk_spi_master_dma_xfer(&msg, 0);
        if (ret < 0)
            bk_printf("spi dma send error%d\r\n", ret);
        else {
            for (int i = 0; i < tx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");
            os_free(buf);
        }
        bk_spi_master_dma_deinit(0);
    }
    else if ((os_strcmp(argv[1], "master_dma_rx") == 0))
    {
        UINT8 *buf;
        int rx_len, ret;

        if (argc < 2)
            rx_len = SPI_RX_BUF_LEN;
        else
            rx_len = atoi(argv[2]) + 1;	//slave tx first send 0x72 so must send one more

        max_hz = atoi(argv[3]);//SPI_BAUDRATE;

        bk_printf("spi master  dma rx: rx_len:%d max_hz:%d\r\n\n", rx_len, max_hz);

        buf = os_malloc(rx_len * sizeof(UINT8));
        if (!buf) {
            bk_printf("spi test malloc buf fail\r\n");
            return ;
        }

        os_memset(buf, 0, rx_len);

        msg.send_buf = NULL;
        msg.send_len = 0;
        msg.recv_buf = buf;
        msg.recv_len = rx_len;

        cfg.u.slave = 0;
        cfg.u.dma = 1;
        bk_spi_master_dma_init(max_hz, cfg.u.value);

        ret = bk_spi_master_dma_xfer(&msg, BK_SPI_M_DMA_DEF_WAIT_TIME);
        if (ret < 0)
            bk_printf("spi dma recv error%d\r\n", ret);
        else {
            for (int i = 0; i < rx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");
            os_free(buf);
        }
        bk_spi_master_dma_deinit(0);
    }
    #endif

    if (os_strcmp(argv[1], "stx") == 0)
    {
        #if (CFG_USE_SPI_SLAVE) 
        UINT8 *buf;
        int tx_len;

        if (argc < 3)
            tx_len = SPI_TX_BUF_LEN;
        else
            tx_len = atoi(argv[2]);

        bk_printf("spi init tx_len:%d\n", tx_len);

        buf = os_malloc(tx_len * sizeof(UINT8));
        if (buf) {
            os_memset(buf, 0, tx_len);
            for (int i = 0; i < tx_len; i++)
                buf[i] = i & 0xff;
            msg.send_buf = buf;
            msg.send_len = tx_len;
            msg.recv_buf = NULL;
            msg.recv_len = 0;

            bk_spi_slave_xfer(&msg);
            for (int i = 0; i < tx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");

            os_free(buf);
        }
        #endif
    }
    #if (CFG_USE_SPI_SLAVE) 
    else if (os_strcmp(argv[1], "srx") == 0)
    {
        UINT8 *buf;
        int rx_len;

        if (argc < 3)
            rx_len = SPI_RX_BUF_LEN;
        else
            rx_len = atoi(argv[2]);

        bk_printf("SPI_RX: rx_len:%d\n", rx_len);

        buf = os_malloc(rx_len * sizeof(UINT8));

        if (buf) {
            os_memset(buf, 0, rx_len);

            msg.send_buf = NULL;
            msg.send_len = 0;
            msg.recv_buf = buf;
            msg.recv_len = rx_len;

            //CLI_LOGI("buf:%d\r\n", buf);
            rx_len = bk_spi_slave_xfer(&msg);
            bk_printf("rx_len:%d\r\n", rx_len);

            for (int i = 0; i < rx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");

            os_free(buf);
        }
    }
    #endif

    #if (CFG_USE_SPI_MASTER) 
    else if (os_strcmp(argv[1], "mtx") == 0)
    {
        UINT8 *buf;
        int tx_len;

        if (argc < 3)
            tx_len = SPI_TX_BUF_LEN;
        else
            tx_len = atoi(argv[2]);

        bk_printf("spi init tx_len:%d\n", tx_len);

        buf = os_malloc(tx_len * sizeof(UINT8));
        if (buf) {
            os_memset(buf, 0, tx_len);
            for (int i = 0; i < tx_len; i++)
                buf[i] = i & 0xff;
            msg.send_buf = buf;
            msg.send_len = tx_len;
            msg.recv_buf = NULL;
            msg.recv_len = 0;

            bk_spi_master_xfer(&msg);

            for (int i = 0; i < tx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");

            os_free(buf);
        }
    }
    else if (os_strcmp(argv[1], "mrx") == 0)
    {
        UINT8 *buf;
        int rx_len;

        if (argc < 3)
            rx_len = SPI_RX_BUF_LEN;
        else
            rx_len = atoi(argv[2]);
        rx_len += 1;

        bk_printf("SPI_RX: rx_len:%d\n", rx_len);

        buf = os_malloc(rx_len * sizeof(UINT8));
        if (buf) {
            os_memset(buf, 0, rx_len);

            msg.send_buf = NULL;
            msg.send_len = 0;
            msg.recv_buf = buf;
            msg.recv_len = rx_len;

            //CLI_LOGI("buf:%d\r\n", buf);
            rx_len = bk_spi_master_xfer(&msg);
            bk_printf("rx_len:%d\r\n", rx_len);

            for (int i = 0; i < rx_len; i++) {
                bk_printf("%02x,", buf[i]);
                if ((i + 1) % 32 == 0)
                    bk_printf("\r\n");
            }
            bk_printf("\r\n");

            os_free(buf);
        }
    }
    #endif

    #if CFG_USE_SPI_MST_FLASH 
    else if(os_strcmp(argv[1], "flash_init") == 0) 
    {
        if (argc < 3)
            max_hz = SPI_DEF_CLK_HZ;
        else
            max_hz = atoi(argv[2]);//SPI_BAUDRATE;

        int ret = spi_flash_init(max_hz);
        if(ret == 0) {
            uint32_t id = spi_flash_read_id();
            bk_printf("spi_flash_init %dhz ok, flash_id:0x%x\r\n", max_hz, id);
        } else {
            bk_printf("spi_flash_init %dhz failed!\r\n", max_hz);
        }
    }
    else if(os_strcmp(argv[1], "flash_rd_id") == 0)
    {
        uint32_t id = spi_flash_read_id();
        bk_printf("flash_id:0x%x\r\n", id);
    }
    else if(os_strcmp(argv[1], "flash_rd_sr") == 0)
    {
        uint16_t status = spi_flash_read_status();
        bk_printf("flash_rd_sr:0x%04x\r\n", status);
    }
    else if(os_strcmp(argv[1], "flash_unprotect") == 0)
    {
        spi_flash_unprotect();
    }
    else if(os_strcmp(argv[1], "flash_protect") == 0)
    {
        spi_flash_protect();
    }
    else if(os_strcmp(argv[1], "flash_block_erase") == 0)
    {
        spi_flash_unprotect();
        spi_flash_erase(0, 64 * 1024);
    }
    else if(os_strcmp(argv[1], "flash_wr") == 0)
    {
        int i;
        uint8_t *buf;
        uint32_t wr_len = 256;

        buf = (uint8_t *)os_zalloc(wr_len);
        if(NULL == buf) {
            bk_printf("flash_wr malloc faile\r\n");
        }

        for(i = 0; i < wr_len; i ++) {
            buf[i] = 0x60 + i;
        }
        bk_printf("flash_wr xfering\r\n");
        spi_flash_write(0, wr_len, buf);
        bk_printf("flash_wr xfer done\r\n");

        for(i = 0; i < wr_len; i ++) {
            bk_printf("[%d]:0x%02x \r\n", i, buf[i]);
        }
        if(buf) {
            os_free(buf);
            buf = NULL;
        }
    }
    else if(os_strcmp(argv[1], "flash_rd") == 0)
    {
        int i;
        uint8_t *buf;
        uint32_t rd_len = 256;

        buf = (uint8_t *)os_zalloc(rd_len);
        if(NULL == buf) {
            bk_printf("flash_rd malloc faile\r\n");
        }

        bk_printf("flash_rd xfering\r\n");
        spi_flash_read(0, rd_len, buf);
        bk_printf("flash_rd xfer done\r\n");

        for(i = 0; i < rd_len; i ++) {
            bk_printf("[%d]:0x%02x \r\n", i, buf[i]);
        }
        if(buf) {
            os_free(buf);
            buf = NULL;
        }
    }
    else if(os_strcmp(argv[1], "flash_abc") == 0)
    {
        test_spi_thread();
    }
    #endif
    else
    {
        bk_printf("master trx:gspi_test mtx/mrx	len\r\n");
        bk_printf("slave trx:gspi_test stx/srx	len\r\n");
    }
}

uint32 spi_dma_slave_rx_thread_main(void);
uint32 spi_dma_master_tx_thread_main(void);
uint32 spi_dma_master_tx_loop_stop(void);
uint32 spi_dma_slave_rx_loop_stop(void);

void spi_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    #if (CFG_SOC_NAME == SOC_BK7271)
    spi_channel = 2;
    #endif

    if (os_strcmp(argv[1], "slave_rx_loop") == 0) {
        bk_printf("spi dma rx loop test\r\n");
        spi_dma_slave_rx_thread_main();
    } else if (os_strcmp(argv[1], "master_tx_loop") == 0) {

        bk_printf("spi dma tx loop test\r\n");
        spi_dma_master_tx_thread_main();
    } else if (os_strcmp(argv[1], "master_tx_stop") == 0) {

        bk_printf("spi dma tx stop loop\r\n");
        spi_dma_master_tx_loop_stop();
    } else if (os_strcmp(argv[1], "slave_rx_stop") == 0) {

        bk_printf("spi dma slave rx stop loop\r\n");
        spi_dma_slave_rx_loop_stop();
    }
}
#endif

static void pwm_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    UINT8 channel1;
    UINT32 duty_cycle1, cycle;
    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
    UINT8 channel2;
    UINT32 duty_cycle2;
    UINT32 dead_band;
    #endif
    OSStatus ret;

    /*get the parameters from command line*/
    channel1	= atoi(argv[2]);
    duty_cycle1	= atoi(argv[3]);
    cycle		= atoi(argv[4]);
    #if (CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N)
    channel2	= atoi(argv[5]);
    duty_cycle2	= atoi(argv[6]);
    dead_band	= atoi(argv[7]);
    #endif

    if (cycle < duty_cycle1)
    {
        PERI_LOGW(TAG, "pwm param error: end < duty\r\n");
        return;
    }

    if (os_strcmp(argv[1], "single") == 0)
    {
        if (5 != argc) {
            PERI_LOGW(TAG, "pwm single test usage: pwm [single][channel][duty_cycle][freq]\r\n");
            return;
        }
        PERI_LOGI(TAG, "pwm channel %d: duty_cycle: %d  freq:%d \r\n", channel1, duty_cycle1, cycle);
        ret = bk_pwm_initialize(channel1, cycle, duty_cycle1);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "init err\r\n");
            return;
        }
        ret = bk_pwm_start(channel1);				/*start single pwm channel once */
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "start err\r\n");
            return;
        }
    }
    else if (os_strcmp(argv[1], "stop") == 0)
    {
        ret = bk_pwm_stop(channel1);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "stop err\r\n");
            return;
        }
    }
    #if ((CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) ||(CFG_SOC_NAME == SOC_BK7271) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N))
    else if (os_strcmp(argv[1], "update") == 0)
    {
        if (5 != argc) {
            PERI_LOGW(TAG, "pwm update usage: pwm [update][channel1][duty_cycle][freq]\r\n");
            return;
        }

        PERI_LOGI(TAG, "pwm %d update: %d\r\n", duty_cycle1);
        ret = bk_pwm_update_param(channel1, cycle, duty_cycle1);		/*updata pwm freq and duty_cycle */
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "updata err\r\n");
            return;
        }
    }
    else if (os_strcmp(argv[1], "cap") == 0)
    {
        uint8_t cap_mode = duty_cycle1;

        if (5 != argc) {
            PERI_LOGW(TAG, "pwm cap usage: pwm [cap][channel1][mode][freq]\r\n");
            return;
        }
        PERI_LOGI(TAG, "pwm %d capture\r\n", channel1);
        ret = bk_pwm_capture_initialize(channel1, cap_mode);			/*capture pwm value */
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "init err\r\n");
            return;
        }
        #if (CFG_SOC_NAME != SOC_BK7231N) && (CFG_SOC_NAME != SOC_BK7236) && (CFG_SOC_NAME != SOC_BK7238) && (CFG_SOC_NAME != SOC_BK7252N)
        bk_pwm_start(channel1);
        #else
        ret = bk_pwm_capture_start(channel1);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "start err\r\n");
            return;
        }
        #endif
    }
    else if (os_strcmp(argv[1], "capvalue") == 0)
    {
        UINT32 cap_value = bk_pwm_get_capvalue(channel1);
        PERI_LOGI(TAG, "pwm : %d cap_value=%x \r\n", channel1, cap_value);

        (void)cap_value;
    }
    else if (os_strcmp(argv[1], "cap_stop") == 0)
    {
        ret = bk_pwm_capture_stop(channel1);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "stop err\r\n");
            return;
        }
        PERI_LOGI(TAG, "pwm : %d cap_stop\r\n", channel1);
    }

    #if ((CFG_SOC_NAME == SOC_BK7231N) || (CFG_SOC_NAME == SOC_BK7236) || (CFG_SOC_NAME == SOC_BK7238) || (CFG_SOC_NAME == SOC_BK7252N))
    else if (os_strcmp(argv[1], "cw") == 0)
    {

        if (8 != argc) {
            PERI_LOGW(TAG, "pwm cw test usage: pwm [cw][channel1][duty_cycle1][freq][channel2][duty_cycle2][dead_band]\r\n");
            return;
        }

        PERI_LOGI(TAG, "pwm : %d / %d cw pwm test \r\n", channel1, channel2);

        ret = bk_pwm_cw_initialize(channel1, channel2, cycle, duty_cycle1, duty_cycle2, dead_band);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "init err\r\n");
            return;
        }
        ret = bk_pwm_cw_start(channel1, channel2);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "start err\r\n");
            return;
        }
    } else if (os_strcmp(argv[1], "updatecw") == 0)
    {
        if (8 != argc) {
            PERI_LOGW(TAG, "pwm cw test usage: pwm [cw][channel1][duty_cycle1][freq][channel2][duty_cycle2][dead_band]\r\n");
            return;
        }

        PERI_LOGI(TAG, "pwm : %d / %d cw updatw pwm test \r\n", channel1, channel2);

        ret = bk_pwm_cw_update_param(channel1, channel2, cycle, duty_cycle1, duty_cycle2, dead_band);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "updata err\r\n");
            return;
        }
    } else if (os_strcmp(argv[1],  "loop") == 0)
    {
        uint16_t cnt = 1000;

        PERI_LOGI(TAG, "pwm : %d / %d pwm update loop test \r\n", channel1, channel2);

        while (cnt--) {
            duty_cycle1 = duty_cycle1 - 100;

            bk_pwm_cw_update_param(channel1, channel2, cycle, duty_cycle1, duty_cycle2, dead_band);
            rtos_delay_milliseconds(10);

            if (duty_cycle1 == 0)
                duty_cycle1 = cycle;
        }
    } else if (os_strcmp(argv[1], "cw_stop") == 0)
    {
        PERI_LOGI(TAG, "pwm stop : %d / %d cw pwm test \r\n", channel1, channel2);
        ret = bk_pwm_cw_stop(channel1, channel2);
        if(ret != kNoErr)
        {
            PERI_LOGW(TAG, "stop err\r\n");
            return;
        }
    }
    #endif
    #endif
}

const struct cli_command peripheral_clis[] = {
    #if (CFG_SUPPORT_SPI_TEST && CFG_USE_SPI_MST_FLASH)
    {"gspi_test", "general spi", gspi_test},
    //{"spi_test", "spi dma rx loop", spi_Command},
    #endif

    #if CFG_USE_I2C1
    {"i2c1_test", "i2c1_test write/read_eeprom", i2c1_test_eeprom},
    #endif

    #if CFG_USE_I2C2
    {"i2c2_test", "i2c2_test write/read_eeprom", i2c2_test_eeprom},
    #endif

    {"pwm_test", "pwm single/update/cw ", pwm_Command},
};

void bk_peripheral_cli_init(void)
{
    int ret;

    os_printf("peripheral cli int \r\n");
    ret = cli_register_commands(peripheral_clis, sizeof(peripheral_clis) / sizeof(struct cli_command));
    if (ret)
        os_printf("ret: %d peripheral commands fail.\r\n",ret);
}
#endif
// eof

