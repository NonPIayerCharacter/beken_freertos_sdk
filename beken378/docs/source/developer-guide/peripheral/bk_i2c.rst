I2C 使用指南
==================


概述
------------------

I2C是是一种串行同步半双工通信协议，只需要两条总线，即串行数据线（SDA）和串行时钟线（SCL）,这些线都需要上拉电阻。
BK7238嵌入了I2C接口，可以作为主模式或从模式。它支持标准（最高100 kbps）以及具有7位寻址的快速（高达400kbps）模式。

I2C 具有简单且制造成本低廉等优点，主要用于低速外围设备的短距离通信（一英尺以内）。

.. figure:: ../../_static/i2c_connection.png
    :align: center
    :alt: I2C Connection
    :figclass: align-center

    I2C Connection

上图为 I2C 硬件连接，其中：
 
 - SCL：时钟信号, 提供传输数据的时钟，用来同步数据传输；
 - SDA：用于传输数据。


参考例程
------------------

::

    #define I2C_TEST_EEPROM_LEGNTH          8

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