
:link_to_translation:`zh_CN:[中文]`

I2C user guide
==================


Overview
------------------

I2C is a serial synchronous half-duplex communication protocol that requires only two buses, namely the serial data line (SDA) and the serial clock line (SCL). These lines require pull-up resistors. The BK7238 integrates an I2C interface that can operate in master mode or slave mode. It supports standard mode (up to 100 kbps) as well as fast mode (up to 400 kbps) with 7-bit addressing.


I2C is simple and cost-effective. It is primarily used for short-range communication (within one foot) of low-speed peripheral devices.

.. figure:: ../../_static/i2c_connection.png
    :align: center
    :alt: I2C Connection
    :figclass: align-center

    I2C Connection

The figure above shows the I2C hardware connection, where:
 
 - SCL: clock signal, provides a clock for data transfer and is used to synchronize data transfer.
 - SDA: used to transfer data.


Reference routine
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