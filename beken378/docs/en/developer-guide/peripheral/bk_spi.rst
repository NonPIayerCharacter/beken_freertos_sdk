
:link_to_translation:`zh_CN:[中文]`

==================
SPI user guide
==================



Overview
==================


SPI (Serial Peripheral Interface）
----------------------------------

SPI is a high-speed, full-duplex, synchronous communication bus, which is widely used in the communication between ADC, Flash, etc. and MCU. The BK7238 integrates an SPI interface that can operate in master mode or slave mode. The SPI interface allows a clock frequency up to 30 MHz in master mode and up to 20 MHz in slave mode.
The SPI interface supports the following features:

- Configurable 8-bit or 16-bit data width
- Supports 4-wire and 3-wire modes (no CSN pin)
- Embedded 64-depth RX FIFO and 64-depth TX FIFO with DMA capability
- Programmable clock polarity and phase
- Programmable data order with MSB-first or LSB-first shifting


.. figure:: ../../_static/spi_connection.png
    :align: center
    :alt: SPI Connection
    :figclass: align-center

    SPI Connection


The figure above shows the SPI hardware connection, where:
 
 - SCK: clock signal, provides a clock for data transfer and is used to synchronize data transfer.
 - MOSI: (Master Output, Slave Input), used for master to transmit data to slave.
 - MISO: (Master Input, Slave Output), used for slave to transmit data to master.
 - CSN: chip select signal, the master selects the corresponding slave to transfer data, low level is active.


SPI modes
------------------

SPI has four communication modes, and they are mainly distinguished by the polarity and phase of the SPI clock.

+----------+------+------+-------------------+----------------------+
| SPI mode | CPOL | CPHA | SCK initial level |  Data is sampled on  |
+==========+======+======+===================+======================+
|    0     |  0   |   0  |     Low level     |   First clock edge   |
+----------+------+------+-------------------+----------------------+
|    1     |  0   |   1  |     Low level     |   Second clock edge  |
+----------+------+------+-------------------+----------------------+
|    2     |  1   |   0  |     High level    |   First clock edge   |
+----------+------+------+-------------------+----------------------+
|    3     |  1   |   1  |     High level    |   Second clock edge  |
+----------+------+------+-------------------+----------------------+

We commonly use mode 0 and mode 3, because they both sample data on the rising edge. You don't need to care about the initial level of the clock, as long as the data is collected on the rising edge. The BK7238 SPI supports four modes.

SPI timing
------------------

.. figure:: ../../_static/spi_timing.png
    :align: center
    :alt: SPI Timing
    :figclass: align-center

    SPI Timing


The figure above shows the timing of the master transmitting 0x56 (0b0101 0110) to the slave (CPOL = 1, CPHA = 1, MSB first).
 
 - Pull CS low to select the corresponding slave.
 - In each SCK cycle, MOSI outputs the corresponding level, and outputs the MSB bit of the data first.
 - The slave will read the level on MOSI on the rising edge of each SCK cycle.




