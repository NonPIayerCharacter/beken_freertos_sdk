
:link_to_translation:`zh_CN:[中文]`

Basic instruction set
======================


------------------------
AT: test AT startup
------------------------

**Execution command**

Command:
::

	AT

Return:
::

	OK

**Note**

- Test whether the AT command is valid.


----------------------------
AT+RST: restart the module
----------------------------

**Execution command**

Command:
::

	AT+RST

Return:
::

	OK


---------------------------------
AT+GMR: display firmware version
---------------------------------

**Execution command**

Command:
::

	AT+GMR

Return:
::

	<ATVER>
	<SDKVER>


----------------------------------------
AT+HELP: display all supported commands
----------------------------------------

**Execution command**

Command:
::

	AT+HELP?

Return:
::

	CMDRSP:cmd:<command>,help


---------------------------------------------------------
AT+ USRRAM: display the RAM space available to the user
---------------------------------------------------------

**Execution command**

Command:
::

	AT+USRRAM?

Return:
::

	+USERRAM:value
	OK


----------------------------------------------------
AT+GSLP: enter deep sleep mode
----------------------------------------------------

**Execution command**

Command:
::

	AT+GSLP=<time>

Return:
::

	<time>
	OK

**Parameter**

- <time>: The length of time the device enters deep sleep, unit: milliseconds. After the set time is up, the device automatically wakes up, calls the deep sleep wake-up stub, and then loads the application.


----------------------------------------------------
ATE: enable or disable AT echo function
----------------------------------------------------

**Execution command**

Command:
::

	ATE0

Or

::

	ATE1

Return:
::

	OK

**Notes**

+ ATE0: Disable echo and print log
+ ATE1: Enable echo and print log


-------------------------------------------------------------------------------------
AT+UART_CUR: set the current temporary configuration of UART without saving to flash
-------------------------------------------------------------------------------------

**Execution command**

Command:
::

	AT+UART_CUR=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>

Return:
::

	OK

**Parameters**

- <baudrate>: UART baud rate

    + Supported range: 9600 to 2000000

- <databits>: data bits

    + 5: 5 bits
    + 6: 6 bits
    + 7: 7 bits
    + 8: 8 bits

- <stopbits>: stop bits

    + 1: 1 bit
    + 2: 2 bits

- <parity>: parity bit

    + 0: none
    + 1: odd
    + 2: even

- <flow control>: flow control

    + 0: disable flow control
    + 1: enable RTS
    + 2: enable CTS
    + 3: enable RTS and CTS at the same time


--------------------------------------------------------------------
AT+UART_DEF: set UART default configuration without saving to flash
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+UART_DEF?

Return:
::

	+UART_DEF:<baudrate>,<databits>,<stopbits>,<parity>,<flow control>
	OK

**Execution command**

Command:
::

	AT+UART_DEF:<baudrate>,<databits>,<stopbits>,<parity>,<flow control>

Return:
::

	OK

**Parameters**

+ <baudrate>: UART baud rate

	+ support range: 9600 to 2000000

+ <databits>: data bits

	+ 5: 5 bits
	+ 6: 6 bits
	+ 7: 7 bits
	+ 8: 8 bits

+ <stopbits>: stop bits

	+ 1: 1 bit
	+ 2: 2 bits

+ <parity>: parity bit

	+ 0: none
	+ 1: odd
	+ 2: even

+ <flow control>: flow control

	+ 0: disable flow control
	+ 1: enable RTS
	+ 2: enable CTS
	+ 3: enable RTS and CTS at the same time


--------------------------------------------------------------------
AT+SYSFLASH: view device Flash partition
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+SYSFLASH?

Return:
::

	<index>,< partition>,<dev>,< offset>,< length>
	OK

**Parameters**

- <index>: partition number
- <partition>: user partition name
- <dev>: physical device number
- <offset>: offset address
- <length>: data length


--------------------------------------------------------------------
AT+SYSTIMESTAMP: query/set local timestamp
--------------------------------------------------------------------

**Query command**

Function:

- Query local timestamp

Command:
::

	AT+SYSTIMESTAMP?

Return:
::

	+SYSTIMESTAMP:<Unix_timestamp>
	OK

**Execution command**

Function:

- Set the local timestamp. When the SNTP time is updated, the timestamp will be updated synchronously.

Command:
::

	AT+SYSTIMESTAMP=<Unix_timestamp>

Return:
::

	OK

**Parameter**

- <Unix-timestamp>: Unix timestamp, unit: seconds

**Parameter**

::

	AT+SYSTIMESTAMP=1686305483    //2023-6-9 10:11:23


--------------------------------------------------------------------
AT+SLEEPPWCFG: set light sleep wake-up source and wake-up GPIO
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+SLEEPPWCFG= <wakeup source>,<param1>[,<param2>]

Return:
::

	OK

**Parameter**

+ <wakeup source>: wake-up source

	+ 0: wake-up on RTC 
	+ 1: Keep configuration
	+ 2：wake-up on GPIO 

+ <param1>:

	+When the wake-up source is specified as RTC, this parameter specifies the sleep time, unit: seconds.
	+When the wake-up source is specified as a GPIO, this parameter specifies the GPIO pin number (0 to 28).

+ <param2>: When the wakeup source is specified as a GPIO, this parameter specifies the GPIO wake-up trigger mode.

	+0: rising edge
	+1: falling edge

**Example**

Wake-up on RTC
::

	AT+SLEEPPWCFG=0,2		//Automatically wake up in 2 seconds

Wake-up on GPIO 
::

	AT+SLEEPPWCFG=2,20,0		//Wake up on GPIO20 rising edge


--------------------------------------------------------------------
AT+SYSSTORE: set parameter storage mode
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+SYSSTORE?

Return:
::

	+SYSSTORE:<store_mode>
	OK

**Execution command**

Command:
::

	AT+SYSSTORE=<store_mode>

Return:
::

	OK

**Parameter**

+ <store_mode>: parameter storage mode

	- 0: The command configuration is not stored in Flash.
	- 1: The command configuration is stored in Flash (default).


--------------------------------------------------------------------
AT+RESTORE: restore factory settings
--------------------------------------------------------------------

**Execution command**

Command:
::

	AT+RESTORE

Return:
::

	OK

**Note**

- This command erases all parameters saved to Flash and restores them to default parameters.
- Running this command will reboot the device.

--------------------------------------------------------------------
AT+PRODUCTID: set/query product ID
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+PRODUCTID?

Return:
::

	+PRODUCTID: <“productid”>
	OK

**Execution command**

Command:
::

	AT+PRODUCTID=<“productid”>

Return:
::

	OK

**Parameter**

- <productid>: product ID, maximum length: 64 bytes.


--------------------------------------------------------------------
AT+DEVICENAME: set/query device name
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+DEVICENAME?

Return:
::

	+DEVICENAME: <“devicename”>
	OK

**Execution command**

Command:
::

	AT+DEVICENAME=<“devicename”>

Return:
::

	OK

**Parameter**

- <devicename>: device name, maximum length: 64 bytes.


--------------------------------------------------------------------
AT+ REGION: set/query the country where the device is located
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+REGION?

Return:
::

	+REGION: <”country”>
	OK

**Execution command**

Command:
::

	AT+REGION=<”country”>

Return:
::

	OK

**Parameter**

- <REGION>: The country where the device is located, maximum length: 64 bytes.


--------------------------------------------------------------------
AT+WORKMODE: set/query the device operating mode
--------------------------------------------------------------------

**Query command**

Command:
::

	AT+WORKMODE?

Return:
::

	+WORKMODE: <mode>
	OK

**Execution command**

Command:
::

	AT+WORKMODE=<mode>

Return:
::

	OK

**Parameter**

- <mode>:
	+ 0: factory mode
	+ 1: normal mode, only supports AT commands

**Note**

- The factory mode, with non-AT commands, supports kernel printing, RF parameter tuning, and other functions. The normal mode is the normal operating mode of the device. This mode only supports printing related to AT commands.
