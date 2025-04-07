
:link_to_translation:`zh_CN:[中文]`

Introduction to Low Power Consumption
=============================================
Power saving is part of power management. We control hardware sleep to reduce the power consumption of the development board to meet longer system life requirements.
The overall idea of power saving is to lower the voltage, lower the clock frequency, and turn off peripherals that are not in use when the system is idle.
The core task of power saving is to control MCU and MAC sleep and wake up the system when data needs to be sent and received.





System clocks
--------------------------------------------
System clock sources include the following:

- X26M: High frequency 26 MHz crystal oscillator
- DCO: Internal high frequency digitally controlled oscillator, 26 MHz to 160 MHz, about ±2% variation after calibration. The startup time of the clock signal is about a few microseconds.
- D32K: 32 kHz clock signal derived from X26M
- ROSC: 32 kHz internal low frequency ring oscillator, about ±2% variation after calibration
- DPLL: High speed 480 MHz PLL clock


System state machine
--------------------------------------------
The system supports different power modes:
 - Active
 - Normal sleep
 - Low-voltage sleep
 - Deep sleep 
 - Shutdown

Active
--------------------------------------------
 - The MCU is in normal operation condition, and Wi-Fi and Bluetooth LE can send and receive data normally.


Normal sleep
--------------------------------------------
 - When the RTOS has no tasks to process, the system enters the IDLE task. In the IDLE task, the MCU will enter sleep and stop running.
 - Any peripheral can continue to work and generate an interrupt to wake up the MCU to continue running, enter the active state again, and work normally.


Low-voltage sleep
--------------------------------------------
 - Low-voltage sleep is a relatively power-saving sleep mode. In this mode, the clocks of the MCU and all digital peripherals are stopped, and the system only has a 32K clock; only some hardware modules are working at this time, and only GPIO interrupts and AON counter interrupts can wake up the system and return to normal voltage to continue running.
 - In low-voltage sleep mode, the voltage of AON will decrease, and the voltage of VDDDIG will also decrease.
 - When the RTOS has no tasks to process, it automatically enters the IDLE task. When the low-voltage conditions are met, it enters the low-voltage state.
 - When the wake-up signal is triggered, the system exits the low-voltage state, and the AON and VDDDIG voltages are upgraded to normal voltages.
 - In a low-voltage state, only wake-up sources (GPIO, RTC) can make the system exit low-voltage mode and return to normal voltage mode.
 - In order to achieve optimal power consumption, please turn off unnecessary modules before entering low-voltage mode. After exiting low-voltage mode, you can turn them on again.
 
Deep sleep
--------------------------------------------
 - Deep sleep is the most power-saving sleep mode. In this mode, the system only has a 32K clock, and only some hardware modules are working at this time. Except for the AON module, other hardware modules have been powered off. When the GPIO interrupt or RTC timeout interrupt wake-up signal is triggered, the system exits the deep sleep state and resets.

 - In deep sleep state, only wake-up sources (GPIO, RTC) can make the system exit deep sleep.
 
 - The 32K clock source in this mode defaults to ROSC.


Shutdown
--------------------------------------------
 - The entire system circuit is powered off

