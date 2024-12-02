低功耗简介
=============================================
低功耗（Power Save）是电源管理（Power Manage）的一部分，我们通过控制硬件睡眠来降低开发板功耗以满足更长的系统续航要求。
低功耗的整体思路就是在系统空闲的时候降低电压，降低时钟频率，关闭暂时不用的外设。
低功耗的核心任务是控制MCU和MAC睡眠，并在需要收发数据的时候唤醒系统。




系统时钟
--------------------------------------------
系统时钟源，有以下几种：

- X26M: High frequency 26 MHz crystal oscillator
- DCO: Internal high frequency digitally controlled oscillator, 26 MHz to 160 MHz, about ±2% variation after calibration. The startup time of the clock signal is about a few microseconds.
- D32K: 32 kHz clock signal derived from X26M
- ROSC: 32 kHz internal low frequency ring oscillator, about ±2% variation after calibration
- DPLL: High speed 480 MHz PLL clock


系统状态机
--------------------------------------------
系统支持不同的睡眠模式:
 - active
 - normal sleep
 - low voltage
 - deepsleep
 - shut down

active(正常工作)
--------------------------------------------
 - MCU处于正常工作状态,WIFI,BLE可以正常收发数据。


normal sleep(普通睡眠)
--------------------------------------------
 - 当RTOS没有任务需要处理时，系统进入IDLE任务中，在IDLE任务中，MCU会进入睡眠，停止运行
 - 任何外设都可以继续工作，并产生中断唤醒MCU继续运行，再次进入active 状态，正常工作。


low voltage(低压睡眠)
--------------------------------------------
 - 低压睡眠是一种相对比较节省功耗的睡眠模式。在该模式下，MCU和所有数字外设的时钟都被停止，系统只有32K时钟；此时只有部分硬件模块在工作，仅有GPIO中断和AON计数器中断可以唤醒系统恢复到正常电压继续运行。
 - 低压睡眠模式AON的电压会减低，VDDDIG的电压也会减低。
 - 当RTOS没有任务处理时，自动进入IDLE任务，当满足了低压条件，进入进入低压状态。
 -  当唤醒信号触发后，系统退出低压状态，AON,VDDDIG电压升级到正常电压。
 - 处于低压状态下，只有唤醒源(GPIO,RTC)可以让系统退出低压，恢复到正常电压模式。
 - 为了达到最优功耗，不需要的模块，进入低压前请关闭，退出低压后，可以再开启。

deep sleep(深度睡眠)
--------------------------------------------
 - 深度睡眠是一种相对最节省功耗的睡眠模式。在该模式下系统只有32K时钟，此时只有部分硬件模块在工作，除了AON模块其他硬件模块都已经下电。当GPIO 中断 或者RTC 超时中断唤醒信号触发后，系统退出深度睡眠状态，系统复位。

 - 处于深度睡眠状态下，只有唤醒源(GPIO,RTC)可以让系统退出深度睡眠。
 
 - 该模式下32K的时钟源默认是使用ROSC的32K。


shut down(关机)
--------------------------------------------
 - 整个系统电路都已经下电

