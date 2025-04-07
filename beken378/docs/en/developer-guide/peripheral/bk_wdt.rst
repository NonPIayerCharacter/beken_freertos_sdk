
:link_to_translation:`zh_CN:[中文]`

WDT user guide
=====================


Overview
-----------------

Watchdog is actually a hardware timer, and the timer expiration will trigger the system watchdog reset.

- bk_wdg_initialize(timeout_ms) can set the watchdog timer time period. Currently the time period configuration range is 1 to 0xFFFF.
- bk_wdg_reload() resets the watchdog timer.


Watchdog usage
------------------

There are two common ways to use watchdog:

 - Feeding the dog during periodic interrupts, such as feeding the dog during the OS tick interrupt. When the tick interrupt fails to feed the dog in time due to software problems, such as the shutdown interrupt being too long, causing the watchdog to time out, eventually triggering the watchdog to restart. In this way, abnormal software interruption problems can be discovered in time. This type of watchdog use is often called interrupt watchdog.
 - Feed the dog in a periodic task, such as feeding the dog in an idle task. Usually the idle task is the task with the lowest priority in the entire system. When there is a problem with the software, such as a certain task having an infinite loop that occupies the CPU for a long time, making the idle task unable to Scheduling, resulting in failure to feed the dog in time, eventually triggering watchdog restart. In this way, problems in software task scheduling can be discovered in time. This type of watchdog use is often called task watchdog.

Interrupt watchdog
-----------------------------

Interrupt watchdog is enabled by default in the BK7238, and can be enabled/disabled through CFG_INT_WDG_ENABLED in sys_config_bk7238.h. Generally, applications should not disable interrupt watchdog. CFG_INT_WDG_PERIOD_MS can be used to set the timeout period.

.. note::

  When the application needs to implement the interrupt watchdog completely by itself, please disable the system default interrupt watchdog through the CFG_INT_WDG_ENABLED macro.


Task watchdog
---------------------

Task watchdog is also implemented in the BK7238, but it is not based on real hardware watchdog. Task watchdog can be enabled/disabled through through CFG_TASK_WDG_ENABLED and the timeout period can be set through CFG_TASK_WDG_PERIOD_MS in sys_config_bk7238.h. The specific implementation method is as follows:

The global variable s_last_task_wdt_feed_tick represents the last dog feeding time. Update s_last_task_wdt_feed_tick in the idle task. In the tick interrupt, it is judged whether the last dog feeding time exceeds the task watchdog timeout. If it times out, a task watchdog timeout warning message will be displayed, but the system will not be restarted.




