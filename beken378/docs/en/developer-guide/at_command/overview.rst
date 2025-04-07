
:link_to_translation:`zh_CN:[中文]`

Overview
=======================================================

BK7238 general AT commands integrate functions commonly used by users, aiming to lower the threshold for customers to use products and improve the efficiency of products from research and development to mass production. The AT instruction set provides two types of tests, query instructions and execution instructions to meet the needs of various scenarios.

.. note::
    1. Currently, AT commands are only supported on FreeRTOS.
    2. Before using the AT command, please confirm whether the AT_SERVICE_CFG macro is 1. The SDK is generally disabled by default.
    3. After the AT command is turned on, there will be less information printed when turning on the computer than before, but the initialization process is the same.
