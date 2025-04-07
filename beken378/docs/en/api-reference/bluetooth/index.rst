
:link_to_translation:`zh_CN:[中文]`

Bluetooth LE APIs
=====================

----------------------------
Introduction to Bluetooth LE
----------------------------
The BK7238 uses a Bluetooth LE 5.2 protocol stack that supports both master and slave modes.

----------------------------
Bluetooth LE API category
----------------------------

Bluetooth LE APIs can be divided into:

 - Bluetooth LE common interface
 - Bluetooth LE scan interface
 - Bluetooth LE ADV interface
 - Bluetooth LE connect interface

----------------------------------------
Programming advice
----------------------------------------

.. important::
   - Some special considerations when using Bluetooth LE APIs:
	   1. Don’t do too much work in the Bluetooth LE event callback. You need to throw the event to your own application task processing through semaphores and other methods.
	   2. The BK7238 Bluetooth LE APIs are executed asynchronously. When the application calls the BLE API, it needs to obtain the corresponding BLE event notification.


----------------------------------------
API reference
----------------------------------------
.. include:: ../../_build/inc/ble_api_5_x.inc



