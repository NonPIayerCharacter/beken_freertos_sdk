@echo off
@REM pick the libs according to the platform params
@REM example:
@REM pick_beken_libs.bat bk7231u

set lib_ip_src=beken378\lib\librwnx_%1.a
set lib_ble_src=beken378\lib\libble_%1.a
set lib_usb_src=beken378\lib\libusb_%1.a
set lib_cal_src=beken378\lib\libcal_%1.a
set lib_supplicant_src=beken378\lib\libsupplicant_%1.a
set lib_uart_debug_src=beken378\lib\libuart_debug_%1.a
set lib_rf_test_src=beken378\lib\librf_test_%1.a
set lib_rf_use_src=beken378\lib\librf_use_%1.a

set lib_ip_dst=beken378\lib\librwnx.a
set lib_ble_dst=beken378\lib\libble.a
set lib_usb_dst=beken378\lib\libusb.a
set lib_cal_dst=beken378\lib\libcal.a
set lib_supplicant_dst=beken378\lib\libsupplicant.a
set lib_uart_debug_dst=beken378\lib\libuart_debug.a
set lib_rf_test_dst=beken378\lib\librf_test.a
set lib_rf_use_dst=beken378\lib\librf_use.a

setlocal enabledelayedexpansion

echo "Copying %lib_ip_src% to %lib_ip_dst%"
copy %lib_ip_src% %lib_ip_dst%

echo "Copying %lib_ble_src% to %lib_ble_dst%"
copy %lib_ble_src% %lib_ble_dst%

echo "Copying %lib_usb_src% to %lib_usb_dst%"
copy %lib_usb_src% %lib_usb_dst%

echo "Copying %lib_cal_src% to %lib_cal_dst%"
copy %lib_cal_src% %lib_cal_dst%

echo "Copying %lib_supplicant_src% to %lib_supplicant_dst%"
copy %lib_supplicant_src% %lib_supplicant_dst%

echo "Copying %lib_uart_debug_src% to %lib_uart_debug_dst%"
copy %lib_uart_debug_src% %lib_uart_debug_dst%

echo "Copying %lib_rf_test_src% to %lib_rf_test_dst%"
copy %lib_rf_test_src% %lib_rf_test_dst%

echo "Copying %lib_rf_use_src% to %lib_rf_use_dst%"
copy %lib_rf_use_src% %lib_rf_use_dst%

endlocal
