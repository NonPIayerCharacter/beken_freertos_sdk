#! /bin/bash

# Clean up the sourcce files after libs are generated
# example:
# generate_beken_lib.sh bk7231u [SDK_path]

source ./tools/scripts/build_include.sh

if [ "$2" == "" ]; then
	BEKEN_SDK_DIR=./beken378
else
	BEKEN_SDK_DIR=$2
fi

IP_LIB_FILE=${BEKEN_SDK_DIR}/lib/librwnx.a
BLE_LIB_FILE=${BEKEN_SDK_DIR}/lib/libble.a
USB_LIB_FILE=${BEKEN_SDK_DIR}/lib/libusb.a
CAL_LIB_FILE=${BEKEN_SDK_DIR}/lib/libcal.a
SUPPLICANT_LIB_FILE=${BEKEN_SDK_DIR}/lib/libsupplicant.a
UART_DEBUG_LIB_FILE=${BEKEN_SDK_DIR}/lib/libuart_debug.a
RF_TEST_LIB_FILE=${BEKEN_SDK_DIR}/lib/librf_test.a
RF_USE_LIB_FILE=${BEKEN_SDK_DIR}/lib/librf_use.a

IP_LIB_STORE=${BEKEN_SDK_DIR}/lib/librwnx_${1}.a
BLE_LIB_STORE=${BEKEN_SDK_DIR}/lib/libble_${1}.a
USB_LIB_STORE=${BEKEN_SDK_DIR}/lib/libusb_${1}.a
CAL_LIB_STORE=${BEKEN_SDK_DIR}/lib/libcal_${1}.a
SUPPLICANT_LIB_STORE=${BEKEN_SDK_DIR}/lib/libsupplicant_${1}.a
UART_DEBUG_LIB_STORE=${BEKEN_SDK_DIR}/lib/libuart_debug_${1}.a
RF_TEST_LIB_STORE=${BEKEN_SDK_DIR}/lib/librf_test_${1}.a
RF_USE_LIB_STORE=${BEKEN_SDK_DIR}/lib/librf_use_${1}.a

echo "making ${1} libs..."

SYS_CFG_FILE=${BEKEN_SDK_DIR}/app/config/sys_config_${1}.h
if [ ! -f ${SYS_CFG_FILE} ]; then
	echo "${SYS_CFG_FILE}: no such file!"
	exit 1
fi
echo "backup ${SYS_CFG_FILE}"
cp -f ${SYS_CFG_FILE} ${SYS_CFG_FILE}.bak

case $1 in
	bk7231u)
		modify_config ${SYS_CFG_FILE} CFG_SUPPORT_BLE 1
		modify_config ${SYS_CFG_FILE} CFG_WIFI_SENSOR 1
		modify_config ${SYS_CFG_FILE} CFG_BK_AWARE    1
		make_target="ip ble sensor bk_aware cal supplicant uart_debug rf_test rf_use"
		;;
	bk7231n)
		modify_config ${SYS_CFG_FILE} CFG_SUPPORT_BLE 1
		modify_config ${SYS_CFG_FILE} CFG_WIFI_SENSOR 1
		modify_config ${SYS_CFG_FILE} CFG_BK_AWARE    1
		make_target="ip ble sensor bk_aware  cal supplicant uart_debug rf_test rf_use"
		;;
	bk7251)
		modify_config ${SYS_CFG_FILE} CFG_SUPPORT_BLE 1
		modify_config ${SYS_CFG_FILE} CFG_WIFI_SENSOR 1
		modify_config ${SYS_CFG_FILE} CFG_BK_AWARE    1
		modify_config ${SYS_CFG_FILE} CFG_USB 1
		make_target="ip ble usb sensor bk_aware cal supplicant uart_debug rf_test rf_use"
		;;
	bk7271)
		modify_config ${SYS_CFG_FILE} CFG_SUPPORT_BLE 0
		modify_config ${SYS_CFG_FILE} CFG_WIFI_SENSOR 1
		modify_config ${SYS_CFG_FILE} CFG_USB 1
		make_target="ip ble usb sensor cal supplicant uart_debug rf_test rf_use"
		;;
	bk7238)
		modify_config ${SYS_CFG_FILE} CFG_SUPPORT_BLE 1
		modify_config ${SYS_CFG_FILE} CFG_WIFI_SENSOR 1
		modify_config ${SYS_CFG_FILE} CFG_BK_AWARE    1
		make_target="ip ble sensor bk_aware cal supplicant uart_debug rf_test rf_use"
		;;
	*)
		echo "unsupported platform"
		exit 1
		;;
esac

echo "make clean/cleanlib..."
make clean > /dev/null
make cleanlib > /dev/null

./tools/scripts/generate_sys_config.sh ${1}
if [ $? != 0 ]; then
	echo "generate sys config fail"
	exit 1
fi

for sub in ${make_target}
do
	echo "making lib ${sub}..."
	make $sub -j8
	if [ $? != 0 ]; then
		echo "make lib $sub fail"
		exit 1
	fi
done

if [ -f ${IP_LIB_FILE} ]; then
	mv -f ${IP_LIB_FILE} ${IP_LIB_STORE}
	echo "mv ${IP_LIB_FILE} to ${IP_LIB_STORE}"
fi

if [ -f ${BLE_LIB_FILE} ]; then
	mv -f ${BLE_LIB_FILE} ${BLE_LIB_STORE}
	echo "mv ${BLE_LIB_FILE} to ${BLE_LIB_STORE}"
fi

if [ -f ${USB_LIB_FILE} ]; then
	mv -f ${USB_LIB_FILE} ${USB_LIB_STORE}
	echo "mv ${USB_LIB_FILE} to ${USB_LIB_STORE}"
fi

if [ -f ${CAL_LIB_FILE} ]; then
	mv -f ${CAL_LIB_FILE} ${CAL_LIB_STORE}
	echo "mv ${CAL_LIB_FILE} to ${CAL_LIB_STORE}"
fi

if [ -f ${SUPPLICANT_LIB_FILE} ]; then
	mv -f ${SUPPLICANT_LIB_FILE} ${SUPPLICANT_LIB_STORE}
	echo "mv ${SUPPLICANT_LIB_FILE} to ${SUPPLICANT_LIB_STORE}"
fi

if [ -f ${UART_DEBUG_LIB_FILE} ]; then
	mv -f ${UART_DEBUG_LIB_FILE} ${UART_DEBUG_LIB_STORE}
	echo "mv ${UART_DEBUG_LIB_FILE} to ${UART_DEBUG_LIB_STORE}"
fi

if [ -f ${RF_TEST_LIB_FILE} ]; then
	mv -f ${RF_TEST_LIB_FILE} ${RF_TEST_LIB_STORE}
	echo "mv ${RF_TEST_LIB_FILE} to ${RF_TEST_LIB_STORE}"
fi

if [ -f ${RF_USE_LIB_FILE} ]; then
	mv -f ${RF_USE_LIB_FILE} ${RF_USE_LIB_STORE}
	echo "mv ${RF_USE_LIB_FILE} to ${RF_USE_LIB_STORE}"
fi

echo "restore ${SYS_CFG_FILE}"
rm -f ${SYS_CFG_FILE}
mv -f ${SYS_CFG_FILE}.bak ${SYS_CFG_FILE}
