#!/bin/bash
# Pick the lib files according to the HW platform
# example:
# pick_beken_libs.sh bk7231u [SDK path]

if [ "$2" == "" ]; then
	BEKEN_SDK_DIR=./beken378
else
	BEKEN_SDK_DIR=$2
fi
if [ ! -d ${BEKEN_SDK_DIR} ]; then
	echo "${BEKEN_SDK_DIR}: No such directory"
	exit 1
fi
echo "SDK DIR: ${BEKEN_SDK_DIR}"

IP_LIB_FILE=${BEKEN_SDK_DIR}/lib/librwnx.a
BLE_LIB_FILE=${BEKEN_SDK_DIR}/lib/libble.a
USB_LIB_FILE=${BEKEN_SDK_DIR}/lib/libusb.a
CAL_LIB_FILE=${BEKEN_SDK_DIR}/lib/libcal.a
SUPPLICANT_LIB_FILE=${BEKEN_SDK_DIR}/lib/libsupplicant.a
UART_DEBUG_LIB_FILE=${BEKEN_SDK_DIR}/lib/libuart_debug.a
RF_TEST_LIB_FILE=${BEKEN_SDK_DIR}/lib/librf_test.a
RF_USE_LIB_FILE=${BEKEN_SDK_DIR}/lib/librf_use.a

IP_LIB_ORIG=${BEKEN_SDK_DIR}/lib/librwnx_${1}.a
BLE_LIB_ORIG=${BEKEN_SDK_DIR}/lib/libble_${1}.a
USB_LIB_ORIG=${BEKEN_SDK_DIR}/lib/libusb_${1}.a
CAL_LIB_ORIG=${BEKEN_SDK_DIR}/lib/libcal_${1}.a
SUPPLICANT_LIB_ORIG=${BEKEN_SDK_DIR}/lib/libsupplicant_${1}.a
UART_DEBUG_LIB_ORIG=${BEKEN_SDK_DIR}/lib/libuart_debug_${1}.a
RF_TEST_LIB_ORIG=${BEKEN_SDK_DIR}/lib/librf_test_${1}.a
RF_USE_LIB_ORIG=${BEKEN_SDK_DIR}/lib/librf_use_${1}.a

if [ -f ${IP_LIB_ORIG} ]; then
	if [ -f ${IP_LIB_FILE} ]; then
		rm -f ${IP_LIB_FILE}
	fi
	cp -f ${IP_LIB_ORIG} ${IP_LIB_FILE}
	echo "cp ${IP_LIB_ORIG} to ${IP_LIB_FILE}"
fi

if [ -f ${BLE_LIB_ORIG} ]; then
	if [ -f ${BLE_LIB_FILE} ]; then
		rm -f ${BLE_LIB_FILE}
	fi
	cp -f ${BLE_LIB_ORIG} ${BLE_LIB_FILE}
	echo "cp ${BLE_LIB_ORIG} to ${BLE_LIB_FILE}"
fi

if [ -f ${USB_LIB_ORIG} ]; then
	if [ -f ${USB_LIB_FILE} ]; then
		rm -f ${USB_LIB_FILE}
	fi
	cp -f ${USB_LIB_ORIG} ${USB_LIB_FILE}
	echo "cp ${USB_LIB_ORIG} to ${USB_LIB_FILE}"
fi

if [ -f ${CAL_LIB_ORIG} ]; then
	if [ -f ${CAL_LIB_FILE} ]; then
		rm -f ${CAL_LIB_FILE}
	fi
	cp -f ${CAL_LIB_ORIG} ${CAL_LIB_FILE}
	echo "cp ${CAL_LIB_ORIG} to ${CAL_LIB_FILE}"
fi

if [ -f ${SUPPLICANT_LIB_ORIG} ]; then
	if [ -f ${SUPPLICANT_LIB_FILE} ]; then
		rm -f ${SUPPLICANT_LIB_FILE}
	fi
	cp -f ${SUPPLICANT_LIB_ORIG} ${SUPPLICANT_LIB_FILE}
	echo "cp ${SUPPLICANT_LIB_ORIG} to ${SUPPLICANT_LIB_FILE}"
fi

if [ -f ${UART_DEBUG_LIB_ORIG} ]; then
	if [ -f ${UART_DEBUG_LIB_FILE} ]; then
		rm -f ${UART_DEBUG_LIB_FILE}
	fi
	cp -f ${UART_DEBUG_LIB_ORIG} ${UART_DEBUG_LIB_FILE}
	echo "cp ${UART_DEBUG_LIB_ORIG} to ${UART_DEBUG_LIB_FILE}"
fi

if [ -f ${RF_TEST_LIB_ORIG} ]; then
	if [ -f ${RF_TEST_LIB_FILE} ]; then
		rm -f ${RF_TEST_LIB_FILE}
	fi
	cp -f ${RF_TEST_LIB_ORIG} ${RF_TEST_LIB_FILE}
	echo "cp ${RF_TEST_LIB_ORIG} to ${RF_TEST_LIB_FILE}"
fi

if [ -f ${RF_USE_LIB_ORIG} ]; then
	if [ -f ${RF_USE_LIB_FILE} ]; then
		rm -f ${RF_USE_LIB_FILE}
	fi
	cp -f ${RF_USE_LIB_ORIG} ${RF_USE_LIB_FILE}
	echo "cp ${RF_USE_LIB_ORIG} to ${RF_USE_LIB_FILE}"
fi

