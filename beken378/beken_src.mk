
WPA_VERSION := wpa_supplicant_2_9

AT_SERVICE_CFG ?= 0
ifeq ($(AT_SERVICE_CFG),1)
ATSVR_CFG ?= 1
else
ATSVR_CFG ?= 0
endif

ifeq ($(CFG_SUPPORT_MATTER), 1)
LWIP_VERSION := lwip-2.1.2
else
LWIP_VERSION := lwip-2.0.2
endif

# -------------------------------------------------------------------
# Include folder list
# -------------------------------------------------------------------

INCLUDES += -I$(ROOT_DIR)/beken378/common
INCLUDES += -I$(ROOT_DIR)/beken378/release
INCLUDES += -I$(ROOT_DIR)/beken378/demo
INCLUDES += -I$(ROOT_DIR)/beken378/app
INCLUDES += -I$(ROOT_DIR)/beken378/app/config
INCLUDES += -I$(ROOT_DIR)/beken378/app/standalone-station
INCLUDES += -I$(ROOT_DIR)/beken378/app/standalone-ap
INCLUDES += -I$(ROOT_DIR)/beken378/app/video_work
INCLUDES += -I$(ROOT_DIR)/beken378/app/net_work
INCLUDES += -I$(ROOT_DIR)/beken378/ip/common
INCLUDES += -I$(ROOT_DIR)/beken378/ip/ke/
INCLUDES += -I$(ROOT_DIR)/beken378/ip/mac/
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/hal
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/mm
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/ps
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/rd
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/rwnx
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/rx
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/scan
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/sta
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/tx
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/vif
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/rx/rxl
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/tx/txl
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/p2p
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/chan
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/td
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/tpc
INCLUDES += -I$(ROOT_DIR)/beken378/ip/lmac/src/tdls
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/mesh
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/mfp
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/rc
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/apm
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/bam
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/ftm
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/llc
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/me
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/rxu
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/scanu
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/sm
INCLUDES += -I$(ROOT_DIR)/beken378/ip/umac/src/txu
INCLUDES += -I$(ROOT_DIR)/beken378/driver/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/common/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/entry
INCLUDES += -I$(ROOT_DIR)/beken378/driver/dma
INCLUDES += -I$(ROOT_DIR)/beken378/driver/intc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/phy
INCLUDES += -I$(ROOT_DIR)/beken378/driver/pwm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/rc_beken
INCLUDES += -I$(ROOT_DIR)/beken378/driver/flash
INCLUDES += -I$(ROOT_DIR)/beken378/driver/rw_pub
INCLUDES += -I$(ROOT_DIR)/beken378/driver/common/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/common
INCLUDES += -I$(ROOT_DIR)/beken378/driver/uart
INCLUDES += -I$(ROOT_DIR)/beken378/driver/sys_ctrl
INCLUDES += -I$(ROOT_DIR)/beken378/driver/gpio
INCLUDES += -I$(ROOT_DIR)/beken378/driver/general_dma
INCLUDES += -I$(ROOT_DIR)/beken378/driver/spidma
INCLUDES += -I$(ROOT_DIR)/beken378/driver/icu
INCLUDES += -I$(ROOT_DIR)/beken378/driver/i2c
INCLUDES += -I$(ROOT_DIR)/beken378/driver/spi
INCLUDES += -I$(ROOT_DIR)/beken378/driver/jpeg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb
INCLUDES += -I$(ROOT_DIR)/beken378/func/include
INCLUDES += -I$(ROOT_DIR)/beken378/func/ble_wifi_exchange
INCLUDES += -I$(ROOT_DIR)/beken378/func/rf_test
INCLUDES += -I$(ROOT_DIR)/beken378/func/user_driver
INCLUDES += -I$(ROOT_DIR)/beken378/func/power_save
INCLUDES += -I$(ROOT_DIR)/beken378/func/uart_debug
INCLUDES += -I$(ROOT_DIR)/beken378/func/ethernet_intf
INCLUDES += -I$(ROOT_DIR)/beken378/func/camera_intf
INCLUDES += -I$(ROOT_DIR)/beken378/func/video_transfer
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/hostapd
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/bk_patch
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src/utils
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src/ap
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src/common
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src/drivers
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/src/wps
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/wpa_supplicant
INCLUDES += -I$(ROOT_DIR)/beken378/func/$(WPA_VERSION)/bk_patch
INCLUDES += -I$(ROOT_DIR)/beken378/func/lwip_intf/lwip-2.0.2/port
INCLUDES += -I$(ROOT_DIR)/beken378/func/lwip_intf/$(LWIP_VERSION)/src
INCLUDES += -I$(ROOT_DIR)/beken378/func/lwip_intf/$(LWIP_VERSION)/src/include
INCLUDES += -I$(ROOT_DIR)/beken378/func/lwip_intf/$(LWIP_VERSION)/src/include/netif
INCLUDES += -I$(ROOT_DIR)/beken378/func/lwip_intf/$(LWIP_VERSION)/src/include/lwip
INCLUDES += -I$(ROOT_DIR)/beken378/func/temp_detect
INCLUDES += -I$(ROOT_DIR)/beken378/func/spidma_intf
INCLUDES += -I$(ROOT_DIR)/beken378/func/saradc_intf
INCLUDES += -I$(ROOT_DIR)/beken378/func/rwnx_intf
INCLUDES += -I$(ROOT_DIR)/beken378/func/joint_up
INCLUDES += -I$(ROOT_DIR)/beken378/func/base64
INCLUDES += -I$(ROOT_DIR)/beken378/func/easy_flash
INCLUDES += -I$(ROOT_DIR)/beken378/func/easy_flash/inc
INCLUDES += -I$(ROOT_DIR)/beken378/func/easy_flash/port
INCLUDES += -I$(ROOT_DIR)/beken378/func/rf_use
INCLUDES += -I$(ROOT_DIR)/beken378/func/usb
INCLUDES += -I$(ROOT_DIR)/beken378/func/misc
INCLUDES += -I$(ROOT_DIR)/beken378/func/sensor
INCLUDES += -I$(ROOT_DIR)/beken378/os/include
INCLUDES += -I$(ROOT_DIR)/beken378/os/FreeRTOSv9.0.0
INCLUDES += -I$(ROOT_DIR)/beken378/func/utf8
INCLUDES += -I$(ROOT_DIR)/beken378/app/http
INCLUDES += -I$(ROOT_DIR)/beken378/func/force_sleep
INCLUDES += -I$(ROOT_DIR)/os/FreeRTOSv9.0.0/FreeRTOS/Source
ifeq ($(CFG_BK_AWARE),1)
INCLUDES += -I$(ROOT_DIR)/beken378/func/bk_aware
endif
ifeq ($(CFG_SUPPORT_MATTER), 1)
INCLUDES += -I$(ROOT_DIR)/beken378/func/key_value_flash
endif

ifeq ($(CFG_USE_SDCARD_HOST),1)
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/msc
INCLUDES += -I$(ROOT_DIR)/beken378/func/fatfs
endif

# For BK7251
ifeq ($(CFG_SOC_NAME), 3)
INCLUDES += -I$(ROOT_DIR)/beken378/driver/audio
INCLUDES += -I$(ROOT_DIR)/beken378/driver/sdcard
endif

# For WPA3
# if WPA3 enabled or non-tls-internal enabled for EAP
ifeq ($(CFG_WPA3),1)
ifneq ("${CFG_USE_MBEDTLS}", "1")
INCLUDES += -I$(ROOT_DIR)/beken378/func/wolfssl
else ifeq ($(CFG_WPA2_ENTERPRISE),1)
ifeq ($(CFG_WPA_TLS_WOLFSSL),1)
INCLUDES += -I$(ROOT_DIR)/beken378/func/wolfssl
endif # CFG_WPA_TLS_WOLFSSL
endif # CFG_USE_MBEDTLS
endif # CFG_WPA3

#paho-mqtt
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/client
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/client/src
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/packet/src
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/mqtt_ui
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/mqtt_ui/ssl_mqtt
INCLUDES += -I$(ROOT_DIR)/beken378/func/paho-mqtt/mqtt_ui/tcp_mqtt

ifeq ($(CFG_SUPPORT_BLE),1)
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_4_2))
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/controller/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/hci/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/host/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/sys/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/config
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/modules/app/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/modules/gernel_api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/modules/mesh_model/ali
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/arch
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/driver/ble_icu
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/driver/ir
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/driver/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/driver/sys_ctrl
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/driver/uart
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/plactform/modules/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/profiles/comm/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/profiles/prf/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/profiles/sdp/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/dbg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/include
endif
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_1))
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/inc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gap/gapc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gap/gapm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt/attc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt/attm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt/atts
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt/gattc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/gatt/gattm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/l2c/l2cc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/hl/src/l2c/l2cm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/import/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/src/llc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/src/lld
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/ble/ll/src/llm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/em/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/hci/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/hci/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/sch/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/ip/sch/import
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/aes/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/aes/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/common/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/dbg/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/dbg/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/ecc_p256/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/h4tl/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/ke/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_lib/modules/ke/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_pub/prf
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/rwip/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/rwip/import/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/nvds/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/config
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/driver/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/driver/rf
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/driver/uart
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/platform/7231n/entry
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/arch/armv5
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/arch/armv5/ll
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/arch/armv5/compiler
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_pub/profiles/comm/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_pub/profiles/sdp/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_pub/app/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_1/ble_pub/ui
endif
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_2))
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/arch/armv5
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/arch/armv5/compiler
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/arch/armv5/ll
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ahi/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/inc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/gap/gapc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/gap/gapm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/gap
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/gatt
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/inc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/hl/src/l2cap
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/iso/data_path
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/iso/data_path/isogen/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/iso/data_path/isogen/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/iso/data_path/isoohci/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/iso/data_path/isoohci/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/import/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/src/llc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/src/lld
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/src/lli
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/ble/ll/src/llm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/em/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/hci/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/hci/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/sch/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/ip/sch/import
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/aes/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/aes/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/common/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/dbg/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/ecc_p256/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/h4tl/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/ke/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/ke/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/rwip/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/rwip/import/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_lib/modules/rwip/src
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/app/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/bk_comm/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/bas/bass/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/hogp/
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/hogp/hogpd/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/find/findt/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/dis/diss/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/profiles/bk_sdp/api
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/ble_pub/ui
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/config
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/driver/reg
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/driver/rf
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/driver/uart
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/entry
INCLUDES += -I$(ROOT_DIR)/beken378/driver/ble/ble_5_2/platform/7238/nvds/api
endif
endif

#usb module
#ifeq ($(CFG_USB),1)
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/include
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/include/class
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/cd
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/comm
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/hid
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/msd
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/compl
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/hub
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/drivers/trans
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/example/msd
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/hid
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/lib
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/msc
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/systems/none/afs
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/systems/none
INCLUDES += -I$(ROOT_DIR)/beken378/driver/usb/src/uvc
#endif

ifeq ("${CFG_MBEDTLS}", "1")
#CFG_DEFINE_INCLUDE += MBEDTLS_CONFIG_FILE=\"tls_config.h\"
ifeq ($(CFG_SUPPORT_MATTER), 1)
INCLUDES += -I$(ROOT_DIR)/beken378/func/mbedtls/mbedtls-2.27.0/include
else
INCLUDES += -I$(ROOT_DIR)/beken378/func/mbedtls/mbedtls/include
INCLUDES += -I$(ROOT_DIR)/beken378/func/mbedtls/mbedtls/include/mbedtls
INCLUDES += -I$(ROOT_DIR)/beken378/func/mbedtls/mbedtls_ui/
endif
INCLUDES += -I$(ROOT_DIR)/beken378/func/mbedtls/mbedtls-port/inc
endif


ifeq ($(ATSVR_CFG),1)
#INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/include
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/mqtt
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/_at_server
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/_at_server_port
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/at_server_func
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/atsvr_cmd
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/network
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/atsvr_net_cmd
INCLUDES += -I$(ROOT_DIR)/beken378/func/at_server/atsvr_airkiss_cmd
endif
INCLUDES += -I$(ROOT_DIR)/beken378/func/ntp
INCLUDES += -I$(ROOT_DIR)/beken378/func/rtc

# -------------------------------------------------------------------
# Source file list
# -------------------------------------------------------------------

#application layer
SRC_C += ./beken378/app/app.c
SRC_C += ./beken378/app/ate_app.c
SRC_C += ./beken378/app/config/param_config.c
SRC_C += ./beken378/app/standalone-ap/sa_ap.c
SRC_C += ./beken378/app/standalone-station/sa_station.c
SRC_C += ./beken378/app/video_work/video_transfer_tcp.c
SRC_C += ./beken378/app/video_work/video_transfer_udp.c
SRC_C += ./beken378/app/video_work/video_buffer.c
SRC_C += ./beken378/app/video_work/video_upd_spd.c
SRC_C += ./beken378/app/video_work/video_upd_spd_pub.c
SRC_C += ./beken378/app/net_work/video_demo_main.c
SRC_C += ./beken378/app/net_work/video_demo_station.c
SRC_C += ./beken378/app/net_work/video_demo_softap.c
SRC_C += ./beken378/app/net_work/video_demo_p2p.c


#demo module
SRC_C += ./beken378/demo/ieee802_11_demo.c

#driver layer
SRC_DRV_C += ./beken378/driver/common/dd.c
SRC_DRV_C += ./beken378/driver/common/drv_model.c
SRC_DRV_C += ./beken378/driver/dma/dma.c
SRC_DRV_C += ./beken378/driver/driver.c
SRC_DRV_C += ./beken378/driver/entry/arch_main.c
SRC_DRV_C += ./beken378/driver/fft/fft.c
SRC_DRV_C += ./beken378/driver/flash/flash.c
SRC_DRV_C += ./beken378/driver/general_dma/general_dma.c
SRC_DRV_C += ./beken378/driver/gpio/gpio.c
SRC_DRV_C += ./beken378/driver/i2s/i2s.c
SRC_DRV_C += ./beken378/driver/icu/icu.c
SRC_DRV_C += ./beken378/driver/intc/intc.c
SRC_DRV_C += ./beken378/driver/irda/irda.c
SRC_DRV_C += ./beken378/driver/macphy_bypass/mac_phy_bypass.c
SRC_DRV_C += ./beken378/driver/phy/phy_trident.c
SRC_DRV_C += ./beken378/driver/pwm/pwm.c
SRC_DRV_C += ./beken378/driver/pwm/pwm_bk7231n.c
SRC_DRV_C += ./beken378/driver/pwm/mcu_ps_timer.c
SRC_DRV_C += ./beken378/driver/pwm/bk_timer.c
SRC_DRV_C += ./beken378/driver/pwm/bk_timer_extense.c
SRC_DRV_C += ./beken378/driver/pwm/pwm_mutex.c
SRC_DRV_C += ./beken378/driver/qspi/qspi.c
SRC_DRV_C += ./beken378/driver/rw_pub/rw_platf_pub.c
SRC_DRV_C += ./beken378/driver/saradc/saradc.c
SRC_DRV_C += ./beken378/driver/saradc/saradc_bk7238.c
SRC_DRV_C += ./beken378/driver/spidma/spidma.c
SRC_DRV_C += ./beken378/driver/sys_ctrl/sys_ctrl.c
SRC_DRV_C += ./beken378/driver/uart/Retarget.c
SRC_DRV_C += ./beken378/driver/uart/uart.c
SRC_DRV_C += ./beken378/driver/uart/printf.c
SRC_DRV_C += ./beken378/driver/wdt/wdt.c
SRC_DRV_C += ./beken378/driver/calendar/calendar.c
SRC_DRV_C += ./beken378/driver/security/security.c
SRC_DRV_C += ./beken378/driver/security/hal_aes.c
SRC_DRV_C += ./beken378/driver/security/hal_sha.c
SRC_DRV_C += ./beken378/driver/jpeg/jpeg_encoder.c
SRC_DRV_C += ./beken378/driver/i2c/i2c1.c
SRC_DRV_C += ./beken378/driver/i2c/i2c2.c

ifeq ($(CFG_SDIO),1)
SRC_DRV_C += ./beken378/driver/sdio/sdio.c
SRC_DRV_C += ./beken378/driver/sdio/sdma.c
SRC_DRV_C += ./beken378/driver/sdio/sutil.c
endif

#function layer
SRC_FUNC_C += ./beken378/func/func.c
SRC_FUNC_C += ./beken378/func/usb_plug/usb_plug.c
SRC_FUNC_C += ./beken378/func/security/security_func.c
SRC_FUNC_C += ./beken378/func/joint_up/role_launch.c
SRC_C += ./beken378/app/http/utils_httpc.c
SRC_C += ./beken378/app/http/utils_net.c
SRC_C += ./beken378/app/http/utils_timer.c
SRC_C += ./beken378/app/http/lite-log.c
SRC_FUNC_C += ./beken378/func/ntp/ntp.c
SRC_FUNC_C += ./beken378/func/rtc/rtc.c
SRC_FUNC_C += ./beken378/func/rtc/soft_rtc.c
SRC_FUNC_C += ./beken378/func/rtc/rtc_time.c

SRC_WPA_C += ./beken378/func/hostapd_intf/hostapd_intf.c
ifeq ($(CFG_USE_SDCARD_HOST),1)
SRC_FUNC_C += ./beken378/func/fatfs/cc936.c
SRC_FUNC_C += ./beken378/func/fatfs/ccsbcs.c
SRC_FUNC_C += ./beken378/func/fatfs/disk_io.c
SRC_FUNC_C += ./beken378/func/fatfs/driver_udisk.c
SRC_FUNC_C += ./beken378/func/fatfs/ff.c
SRC_FUNC_C += ./beken378/func/fatfs/playmode.c
endif

ifeq ($(CFG_SDIO),1)
SRC_FUNC_C += ./beken378/func/sdio_intf/sdio_intf.c
endif

SRC_LWIP_C = 
SRC_LWIP_C += ./beken378/func/lwip_intf/lwip-2.0.2/port/ethernetif.c
SRC_LWIP_C += ./beken378/func/lwip_intf/lwip-2.0.2/port/net.c
SRC_LWIP_C += ./beken378/func/lwip_intf/lwip-2.0.2/port/sys_arch.c
SRC_LWIP_C += ./beken378/func/lwip_intf/lwip-2.0.2/src/apps/ping/ping.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/api_lib.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/api_msg.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/err.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/netbuf.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/netdb.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/netifapi.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/sockets.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/api/tcpip.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/def.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/dns.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/inet_chksum.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/init.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ip.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/dhcp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/etharp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/icmp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/igmp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/ip4_addr.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/ip4.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv4/ip4_frag.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/dhcp6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/ethip6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/icmp6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/inet6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/ip6_addr.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/ip6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/ip6_frag.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/mld6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/ipv6/nd6.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/mem.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/memp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/netif.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/pbuf.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/raw.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/stats.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/sys.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/tcp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/tcp_in.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/tcp_out.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/timeouts.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/core/udp.c
SRC_LWIP_C += ./beken378/func/lwip_intf/$(LWIP_VERSION)/src/netif/ethernet.c
SRC_LWIP_C += ./beken378/func/lwip_intf/dhcpd/dhcp-server.c
SRC_LWIP_C += ./beken378/func/lwip_intf/dhcpd/dhcp-server-main.c

SRC_FUNC_C += ./beken378/func/misc/fake_clock.c
SRC_FUNC_C += ./beken378/func/misc/pseudo_random.c
SRC_FUNC_C += ./beken378/func/misc/target_util.c
SRC_FUNC_C += ./beken378/func/misc/start_type.c
SRC_FUNC_C += ./beken378/func/misc/soft_encrypt.c
SRC_FUNC_C += ./beken378/func/misc/flash_bypass.c
SRC_FUNC_C += ./beken378/func/power_save/power_save.c
SRC_FUNC_C += ./beken378/func/power_save/manual_ps.c
SRC_FUNC_C += ./beken378/func/power_save/mcu_ps.c
SRC_FUNC_C += ./beken378/func/power_save/ap_idle.c
SRC_FUNC_C += ./beken378/func/saradc_intf/saradc_intf.c
SRC_FUNC_C += ./beken378/func/rwnx_intf/rw_ieee80211.c
SRC_FUNC_C += ./beken378/func/rwnx_intf/rw_msdu.c
SRC_FUNC_C += ./beken378/func/rwnx_intf/rw_tx_buffering.c
SRC_FUNC_C += ./beken378/func/rwnx_intf/rw_msg_rx.c
SRC_FUNC_C += ./beken378/func/rwnx_intf/rw_msg_tx.c
SRC_FUNC_C += ./beken378/func/sim_uart/gpio_uart.c
SRC_FUNC_C += ./beken378/func/sim_uart/pwm_uart.c
SRC_FUNC_C += ./beken378/func/spidma_intf/spidma_intf.c
SRC_FUNC_C += ./beken378/func/temp_detect/temp_detect.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverFlash.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverGpio.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverPwm.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverUart.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverWdg.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverRng.c
SRC_FUNC_C += ./beken378/func/user_driver/BkDriverTimer.c
SRC_FUNC_C += ./beken378/func/wlan_ui/wlan_cli.c
SRC_FUNC_C += ./beken378/func/wlan_ui/bk_peripheral_test.c
# utf8
SRC_FUNC_C += ./beken378/func/utf8/conv_utf8.c

# video / jpeg
SRC_FUNC_C += ./beken378/func/camera_intf/camera_intf.c
SRC_FUNC_C += ./beken378/func/camera_intf/camera_intf_gc2145.c
SRC_FUNC_C += ./beken378/func/video_transfer/video_transfer.c

ifeq ($(CFG_LOW_VOLTAGE_PS), 1)
SRC_FUNC_C += ./beken378/func/power_save/low_voltage_ps.c
SRC_FUNC_C += ./beken378/func/power_save/low_voltage_compensation.c
endif


ifeq ($(CFG_SUPPORT_MATTER), 1)
SRC_FUNC_C += ./beken378/func/key_value_flash/flash_namespace_value.c
endif


SRC_WOLFSSL_C =
# For WPA3: wolfssl
ifeq ($(CFG_WPA3),1)
ifneq ("${CFG_USE_MBEDTLS}", "1")
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/wolfmath.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/memory.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/tfm.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/ecc.c

# wpa_supplicant 2.9 needs random generator
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/random.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/sha256.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/sha512.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/md5.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/hmac.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/sha.c
endif
endif


ifeq ($(CFG_WPA2_ENTERPRISE),1)
ifeq ($(CFG_WPA_TLS_WOLFSSL),1)
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/wolfmath.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/memory.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/tfm.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/ecc.c

# WPA-Enterprise starts
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/asn.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/hash.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/md5.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/wc_port.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/coding.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/wc_encrypt.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/sha.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/aes.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/dh.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/hmac.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/rsa.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/logging.c
# WPA-Enterprise ends

# wpa_supplicant 2.9 needs random generator
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/random.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/wolfcrypt/src/sha256.c

# WPA-Enterprise starts
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/ssl.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/tls.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/internal.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/keys.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/wolfio.c
SRC_WOLFSSL_C += ./beken378/func/wolfssl/src/ocsp.c
# WPA-Enterprise ends
endif # CFG_WPA_TLS_WOLFSSL
endif # CFG_WPA_ENTERPRISE


# For BK7251
ifeq ($(CFG_SOC_NAME), 3)
SRC_DRV_C += ./beken378/driver/audio/audio.c
SRC_DRV_C += ./beken378/driver/audio/audio_adc.c
SRC_DRV_C += ./beken378/driver/audio/audio_dac.c
SRC_DRV_C += ./beken378/driver/audio/ring_buffer.c
SRC_DRV_C += ./beken378/driver/audio/ring_buffer_dma_read.c
SRC_DRV_C += ./beken378/driver/audio/ring_buffer_dma_write.c

SRC_FUNC_C += ./beken378/func/audio/audio_intf.c

SRC_DRV_C += ./beken378/driver/sdcard/sdcard.c
SRC_DRV_C += ./beken378/driver/sdcard/sdio_driver.c
SRC_DRV_C += ./beken378/driver/spi/spi.c
SRC_DRV_C += ./beken378/driver/spi/spi_master.c
SRC_DRV_C += ./beken378/driver/spi/spi_slave.c
SRC_FUNC_C += ./beken378/func/sd_music/sdcard_test.c
endif

# For BK7231U
ifeq ($(CFG_SOC_NAME), 2)
SRC_DRV_C += ./beken378/driver/spi/spi.c
SRC_DRV_C += ./beken378/driver/spi/spi_master.c
SRC_DRV_C += ./beken378/driver/spi/spi_slave.c
endif

# For BK7231N
ifeq ($(CFG_SOC_NAME), 5)
SRC_DRV_C += ./beken378/driver/spi/spi_bk7231n.c
SRC_DRV_C += ./beken378/driver/spi/spi_master_bk7231n.c
SRC_DRV_C += ./beken378/driver/spi/spi_slave_bk7231n.c
endif

# For BK7238
ifeq ($(CFG_SOC_NAME), 7)
SRC_DRV_C += ./beken378/driver/spi/spi_bk7231n.c
SRC_DRV_C += ./beken378/driver/spi/spi_master_bk7231n.c
SRC_DRV_C += ./beken378/driver/spi/spi_slave_bk7231n.c
endif

SRC_FUNC_C += ./beken378/func/wlan_ui/wlan_ui.c
SRC_FUNC_C += ./beken378/func/net_param_intf/net_param.c
ifneq ($(CFG_WPA2_ENTERPRISE),1)
SRC_FUNC_C += ./beken378/func/base64/base_64.c
endif
SRC_FUNC_C += ./beken378/func/airkiss/bk_airkiss.c
SRC_FUNC_C += ./beken378/func/airkiss/airkiss_main.c
SRC_FUNC_C += ./beken378/func/airkiss/airkiss_pingpong.c

ifeq ($(CFG_AP_MONITOR_COEXIST_DEMO), 1)
SRC_FUNC_C += ./beken378/func/monitor/monitor.c
endif

#easy flash
SRC_FUNC_C += ./beken378/func/easy_flash/bk_ef.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/easyflash.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/ef_env.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/ef_env_wl.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/ef_iap.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/ef_log.c
SRC_FUNC_C += ./beken378/func/easy_flash/src/ef_utils.c
SRC_FUNC_C += ./beken378/func/easy_flash/port/ef_port.c

#force sleep
SRC_FUNC_C += ./beken378/func/force_sleep/force_mac_ps.c
SRC_FUNC_C += ./beken378/func/force_sleep/force_mcu_ps.c

#paho-mqtt
ifeq ("${CFG_SUPPORT_RTOS}", "3")
SRC_FUNC_C += ./beken378/func/paho-mqtt/client/src/MQTTClient.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/client/src/MQTTFreeRTOS.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/client/paho_mqtt_udp.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTConnectClient.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTConnectServer.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTDeserializePublish.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTFormat.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTPacket.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTSerializePublish.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTSubscribeClient.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTSubscribeServer.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTUnsubscribeClient.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/packet/src/MQTTUnsubscribeServer.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/mqtt_ui/ssl_mqtt/ssl_mqtt_client_port.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/mqtt_ui/tcp_mqtt/tcp_mqtt_client_port.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/mqtt_ui/mqtt_client_core.c
SRC_FUNC_C += ./beken378/func/paho-mqtt/mqtt_ui/mqtt_client_com_port.c
endif

ifeq ("${CFG_MBEDTLS}", "1")
#MBEDTLS_SRC_DIRS += $(shell find beken378/func/mbedtls/mbedtls/library -type d)
#MBEDTLS_SRC_DIRS += $(shell find beken378/func/mbedtls/mbedtls-port/src -type d)
##MBEDTLS_SRC_DIRS += ./beken378/func/mbedtls/mbedtls/library/
##MBEDTLS_SRC_DIRS += ./beken378/func/mbedtls/mbedtls-port/src/
#SRC_C += $(foreach dir, $(MBEDTLS_SRC_DIRS), $(wildcard $(dir)/*.c))

SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/tls_hardware.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/tls_mem.c
ifeq ($(CFG_SUPPORT_MATTER), 1)
MBEDTLS_LIB_DIRS += ./beken378/func/mbedtls/mbedtls-2.27.0/library
SRC_MBEDTLS_C += $(foreach dir, $(MBEDTLS_LIB_DIRS), $(wildcard $(dir)/*.c))
else
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/ecp_curves_alt.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/ecp_alt.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/timing_alt.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/tls_certificate.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/tls_client.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls-port/src/tls_net.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/aes.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/aesni.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/arc4.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/asn1parse.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/asn1write.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/base64.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/bignum.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/blowfish.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/camellia.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ccm.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/certs.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/cipher.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/cipher_wrap.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/cmac.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ctr_drbg.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/debug.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/des.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/dhm.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ecdh.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ecdsa.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ecjpake.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ecp.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ecp_curves.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/entropy.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/entropy_poll.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/error.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/gcm.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/havege.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/hmac_drbg.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/md.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/md_wrap.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/md2.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/md4.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/md5.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/memory_buffer_alloc.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/net_sockets.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/oid.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/padlock.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pem.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pk.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pk_wrap.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pkcs5.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pkcs11.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pkcs12.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pkparse.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/pkwrite.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/platform.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ripemd160.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/rsa.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/sha1.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/sha256.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/sha512.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_cache.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_ciphersuites.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_cli.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_cookie.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_srv.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_ticket.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/ssl_tls.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/threading.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/timing.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/version.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/version_features.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509_create.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509_crl.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509_crt.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509_csr.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509write_crt.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/x509write_csr.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls/library/xtea.c
SRC_MBEDTLS_C += ./beken378/func/mbedtls/mbedtls_ui/sl_tls.c
endif
endif


SRC_BLE_PUB_C =
ifeq ($(CFG_SUPPORT_BLE),1)
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_4_2))
#ble pub
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_sdp.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_sec.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/driver/ble_icu/ble_icu.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/driver/uart/ble_uart.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/arch/ble_arch_main.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/common/RomCallFlash.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/dbg/dbg.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/dbg/dbg_mwsgen.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/dbg/dbg_swdiag.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/dbg/dbg_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/plactform/modules/rf/src/ble_rf_xvr.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/comm/src/comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/comm/src/comm_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/prf/src/prf.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/prf/src/prf_utils.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/sdp/src/sdp_service.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/profiles/sdp/src/sdp_service_task.c
#ble mesh pub
ifeq ($(CFG_SUPPORT_BLE_MESH),1)
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/mesh_api.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/mesh_api_msg.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/mesh_param_int.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/mm_api.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/mm_api_msg.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/m_api.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/mesh_api/m_api_msg.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/common/mm_route.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/common/mm_tb.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/common/mm_tb_bind.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/common/mm_tb_replay.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/common/mm_tb_state.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_bat.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_dtt.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_loc.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_lvl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_oo.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_plvl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_poo.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/gens/mm_gens_prop.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lightc/mm_lightc.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lightc/mm_lightc_ctl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lightc/mm_lightc_hsl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lightc/mm_lightc_ln.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lightc/mm_lightc_xyl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lights/mm_lights.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lights/mm_lights_ctl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lights/mm_lights_hsl.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/lights/mm_lights_ln.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/Scenes/m_fnd_Scenes.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/transition_time/m_fnd_generic_transition_time.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/vendor/mm_vendors.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/beken_ble_sdk/mesh/src/models/vendor/mm_vendor_midea.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_mesh.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/app/src/app_mm_msg.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/gernel_api/mesh_general_api.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_4_2/modules/mesh_model/ali/app_light_ali_server.c
endif
endif #BLE_VERSION_4_2
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_1))
# ble pub
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/prf/prf.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/prf/prf_utils.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/profiles/comm/src/comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/profiles/comm/src/comm_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/ui/ble_ui.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/rwip/src/rwip.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/rwip/src/rwble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/entry/ble_main.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/driver/rf/rf_xvr.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/driver/rf/ble_rf_port.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/platform/7231n/driver/uart/uart_ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_ble_init.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_sdp.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/app/src/app_sec.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/profiles/sdp/src/sdp_common.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_1/ble_pub/profiles/sdp/src/sdp_comm_task.c
endif #BLE_VERSION_5_1
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_2))
# ble pub
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/bk_comm/src/comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/bk_comm/src/comm_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/bas/bass/src/bass.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/dis/diss/src/diss.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_diss.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_comm.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_bass.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/hogp/hogpd/src/hogpd.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_hogpd.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/find/findt/src/findt.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_findt.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_task.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/ui/ble_ui.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/platform/7238/entry/ble_main.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/platform/7238/driver/rf/rf_xvr.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/platform/7238/driver/rf/ble_rf_port.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/platform/7238/driver/uart/uart_ble.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_ble_init.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_sdp.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/app/src/app_sec.c
SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/bk_sdp/src/sdp_common.c
# SRC_BLE_PUB_C += ./beken378/driver/ble/ble_5_2/ble_pub/profiles/sdp/src/sdp_comm_task.c
endif #BLE_VERSION_5_2
endif #CFG_SUPPORT_BLE

#operation system module
ifeq ("${CFG_SUPPORT_RTOS}", "3")
SRC_OS += ./beken378/os/FreeRTOSv9.0.0/rtos_pub.c
SRC_OS  += ./beken378/os/mem_arch.c
SRC_OS  += ./beken378/os/platform_stub.c
SRC_OS  += ./beken378/os/str_arch.c
endif

ifeq ($(ATSVR_CFG),1)
SRC_FUNC_C += ./beken378/func/at_server/_at_server_port/atsvr_core.c
SRC_FUNC_C += ./beken378/func/at_server/_at_server_port/atsvr_port.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_cmd/atsvr_cmd.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_cmd/atsvr_wlan.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_cmd/atsvr_ble.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_cmd/atsvr_misc.c

SRC_FUNC_C += ./beken378/func/at_server/atsvr_http_cmd/atsvr_http_cmd.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_airkiss_cmd/atsvr_airkiss_cmd.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_airkiss_cmd/ble_config.c


SRC_FUNC_C += ./beken378/func/at_server/_at_server/_at_server.c
SRC_FUNC_C += ./beken378/func/at_server/at_server_func/_atsvr_func.c
SRC_FUNC_C += ./beken378/func/at_server/at_server.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_comm.c
SRC_FUNC_C += ./beken378/func/at_server/network/network_interface.c
SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_TCP_lwip.c
SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_UDP_lwip.c
SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_TLS_mbedtls.c
SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_freertos.c
SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_DTLS_mbedtls.c
#SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_Device_freertos.c
#SRC_FUNC_C += ./beken378/func/at_server/network/net_hal/HAL_Device_bk_flash.c
SRC_FUNC_C += ./beken378/func/at_server/atsvr_net_cmd/atsvr_net_cmd.c
SRC_FUNC_C += ./beken378/func/at_server/network/network_app.c
SRC_FUNC_C += ./beken378/func/at_server/network/network_tls.c
SRC_FUNC_C += ./beken378/func/at_server/network/network_socket.c
SRC_FUNC_C += ./beken378/func/at_server/utils/utils_base64.c

SRC_FUNC_C += ./beken378/func/at_server/utils/utils_sha1.c
SRC_FUNC_C += ./beken378/func/at_server/utils/utils_ringbuff.c
SRC_FUNC_C += ./beken378/func/at_server/utils/utils_md5.c
SRC_FUNC_C += ./beken378/func/at_server/utils/utils_list.c

SRC_FUNC_C += ./beken378/func/at_server/mqtt/qcloud_at_mqtt.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_common.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_connect.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_net.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_publish.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_subscribe.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_unsubscribe.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/mqtt_client_yield.c
SRC_FUNC_C += ./beken378/func/at_server/mqtt/atsvr_mqtt_cmd.c

endif

ifeq ($(CFG_WRAP_LIBC),1)
SRC_FUNC_C += ./beken378/func/libc/errno/lib_errno.c
SRC_FUNC_C += ./beken378/func/libc/math/lib_exp.c
SRC_FUNC_C += ./beken378/func/libc/math/lib_libexpi.c
SRC_FUNC_C += ./beken378/func/libc/math/lib_log.c
SRC_FUNC_C += ./beken378/func/libc/math/lib_pow.c
SRC_FUNC_C += ./beken378/func/libc/stdio/lib_libvscanf.c
SRC_FUNC_C += ./beken378/func/libc/stdio/lib_memsistream.c
SRC_FUNC_C += ./beken378/func/libc/stdio/lib_meminstream.c
SRC_FUNC_C += ./beken378/func/libc/stdio/lib_sscanf.c
SRC_FUNC_C += ./beken378/func/libc/stdio/lib_vsscanf.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_checkbase.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_strtod.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_qsort.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_srand.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_strtol.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_strtoll.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_strtoul.c
SRC_FUNC_C += ./beken378/func/libc/stdlib/lib_strtoull.c
SRC_FUNC_C += ./beken378/func/libc/string/lib_isbasedigit.c
endif

ifeq ($(CFG_USB),1)
SRC_DRV_C += ./beken378/driver/usb/usb.c
SRC_FUNC_C += ./beken378/func/usb/fusb.c
endif

ifeq ($(CFG_QUICK_TRACK),1)
SRC_FUNC_C += ./beken378/func/controlappc/controlappc_main.c
SRC_FUNC_C += ./beken378/func/controlappc/controlapp_eloop.c
SRC_FUNC_C += ./beken378/func/controlappc/indigo_api.c
SRC_FUNC_C += ./beken378/func/controlappc/indigo_packet.c
SRC_FUNC_C += ./beken378/func/controlappc/utils.c
SRC_FUNC_C += ./beken378/func/controlappc/indigo_api_callback_dut.c
SRC_FUNC_C += ./beken378/func/controlappc/vendor_specific_dut.c
endif

SRC_FUNC_C += ./beken378/func/ble_wifi_exchange/ble_wifi_port.c

#rf calibration public
SRC_FUNC_C += ./beken378/func/bk7011_cal/bk_cal_pub.c

#assembling files
ifeq ("${CFG_SUPPORT_RTOS}", "3")
SRC_S +=  ./beken378/driver/entry/boot_handlers.S
endif

SRC_S +=  ./beken378/driver/entry/boot_vectors.S

ifeq ("${CFG_SUPPORT_RTOS}", "4")
SRC_S +=  ./beken378/driver/entry/boot_handlers_liteos.S
endif


# -------------------------------------------------------------------
# Lib source file list
# -------------------------------------------------------------------

# Lib files will be deleted when making SDK.
SRC_IP_C =
-include ./beken378/ip/ip_lib_src.mk

SRC_BLE_C =
ifeq ($(CFG_SUPPORT_BLE),1)
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_4_2))
-include ./beken378/driver/ble/ble_4_2/ble_lib_src.mk
endif
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_1))
-include ./beken378/driver/ble/ble_5_1/ble_lib_src.mk
endif
ifeq ($(CFG_BLE_VERSION),$(BLE_VERSION_5_2))
-include ./beken378/driver/ble/ble_5_2/ble_lib_src.mk
endif
endif

SRC_USB_C =
ifeq ($(CFG_USB),1)
-include ./beken378/driver/usb/usb_lib_src.mk
endif

SRC_SENSOR_C =
ifeq ($(CFG_WIFI_SENSOR),1)
-include ./beken378/func/sensor/sensor_lib_src.mk
endif

SRC_BK_AWARE_C =
ifeq ($(CFG_BK_AWARE),1)
-include ./beken378/func/bk_aware/bk_aware_lib_src.mk
endif

SRC_CAL_C =
-include ./beken378/func/bk7011_cal/cal_lib_src.mk

SRC_SUPPLICANT_C =
-include ./beken378/func/wpa_supplicant_2_9/supplicant_lib_src.mk

SRC_UART_DEBUG_C =
-include ./beken378/func/uart_debug/uart_debug_lib_src.mk

SRC_RF_TEST_C =
-include ./beken378/func/rf_test/rf_test_lib_src.mk

SRC_RF_USE_C =
-include ./beken378/func/rf_use/rf_use_lib_src.mk


