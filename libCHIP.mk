#!/bin/bash
# This file is used to generate CHIP core library
#

ARM_GCC_TOOLCHAIN = ${FREERTOS_EXEC_PATH}
CROSS_COMPILE = $(ARM_GCC_TOOLCHAIN)arm-none-eabi-
CHIP_DIR = $(ROOT_DIR)/connectedhomeip
OUTPUT_DIR = $(ROOT_DIR)/out/matter
MATTER_EXAMPLE = lighting-app
# Compilation tools
AR = $(CROSS_COMPILE)ar
CXX = $(CROSS_COMPILE)g++
CC = $(CROSS_COMPILE)gcc
AS = $(CROSS_COMPILE)as
NM = $(CROSS_COMPILE)nm
LD = $(CROSS_COMPILE)gcc
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump

Q := @
ifeq ($(V),1)
Q := 
endif

ifeq ($(ECHO),)
ECHO=echo
endif

-include .config

# -------------------------------------------------------------------
# Include folder list
# -------------------------------------------------------------------
INCLUDES =

INCLUDES += -I$(ROOT_DIR)/config
INCLUDES += -I$(ROOT_DIR)/release

ifeq ("${CFG_SUPPORT_RTOS}", "4")
INCLUDES += -I./os/liteos_m/config/
INCLUDES += -I./os/liteos_m/components/exchook
INCLUDES += -I./os/liteos_m/components/cpup
INCLUDES += -I./os/liteos_m/components/power
INCLUDES += -I./os/liteos_m/components/port
INCLUDES += -I./os/liteos_m/components/bounds_checking_function/include/
INCLUDES += -I./os/liteos_m/kernel/include/
INCLUDES += -I./os/liteos_m/kernel/arch/include/
INCLUDES +=  -I./os/liteos_m/targets/bk72xx/
INCLUDES += -I./os/liteos_m/port/
INCLUDES += -I./os/liteos_m/utils/
INCLUDES += -I./os/liteos_m/kernel/arch/arm/arm9/gcc/
endif

ifeq ("${CFG_SUPPORT_RTOS}", "3")
INCLUDES += -I$(ROOT_DIR)/os/FreeRTOSv9.0.0/FreeRTOS/Source/portable/Keil/ARM968es
INCLUDES += -I$(ROOT_DIR)/os/FreeRTOSv9.0.0/FreeRTOS/Source/include
endif

INCLUDES += -I$(CHIP_DIR)/src/platform/Beken/bk7231

# -------------------------------------------------------------------
# CHIP compile options
# -------------------------------------------------------------------
CFLAGS =
CXXFLAGS =

CXXFLAGS = -mthumb -mcpu=arm968e-s -mthumb-interwork -mlittle-endian -march=armv5te
CXXFLAGS += -DCHIP_ADDRESS_RESOLVE_IMPL_INCLUDE_HEADER="<lib/address_resolve/AddressResolve_DefaultImpl.h>"
CFLAGS += -DCHIP_PROJECT=1
CFLAGS += -DCHIP_HAVE_CONFIG_H=1
CXXFLAGS += -Wno-conversion
CXXFLAGS += -Os
CFLAGS += -DCFG_MBEDTLS=1
CFLAGS += -DLWIP_IPV6=1

CFLAGS += -DLWIP_IPV6_ND=1
CFLAGS += -DLWIP_IPV6_SCOPES=1
CFLAGS += -DLWIP_PBUF_FROM_CUSTOM_POOLS=0
CFLAGS += -DLWIP_IPV6_ROUTE_TABLE_SUPPORT=1

CFLAGS += -DCHIP_DEVICE_LAYER_NONE=0
CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_ZEPHYR_NET_IF=0
CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_BSD_IFADDRS=0
CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_ZEPHYR_SOCKET_EXTENSIONS=0

CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_LWIP=1
CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_SOCKETS=0
CFLAGS += -DCHIP_SYSTEM_CONFIG_USE_NETWORK_FRAMEWORK=0
#CFLAGS += -DDOXYGEN_SHOULD_SKIP_THIS
#CXXFLAGS += -DFD_SETSIZE=10

CXXFLAGS += -Wno-sign-compare
CXXFLAGS += -Wno-unused-function
CXXFLAGS += -Wno-unused-but-set-variable
CXXFLAGS += -Wno-unused-variable
#CXXFLAGS += -Wno-deprecated-declarations
CXXFLAGS += -Wno-unused-parameter
#CXXFLAGS += -Wno-format
CXXFLAGS += -Wno-literal-suffix
CXXFLAGS += -std=gnu++14
CXXFLAGS += -fno-rtti
CXXFLAGS += -Wno-write-strings


CHIP_CFLAGS = $(CFLAGS)
CHIP_CFLAGS += $(INCLUDES)

CHIP_CXXFLAGS += $(CFLAGS)
CHIP_CXXFLAGS += $(CXXFLAGS)
CHIP_CXXFLAGS += $(INCLUDES)

#Beken SDK include folder and source file list
-include ./beken378/beken_src.mk

#*****************************************************************************#
#                        RULES TO GENERATE libCHIP.a and libAPP.a             #
#*****************************************************************************#

# Define the Rules to build the core targets
all: CHIP_CORE
CHIP_CORE:
	@echo "Compiling CHIP SDK static library"
	@if [ ! -d $(OUTPUT_DIR) ]; then \
		mkdir -p $(OUTPUT_DIR); \
	fi
	@echo $(ROOT_DIR)
	@echo                                   > $(OUTPUT_DIR)/args.gn
	@echo "import(\"//args.gni\")"          >> $(OUTPUT_DIR)/args.gn
	@echo target_cflags_c  = [$(foreach word,$(CHIP_CFLAGS),\"$(word)\",)] | sed -e 's/=\"/=\\"/g;s/\"\"/\\"\"/g;'  >> $(OUTPUT_DIR)/args.gn
	@echo target_cflags_cc = [$(foreach word,$(CHIP_CXXFLAGS),\"$(word)\",)] | sed -e 's/=\"/=\\"/g;s/\"\"/\\"\"/g;'   >> $(OUTPUT_DIR)/args.gn
	@echo chip_progress_logging = true 	>> $(OUTPUT_DIR)/args.gn
	@echo chip_detail_logging = false	>> $(OUTPUT_DIR)/args.gn
	@echo chip_automation_logging = false	>> $(OUTPUT_DIR)/args.gn
	@echo beken_soc = \"bk7231\" 		>> $(OUTPUT_DIR)/args.gn
	@echo target_cpu = \"arm\" 		>> $(OUTPUT_DIR)/args.gn
	@echo target_os = \"freertos\" 		>> $(OUTPUT_DIR)/args.gn
	@echo beken_ar = \"${AR}\"    >> $(OUTPUT_DIR)/args.gn
	@echo beken_cc = \"${CC}\"   >> $(OUTPUT_DIR)/args.gn
	@echo beken_cxx = \"${CXX}\"  >> $(OUTPUT_DIR)/args.gn
	@cd $(CHIP_DIR)/examples/$(MATTER_EXAMPLE)/beken && gn gen --check --fail-on-unused-args $(OUTPUT_DIR)
	@cd $(CHIP_DIR)/examples/$(MATTER_EXAMPLE)/beken ; ninja -C $(OUTPUT_DIR)

.PHONY: clean
clean:
	rm -rf $(OUTPUT_DIR)/

