#############################################################
#
# Created by Espressif
# UDK modifications by CHERTS <sleuthhound@gmail.com>
# Cross platform compatability by kireevco <dmitry@kireev.co>
#
#############################################################

## Defaults for Sming Core

## ESP_HOME sets the path where ESP tools and SDK are located.
## Windows:
# ESP_HOME = c:/Espressif

## MacOS / Linux:
# ESP_HOME = /opt/esp-open-sdk

## SMING_HOME sets the path where Sming framework is located.
## Windows:
# SMING_HOME = c:/tools/sming/Sming 

# MacOS / Linux
# SMING_HOME = /opt/esp-open-sdk

## COM port parameter is reqruied to flash firmware correctly.
## Windows: 
# COM_PORT = COM3

# MacOS / Linux:
# COM_PORT = /dev/tty.usbserial

# Com port speed
COM_SPEED = 115200

XTENSA_TOOLS_ROOT := /Users/pit/Library/Arduino15/packages/esp8266/tools/xtensa-lx106-elf-gcc/1.20.0-26-gb404fb9/bin


ifeq ($(OS),Windows_NT)
  # Windows detected
  UNAME := Windows
  
  # Default SMING_HOME. Can be overriden.
  SMING_HOME ?= c:\tools\Sming\Sming

  # Default ESP_HOME. Can be overriden.
  ESP_HOME ?= c:\Espressif

  # Making proper path adjustments - replace back slashes, remove colon and add forward slash.
  SMING_HOME := $(subst \,/,$(addprefix /,$(subst :,,$(SMING_HOME))))
  ESP_HOME := $(subst \,/,$(addprefix /,$(subst :,,$(ESP_HOME))))
  include Makefile-windows.mk  
else
  UNAME := $(shell uname -s)
  ifeq ($(UNAME),Darwin)
      # MacOS Detected
      UNAME := MacOS

      # Default SMING_HOME. Can be overriden.
      SMING_HOME ?= /opt/sming/Sming

      # Default ESP_HOME. Can be overriden.
      ESP_HOME ?= /opt/esp-open-sdk

      include Makefile-macos.mk      
  endif
  ifeq ($(UNAME),Linux)
      # Linux Detected
      UNAME := Linux

      # Default SMING_HOME. Can be overriden.
      SMING_HOME ?= /opt/sming/Sming

      # Default ESP_HOME. Can be overriden.
      ESP_HOME ?= /opt/esp-open-sdk
      include Makefile-linux.mk     
  endif
  ifeq ($(UNAME),FreeBSD)
      # Freebsd Detected
      UNAME := FreeBSD

      # Default SMING_HOME. Can be overriden.
      SMING_HOME ?= /usr/local/esp8266/Sming/Sming

      # Default ESP_HOME. Can be overriden.
      ESP_HOME ?= /usr/local/esp8266/esp-open-sdk
      include $(SMING_HOME)/Makefile-bsd.mk     
  endif
endif

export COMPILE := gcc
export PATH := $(ESP_HOME)/xtensa-lx106-elf/bin:$(PATH)

# Sming Framework Path
SMF = $(SMING_HOME)

SPIFF_FILES = files

BUILD_BASE	= out/build
FW_BASE		= out/firmware

# name for the target project
TARGET		= app

# which modules (subdirectories) of the project to include in compiling
#MODULES		= system system/helpers Wiring SmingCore appinit $(filter %/, $(wildcard SmingCore/*/)) $(filter %/, $(wildcard Services/*/)) $(filter %/, $(wildcard Libraries/*/))
MODULES		= \
	system \
	system/helpers \
	Wiring \
	SmingCore \
	appinit \
	$(filter %/, $(wildcard SmingCore/*/)) \
	$(filter %/, $(wildcard Services/*/))
#	$(filter %/, $(wildcard Libraries/*/))

#EXTRA_INCDIR    = include include system/include Wiring Libraries SmingCore $(SDK_BASE)/include
EXTRA_INCDIR    ?= include \
  $(SMING_HOME)/include \
  $(SMING_HOME)/ \
  $(SMING_HOME)/system/include \
  $(SMING_HOME)/Wiring \
  $(SMING_HOME)/Libraries \
  $(SMING_HOME)/SmingCore \
  $(SDK_BASE)/include \
  $(SDK_BASE)/include/espressif \
  $(SDK_BASE)/include/freertos \
  $(SDK_BASE)/include/json \
  $(SDK_BASE)/include/lwip \
  $(SDK_BASE)/include/lwip/ipv4 \
  $(SDK_BASE)/include/lwip/ipv6 \
  $(SDK_BASE)/include/ssl \
  $(SDK_BASE)/include/udhcp \
  $(SDK_BASE)/extra_include

# libraries used in this project, mainly provided by the SDK
USER_LIBDIR = compiler/lib
#LIBS		= microc microgcc hal phy pp net80211 wpa main
LIBS		= gcc hal phy pp net80211 wpa main freertos lwip udhcp sming $(EXTRA_LIBS)

# compiler flags using during compilation of source files. Add '-pg' for debugging
#-Wundef
CFLAGS		= -Os -g -Wpointer-arith -Werror -Wundef -Wl,-EL -nostdlib -mlongcalls -mtext-section-literals -finline-functions -fdata-sections -ffunction-sections -D__ets__ -DICACHE_FLASH -DARDUINO=106
CXXFLAGS	= $(CFLAGS) -fno-rtti -fno-exceptions -std=c++11 -felide-constructors

# linker flags used to generate the main object file
LDFLAGS		= -nostdlib -u call_user_start -Wl,-static -Wl,--gc-sections

# linker script used for the above linkier step
LD_PATH     = compiler/ld/
LD_SCRIPT	= $(LD_PATH)eagle.app.v6.cpp.ld

# BOOT = none
# BOOT = old - boot_v1.1
# BOOT = new - boot_v1.2+
BOOT?=none
# APP = 0 - eagle.flash.bin + eagle.irom0text.bin
# APP = 1 - user1.bin
# APP = 2 - user2.bin
APP?=0
# SPI_SPEED = 20MHz, 26.7MHz, 40MHz, 80MHz
SPI_SPEED?=40
# SPI_MODE: QIO, QOUT, DIO, DOUT
SPI_MODE?=QIO
# SPI_SIZE: 256KB, 512KB, 1024KB, 2048KB, 4096KB
SPI_SIZE?=512

ifeq ($(BOOT), new)
    boot = new
else
    ifeq ($(BOOT), old)
        boot = old
    else
        boot = none
    endif
endif

ifeq ($(APP), 1)
    app = 1
else
    ifeq ($(APP), 2)
        app = 2
    else
        app = 0
    endif
endif

ifeq ($(SPI_SPEED), 26.7)
    freqdiv = 1
else
    ifeq ($(SPI_SPEED), 20)
        freqdiv = 2
    else
        ifeq ($(SPI_SPEED), 80)
            freqdiv = 15
        else
            freqdiv = 0
        endif
    endif
endif


ifeq ($(SPI_MODE), QOUT)
    mode = 1
else
    ifeq ($(SPI_MODE), DIO)
        mode = 2
    else
        ifeq ($(SPI_MODE), DOUT)
            mode = 3
        else
            mode = 0
        endif
    endif
endif

# flash larger than 1024KB only use 1024KB to storage user1.bin and user2.bin
ifeq ($(SPI_SIZE), 256)
    size = 1
    flash = 256
else
    ifeq ($(SPI_SIZE), 1024)
        size = 2
        flash = 1024
    else
        ifeq ($(SPI_SIZE), 2048)
            size = 3
            flash = 1024
        else
            ifeq ($(SPI_SIZE), 4096)
                size = 4
                flash = 1024
            else
                size = 0
                flash = 512
            endif
        endif
    endif
endif

ifeq ($(flash), 512)
  ifeq ($(app), 1)
    addr = 0x01000
  else
    ifeq ($(app), 2)
      addr = 0x41000
    endif
  endif
else
  ifeq ($(flash), 1024)
    ifeq ($(app), 1)
      addr = 0x01000
    else
      ifeq ($(app), 2)
        addr = 0x81000
      endif
    endif
  endif
endif

ifneq ($(boot), none)
ifneq ($(app),0)
	LD_SCRIPT = $(LD_PATH)eagle.app.v6.$(boot).$(flash).app$(app).cpp.ld
	BIN_NAME = user$(app).$(flash).$(boot)
endif
else
    app = 0
endif

# various paths from the SDK used in this project
SDK_LIBDIR	= lib
SDK_LDDIR	= ld
SDK_INCDIR	= include include/json

# select which tools to use as compiler, librarian and linker
AS		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-gcc
CC		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-gcc
CXX		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-g++
AR		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-ar
LD		:= $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-gcc
OBJCOPY := $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-objcopy
OBJDUMP := $(XTENSA_TOOLS_ROOT)/xtensa-lx106-elf-objdump

SRC_DIR		:= $(MODULES)
BUILD_DIR	:= $(addprefix $(BUILD_BASE)/,$(MODULES))

SDK_LIBDIR	:= $(addprefix $(SDK_BASE)/,$(SDK_LIBDIR))
SDK_INCDIR	:= $(addprefix -I$(SDK_BASE)/,$(SDK_INCDIR))

AS_SRC		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.s))
C_SRC		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.c))
CXX_SRC		:= $(foreach sdir,$(SRC_DIR),$(wildcard $(sdir)/*.cpp))
AS_OBJ		:= $(patsubst %.s,$(BUILD_BASE)/%.o,$(AS_SRC))
C_OBJ		:= $(patsubst %.c,$(BUILD_BASE)/%.o,$(C_SRC))
CXX_OBJ		:= $(patsubst %.cpp,$(BUILD_BASE)/%.o,$(CXX_SRC))
OBJ		:= $(AS_OBJ) $(C_OBJ) $(CXX_OBJ)
LIBS		:= $(addprefix -l,$(LIBS))
APP_AR		:= $(addprefix $(BUILD_BASE)/,$(TARGET)_app.a)
TARGET_OUT	:= $(addprefix $(BUILD_BASE)/,$(TARGET).out)

#LD_SCRIPT	:= $(addprefix -T$(SDK_BASE)/$(SDK_LDDIR)/,$(LD_SCRIPT))
LD_SCRIPT	:= $(addprefix -T,$(LD_SCRIPT))

INCDIR	:= $(addprefix -I,$(SRC_DIR))
EXTRA_INCDIR	:= $(addprefix -I,$(EXTRA_INCDIR))
MODULE_INCDIR	:= $(addsuffix /include,$(INCDIR))

V ?= $(VERBOSE)
ifeq ("$(V)","1")
Q :=
vecho := @true
else
Q := @
vecho := @echo
endif

# test:
# 	#$(vecho) $(OBJ) $(APP_AR)
# 	$(vecho) $(BUILD_DIR)

vpath %.c $(SRC_DIR)
vpath %.cpp $(SRC_DIR)
vpath %.s $(SRC_DIR)

define compile-objects
$1/%.o: %.s
	$(vecho) "AS $$<"
	$(Q) $(AS) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@	
$1/%.o: %.c
	$(vecho) "CC $$<"
	$(Q) $(CC) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CFLAGS)  -c $$< -o $$@
$1/%.o: %.cpp
	$(vecho) "C+ $$<"
	$(Q) $(CXX) $(INCDIR) $(MODULE_INCDIR) $(EXTRA_INCDIR) $(SDK_INCDIR) $(CXXFLAGS)  -c $$< -o $$@
endef

.PHONY: all checkdirs clean

all: checkdirs $(APP_AR) $(FW_FILE_1) $(FW_FILE_2)

$(APP_AR): $(OBJ)
	$(vecho) "AR $@"
	$(Q) $(AR) cru $@ $^
	$(vecho) "Installing libsming"
	$(Q) cp -r $(APP_AR) $(USER_LIBDIR)/libsming.a  
	$(vecho) "Done"

checkdirs: $(BUILD_DIR) $(FW_BASE)

$(BUILD_DIR):
	$(Q) mkdir -p $@

$(FW_BASE):
	$(Q) mkdir -p $@
	$(Q) mkdir -p $@/upgrade

rebuild: clean all

clean:
	$(Q) rm -f $(APP_AR)
	$(Q) rm -f $(TARGET_OUT)
	$(Q) rm -rf $(BUILD_DIR)
	$(Q) rm -rf $(BUILD_BASE)
	$(Q) rm -rf $(FW_BASE)

$(foreach bdir,$(BUILD_DIR),$(eval $(call compile-objects,$(bdir))))
