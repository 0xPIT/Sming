# ESP8266 sdk package home directory
ESP_HOME ?= /Users/pit/Desktop/ESP/

# base directory of the ESP8266 SDK package, absolute
SDK_BASE	?= /Users/pit/Desktop/ESP/sdk/esp_iot_rtos_sdk/
SDK_TOOLS	 ?= $(SDK_BASE)/tools

# Other tools mappings
ESPTOOL		 ?= /Users/pit/Development/git/ESP8266/esptool/esptool.py
GEN_APPBIN   := PATH="$(ESP_HOME)/xtensa-lx106-elf/bin:$(PATH)" && $(SDK_TOOLS)/gen_appbin.py
GEN_FLASHBIN := PATH="$(ESP_HOME)/xtensa-lx106-elf/bin:$(PATH)" && $(SDK_TOOLS)/gen_flashbin.py
KILL_TERM    ?= pkill screen
GET_FILESIZE ?= stat -L -f%z

COM_PORT     ?= /dev/tty.usbserial