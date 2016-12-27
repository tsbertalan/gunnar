# Original project where this Makefile comes from: https://github.com/WeAreLeka/Bare-Arduino-Project
REPO_DIR       = $(HOME)/catkin_ws/src/Gunnar
PROJECT_DIR = $(REPO_DIR)/src/arduino/libraries/ROSArduinoBridge


AVRDUDE_CONF = $(REPO_DIR)/src/arduino/avr/avrdude.conf
AVRDUDE  = /opt/arduino-1.6.3/hardware/tools/avr/bin/avrdude
ARDMK_DIR         = $(REPO_DIR)/src/arduino/avr/Arduino-Makefile
ARDUINO_DIR       = /opt/arduino-1.6.3/
USER_LIB_PATH = $(REPO_DIR)/src/arduino/libraries
AVR_TOOLS_DIR     = /opt/arduino-1.6.3/hardware/tools/avr/
#AVR_TOOLS_DIR     = /usr/bin
AVRDUDE          = /opt/arduino-1.6.3/hardware/tools/avr/bin/avrdude
#AVRDUDE = /usr/share/arduino/hardware/tools/avrdude

BOARD_TAG = uno
#BOARD_SUB = atmega2560

MONITOR_BAUDRATE = 38400
MONITOR_PORT = /dev/ttyUSB*

# Set the C standard to be used during compilation. Documentation (https://github.com/WeAreLeka/Arduino-Makefile/blob/std-flags/arduino-mk-vars.md#cflags_std)
#CFLAGS_STD        = -std=gnu11
# Set the C++ standard to be used during compilation. Documentation (https://github.com/WeAreLeka/Arduino-Makefile/blob/std-flags/arduino-mk-vars.md#cxxflags_std)
#CXXFLAGS_STD      = -std=gnu++11
# Ignore warnings, since the Arduino libraries are filled with them, and -isystem doesn't seem to work.
CXXFLAGS_STD      = -std=c++0x -w
# Flags you might want to set for debugging purpose. Comment to stop.
#CXXFLAGS         += -pedantic -Wall -Wextra

# Do not touch
CURRENT_DIR       = $(shell basename $(CURDIR))
OBJDIR            = $(PROJECT_DIR)/bin/$(BOARD_TAG)/$(CURRENT_DIR)
include $(ARDMK_DIR)/Arduino.mk

