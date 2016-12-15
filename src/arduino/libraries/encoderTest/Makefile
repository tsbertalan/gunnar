REPO_DIR       = $(HOME)/catkin_ws/src/Gunnar
PROJECT_DIR = $(REPO_DIR)/src/arduino/libraries/encoderTest
AVRDUDE_CONF = $(REPO_DIR)/src/arduino/avr/avrdude.conf
ARDMK_DIR         = $(REPO_DIR)/src/arduino/avr/Arduino-Makefile
ARDUINO_DIR       = /opt/arduino-1.6.3/
USER_LIB_PATH = $(REPO_DIR)/src/arduino/libraries
BOARD_TAG = mega
BOARD_SUB = atmega2560
MONITOR_BAUDRATE  = 57600
# Path to the AVR tools directory such as avr-gcc, avr-g++, etc.
AVR_TOOLS_DIR     = /opt/arduino-1.6.3/hardware/tools/avr/
AVRDUDE          = /opt/arduino-1.6.3/hardware/tools/avr/bin/avrdude
# posisbly also
#AVRDUDE = /usr/share/arduino/hardware/tools/avrdude
# Set the C standard to be used during compilation. Documentation (https://github.com/WeAreLeka/Arduino-Makefile/blob/std-flags/arduino-mk-vars.md#cflags_std)
#CFLAGS_STD        = -std=gnu11
# Set the C++ standard to be used during compilation. Documentation (https://github.com/WeAreLeka/Arduino-Makefile/blob/std-flags/arduino-mk-vars.md#cxxflags_std)
#CXXFLAGS_STD      = -std=gnu++11
# Ignore warnings, since the Arduino libraries are filled with them, and -isystem doesn't seem to work.
CXXFLAGS_STD      = -std=c++0x -w
# Flags you might want to set for debugging purpose. Comment to stop.
#CXXFLAGS         += -pedantic -Wall -Wextra
# The port your board is connected to. Using an '*' tries all the ports and finds the right one.
#MONITOR_PORT      = /dev/tty.usbmodem*
MONITOR_PORT      = /dev/ttyACM*
# Do not touch - used for binaries path
CURRENT_DIR       = $(shell basename $(CURDIR))
# This is where you put the binaries you just compile using 'make'
OBJDIR            = $(PROJECT_DIR)/bin/$(BOARD_TAG)/$(CURRENT_DIR)
### Do not touch - the path to Arduino.mk, inside the ARDMK_DIR
include $(ARDMK_DIR)/Arduino.mk

