REPO_DIR       = $(HOME)/catkin_ws/src/Gunnar
PROJECT_DIR = $(REPO_DIR)/src/arduino/libraries/9dofTest
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
#AVRDUDE = /usr/share/arduino/hardware/tools/avrdude
#CFLAGS_STD        = -std=gnu11
CXXFLAGS_STD      = -std=c++0x -w
#CXXFLAGS         += -pedantic -Wall -Wextra
#MONITOR_PORT      = /dev/tty.usbmodem*
MONITOR_PORT      = /dev/ttyACM*
CURRENT_DIR       = $(shell basename $(CURDIR))
OBJDIR            = $(PROJECT_DIR)/bin/$(BOARD_TAG)/$(CURRENT_DIR)

ARDUINO_LIBS = Adafruit_9DOF Adafruit_Sensor Wire Adafruit_LSM303_U Adafruit_L3GD20_U

include $(ARDMK_DIR)/Arduino.mk

