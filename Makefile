ARDUINO_DIR = /usr/share/arduino
#ARDMK_DIR = /usr/share/arduino
AVR_TOOLS_DIR = /usr

BOARD_TAG = mega2560
IDE_BOARD_NAME = arduino:avr:mega
# The folder gunnar must be copied or, better, symlinked into your
# user libraries folder:
ARDUINO_LIBS = Servo Wire gunnar Adafruit_Sensor Adafruit_LSM303_U Adafruit_9DOF Adafruit_L3GD20_U
IDE_MAKE = arduino --port ${ARDUINO_PORT} --board ${IDE_BOARD_NAME}
ARDUINO_PORT = $(shell ls /dev/ttyACM* 2> /dev/null)

include /usr/share/arduino/Arduino.mk

ide:
	arduino `pwd`/gunnar.ino &

# The targets "go" and "stop" require a newer version of the Arduino IDE than is
# available in the Ubuntu 14.04 repositories. Download 1.5.2 or later from
# arduino.cc/download , put the arduino-1.*.* folder in /opt/ , and symlink
# /opt/arduino-1.*.*/arduino to /usr/local/bin .
#
# After this, you may have to run `arduino` once, and, in the preferences,
# change the "Sketchbook location" (presumably you can also do this in
# ~/.arduino/preferences.txt).

# .PHONY directives specify that this is an action to take,
# not a real target to build (so make won't just claim the target
# is up-to-date and then stop).
.PHONY: go
go:
	${IDE_MAKE} --upload gunnar.ino
	
upload: go

.PHONY: stop
stop:
	${IDE_MAKE} --upload stop/stop.ino

test:
	python testing.py