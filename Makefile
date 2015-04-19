ARDUINO_DIR = /usr/share/arduino
#ARDMK_DIR = /usr/share/arduino
AVR_TOOLS_DIR = /usr


BOARD_TAG = mega2560
IDE_BOARD_NAME = arduino:avr:mega
ARDUINO_PORT = `ls /dev/ttyACM*`
ARDUINO_LIBS = Servo Wire MemoryFree
IDE_MAKE = arduino --port ${ARDUINO_PORT} --board ${IDE_BOARD_NAME}

include /usr/share/arduino/Arduino.mk

ide:
	arduino `pwd`/gunnar.ino &

# .PHONY directives specify that this is an action to take,
# not a real target to build (so make won't just claim the target
# is up-to-date).

.PHONY: go
go:
	${IDE_MAKE} --upload gunnar.ino
	
upload: go

.PHONY: stop
stop:
	${IDE_MAKE} --upload stop/stop.ino 
