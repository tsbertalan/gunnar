
.PHONY: .Force

.FORCE:
GUNNAR_MAKE = cd src/arduino/sketches/gunnar && make

all:
	$(GUNNAR_MAKE)
	
upload:
	$(GUNNAR_MAKE) upload
	
clean: cleanPyc
	$(GUNNAR_MAKE) clean
	
monitor:
	$(GUNNAR_MAKE) monitor

stop: .FORCE
	python bin/stop.py

go: upload

cleanPyc:
	find . -name "*.pyc" -type f -delete
	find . -name "*.pyo" -type f -delete

# Flash Raspbian Jessie image to SD card.
sdx = zenity --entry --text="Drive name (e.g. 'sdc' in /dev/sdc):" --entry-text=sdc
flash:
	sudo dcfldd bs=4M if=2016-05-27-raspbian-jessie.img of=/dev/`$(sdx)`
	sync
verify:
	sudo dcfldd bs=4M if=/dev/`$(sdx)` of=from-sd-card.img count=2048
	sync
	sudo truncate --reference 2016-05-27-raspbian-jessie.img from-sd-card.img
	sudo diff -s from-sd-card.img 2016-05-27-raspbian-jessie.img

