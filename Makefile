
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
dlUbuntu:
	wget http://www.finnie.org/software/raspberrypi/ubuntu-rpi3/ubuntu-16.04-preinstalled-server-armhf+raspi3.img.xz
	

# Flash Raspbian Jessie image to SD card.
sdx = zenity --entry --text="Drive name (e.g. 'sdc' in /dev/sdc):" --entry-text=sdc
pathToFlash = zenity --entry --text="Path to img file to flash:" --entry-text="$(HOME)/Downloads/robotics/2016-05-27-raspbian-jessie.img"
flash:
	sudo dcfldd bs=4M if=`$(pathToFlash)` of=/dev/`$(sdx)`
	sync
verify:
	sudo dcfldd bs=4M if=/dev/`$(sdx)` of=from-sd-card.img count=2048
	sync
	sudo truncate --reference `$(pathToFlash)` from-sd-card.img
	sudo diff -s from-sd-card.img `$(pathToFlash)`

vagrantSaveKey:
	vagrant up | tee /tmp/vagrant.out
	grep ssh-rsa /tmp/vagrant.out | sed 's/==> gunnar: //g' >> ~/.ssh/authorized_keys
