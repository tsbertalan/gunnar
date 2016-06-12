
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
ssdx = zenity --entry --text="Drive name (e.g. 'sdc' in /dev/sdc):" --entry-text=sdc > /tmp/sdx_id
sdx = cat /tmp/sdx_id
spathToFlash = zenity --entry --text="Path to img file to flash:" --entry-text="$(HOME)/Downloads/robotics/2016-05-27-raspbian-jessie.img" > /tmp/ptf_p
pathToFlash = cat /tmp/ptf_p
flash:
	`$(ssdx)`
	`$(spathToFlash)`
	sudo dcfldd bs=4M if=`$(pathToFlash)` of=/dev/`$(sdx)`
	sync
verify:
	`$(ssdx)`
	`$(spathToFlash)`
	sudo dcfldd bs=4M if=/dev/`$(sdx)` of=/tmp/from-sd-card.img count=1024
	sync
	sudo truncate --reference `$(pathToFlash)` /tmp/from-sd-card.img
	sudo diff -s /tmp/from-sd-card.img `$(pathToFlash)`

vagrantSaveKey:
	vagrant up | tee /tmp/vagrant.out
	grep ssh-rsa /tmp/vagrant.out | sed 's/==> gunnar: //g' >> ~/.ssh/authorized_keys
