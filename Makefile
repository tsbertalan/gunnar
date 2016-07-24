
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
	
downloadDiskimage:
	wget http://ftp.jaist.ac.jp/pub/raspberrypi/raspbian/images/raspbian-2016-05-31/2016-05-27-raspbian-jessie.zip
	
unzipDiskImage:
	rm 2016-05-27-raspbian-jessie.img || true
	unzip 2016-05-27-raspbian-jessie.zip
	sync
	
# Load config file.
include makeconfig.mk

# Modify image file.
mountAndAlterSD:
	scripts/utils/mountSDcard.sh $(IMGPATH) $(MNTPOINT)
	sudo scripts/utils/doctorSDcard.sh $(MNTPOINT) $(USER)
	sync
	sudo umount $(MNTPOINT)
	
# Flash Raspbian Jessie image to SD card.
flash:
	sudo dcfldd bs=4M if=$(IMGPATH) of=$(SDX)
	sync
verify:
	sudo dcfldd bs=4M if=$(SDX) of=/tmp/from-sd-card.img count=1024
	sync
	sudo truncate --reference $(IMGPATH) /tmp/from-sd-card.img
	sudo diff -s /tmp/from-sd-card.img $(IMGPATH)
	sudo rm /tmp/from-sd-card.img

vagrantSaveKey:
	vagrant up | tee /tmp/vagrant.out
	grep ssh-rsa /tmp/vagrant.out | sed 's/==> gunnar: //g' >> ~/.ssh/authorized_keys
