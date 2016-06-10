flash:
	sudo dcfldd bs=4M if=2016-05-27-raspbian-jessie.img of=/dev/sdc
	sync

verify:
	sudo dcfldd bs=4M if=/dev/sdc of=from-sd-card.img count=2048
	sync
	sudo truncate --reference 2016-05-27-raspbian-jessie.img from-sd-card.img
	sudo diff -s from-sd-card.img 2016-05-27-raspbian-jessie.img

startVM:
	vagrant up

