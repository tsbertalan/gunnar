
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
