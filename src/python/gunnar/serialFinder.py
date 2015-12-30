from config import systemOut

devices = [x for x in systemOut(["ls", "/dev/"], sayCmd=False).split('\n')
           if len(x) > 0 and "ACM" in x]
if len(devices) == 0:
    raise IOError('No /dev/ttyACM* device found.')
else:
    for i, dev in enumerate(devices):
        if "ACM" in dev:
            port = "/dev/%s" % dev.strip()
del i, dev, devices
