from sys import argv
import numpy as np
import matplotlib.pyplot as plt
#fname = argv[1]
fname = "serial1443586325.out"
fname = "serial1443586325.out"

lines = [l.strip() for l in open(fname).readlines() if len(l.strip())>0]
lines = [line for line in lines if line.count("#") > 4]

#1443586329.94#    #105.54#4.40#110.08#-9.34#1.51#-9.34#1.51#110.08#-9.34#1.51#110.08#-5#138#2#26#138#1#1

data = []
for line in lines:
    try:
        data.append([
                     float(x.strip())
                     for x in line.split("#")
                     if len(x.strip()) > 0
                    ])
    except ValueError:
        print "failed line: %s" % line

data = np.array(data)
print data.shape

labels = [
            "t",
            "getSonarDist",
            "getHeading", "heading", "roll", "pitch",
            
            "x", "y", "z",
            "v0", "v1", "v2",
            
            "encoder0.position",
            "motor1.getSpeed",
            "motor1.getStatus",
            
            "encoder1.position",
            "motor2.getSpeed",
            "motor2.getStatus",
            
            "isTurning"
            ]
ddata = {label: x for (label, x) in zip(labels, data.T)}
print ddata["encoder0.position"]
            
fig = plt.figure()

separatedLabelsLists = [
                            [
                                "getSonarDist",
                                "getHeading", "heading", "roll", "pitch",
                                "isTurning"
                            ],
                            [
                                "x", "y", "z",
                                "encoder0.position",
                                "encoder1.position",
                            ],
                            [
                                "v0", "v1", "v2",
                                "motor1.getSpeed",
                                "motor1.getStatus",
                                "motor2.getSpeed",
                                "motor2.getStatus",
                            ]
                        ]

nfigs = len(separatedLabelsLists)
A = [fig.add_subplot(nfigs,1,i+1) for i in range(nfigs)]

a = A[0]

            
            
for a, slabels in zip(A, separatedLabelsLists):
    print slabels
    for label in slabels:
        print label
        x = ddata[label]
        x /= np.abs(x).max()
        a.plot(ddata["t"]-ddata["t"].min(), x, label=label)
        if a is not A[-1]:
            a.set_xticks([])
        a.legend(loc="best")
    
A[-1].set_xlabel("time [sec]")
    
plt.show()
