import numpy as np
import matplotlib.pyplot as plt

class Scene:
    obstructions = []
    
    def display(self, figax=None, resolution=1024, 
                xdom=(-20, 20), ydom=(-20, 20)):
        if figax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111)
        else:
            fig, ax = figax
        X = np.linspace(xdom[0], xdom[1], resolution)
        Y = np.linspace(ydom[0], ydom[1], resolution)
        Xg, Yg = np.meshgrid(X, Y)
        Z = np.zeros(Xg.shape).astype(bool)
        for obs in self.obstructions:
            Z = np.logical_or(Z, obs.present(Xg, Yg))
        ax.imshow(Z, extent=[xdom[0], xdom[1], ydom[0], ydom[1]],
                  interpolation=None, origin='lower')
        return (fig, ax)

    def obstructed(self, x, y):
        for obs in self.obstructions:
            if obs.present(x, y):
                return True
        return False
    
    def Trace(self, *args, **kwargs):
        return Trace(self, *args, **kwargs)
    
class Trace:
    def __init__(trace, scene, location, facingAngle=0, scanRad=45, scanSteps=128, dDist=.01, maxDist=20, minDist=1, distNoise=0, angleNoise=0):
        x, y = location
        for obs in scene.obstructions:
            assert not obs.present(x, y), "Collided with object %s at loc %s." % (obs, (x, y))
       
        facingAngle = facingAngle * np.pi / 180.0
        scanRad = scanRad * np.pi / 180.0
        thetas = np.linspace(facingAngle-scanRad, facingAngle+scanRad, scanSteps)
        dists = np.zeros(thetas.shape)
        X = np.empty(thetas.shape)
        Y = np.empty(thetas.shape)
        for i, t in enumerate(thetas):
            
            trand = (np.random.rand()-.5)*angleNoise
            t += trand
            drand = (np.random.rand()-.5)*distNoise
            for d in np.arange(0, maxDist, dDist):
                d += drand
                d = max(d, minDist)
                if d == minDist:
                    d = 1.0
                rayX = x + np.cos(t) * d
                rayY = y + np.sin(t) * d
                if scene.obstructed(rayX, rayY):
                    break
            dists[i] = d
            X[i] = rayX
            Y[i] = rayY
        trace.dists = dists
        trace.thetas = thetas
        trace.X = X
        trace.Y = Y
           
            
        
class Obstruction:
    def __init__(centroid):
        self.centroid = np.array(centroid).astype(float)
    def present(x, y):
        raise NotImplementedError("virtual")

class Column(Obstruction):
    def __init__(self, center, radius):
        self.centroid = np.array(center).astype(float)
        self.center = self.centroid
        self.radius = radius

    def present(self, x, y):
        x0  = self.centroid[0]  
        y0  = self.centroid[1]  
        r = np.sqrt(((x - x0)**2 + (y - y0)**2))
        return r <= self.radius

class Corner(Obstruction):
    def __init__(self, point, rotation, angle=90):
        self.centrold = self.point = np.array(point).astype(float)
        self.rotation = rotation*np.pi/180.0
        self.angle = angle*np.pi/180.0

    def present(self, x, y):
        m1 = np.tan(self.rotation)
        m2 = np.tan(self.rotation + self.angle)
        x0  = self.point[0]  
        y0  = self.point[1]  
        def c1(x, y):
            return (m1 * (x - x0) + y0 ) <= y
        def c2(x, y):
            return ((y - y0)/m2 + x0) <= x
        return np.logical_and(c1(x, y), c2(x, y))
    
columns = [Column((-2, 2), 2), Column((-11, -6), 1)]
commonAngle = 30
corners = [Corner((3, 2), -commonAngle), Corner((-4, 16), -commonAngle)]

scene = Scene()
scene.obstructions.extend(columns)
scene.obstructions.extend(corners)

fig = plt.figure(figsize=(5,12))
ax1 = fig.add_subplot(2,1,1)

robotLocation = (2, -3)
facingAngle = 90 
minDist = 8
maxDist = 20
trace = scene.Trace(robotLocation, scanRad=110, facingAngle=facingAngle, distNoise=1.0, angleNoise=.10, scanSteps=256, minDist=minDist, maxDist=maxDist)

ax1.scatter(trace.X, trace.Y, color="yellow", s=1)
ax1.scatter(robotLocation[0], robotLocation[1], color="red", marker="d", s=20)
ax1.text(robotLocation[0], robotLocation[1], "robot", color="red")

ax3 = fig.add_subplot(2,1,2)
order = np.argsort(trace.thetas)
tmod = -(trace.thetas*180/np.pi - facingAngle)

ax3.plot(tmod[order], trace.dists[order], color="blue")
#ax3.scatter(tmod, trace.dists, color="blue")
condition = trace.dists > 1.01*minDist#, trace.dists < 0.90 * maxDist)
ax3.scatter(tmod[condition], trace.dists[condition], color="red")
ax3.set_xlabel(r"$-\theta$"); ax3.set_ylabel("$d$")

scene.display(figax=(fig, ax1))
#ax.scatter([0], [0], 'y')

plt.show()







