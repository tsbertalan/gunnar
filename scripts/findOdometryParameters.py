'''From a set of experiments, estimate the true odometry parameters.'''
import numpy as np
from scipy.optimize import fsolve, minimize
sin = np.sin
cos = np.cos
metersPerInch = 2.54 / 100. 

# Initial guesses.
LEFT_CLICKS_PER_M = RIGHT_CLICKS_PER_M = 202.6 * 100
WHEEL_BASE = wbfixed = 11.5 * metersPerInch

# Define a function the root of which we must find.
def rootFunction(args):
    args[2] = wbfixed
    LEFT_CLICKS_PER_M, RIGHT_CLICKS_PER_M, WHEEL_BASE = args
    # Ground truth translation for the experiments.
    endPoseGroundTruth = (-56. * metersPerInch, 133.25 * metersPerInch, 0)
    ticks = [
             (65347, 66034),
             (63357, 61538),
             (62611, 60310),
             (66865, 66054),
             (74641, 74341),
             (68325, 67558),
                             ]
    endPoses = np.empty((len(ticks), 3))
    
    def getTranslation(enclFinal, encrFinal):
        '''Simulate odometry.'''
        assert enclFinal > 0 and encrFinal > 0
        x = y = theta = 0.0
        lsamp = rsamp = last_left = last_right = 0
        
        for i in range(max(enclFinal, encrFinal)):
            if lsamp < enclFinal:
                lsamp += 1
            if rsamp < encrFinal:
                rsamp += 1
                
            # determine how many ticks since our last sampling?
            L_ticks = lsamp - last_left
            R_ticks = rsamp - last_right
    
            # and update last sampling for next time
            last_left = lsamp 
            last_right = rsamp 
    
            # convert longs to floats and ticks to meters
            left_meters = float(L_ticks) / LEFT_CLICKS_PER_M
            right_meters = float(R_ticks) / RIGHT_CLICKS_PER_M
    
            # calculate distance we have traveled since last sampling
            meters = (left_meters + right_meters) / 2.0
        
            # accumulate total rotation around our center (radians)
            oldTheta = theta
            theta += (left_meters - right_meters) / WHEEL_BASE
    
            # and clip the rotation to plus or minus 360 degrees
            # We could perhaps use modulo for this, but Python's modulo operator
            # is "improved" s.t.  
            theta -= float(
                int(
                    theta / 6.283185307179586
                    )
                ) * 6.283185307179586
    
            # now calculate and accumulate our position in meters
            oldx = x
            oldy = y
            y += meters * cos(theta) 
            x += meters * sin(theta)
            
        return x, y, theta
    
    for i in range(len(ticks)):
        enclFinal, encrFinal = ticks[i]
        endPoses[i, :] = getTranslation(enclFinal, encrFinal)
    
    error = endPoses.mean(0) - endPoseGroundTruth
    return error

iguess = np.array((LEFT_CLICKS_PER_M, RIGHT_CLICKS_PER_M, WHEEL_BASE))

# Try the initial guess out.
print 'Inital guess:'
print iguess

print 'Initial guess error:'
print rootFunction([LEFT_CLICKS_PER_M, RIGHT_CLICKS_PER_M, WHEEL_BASE])

# Solve the function.
soln = fsolve(rootFunction, iguess)
print 'Solution:'
print soln
print 'Solution error:'
print rootFunction(soln)



# Alternately, solve a regularized least-squares problem.
def objectiveFunction(args, l1=1./np.mean(iguess)**.5, l2=0.0):
    
    return np.linalg.norm(rootFunction(args))\
        + np.linalg.norm(args, ord=1) * l1\
        + np.linalg.norm(args, ord=2) * l2
        
print 'Initial objective value:'
print objectiveFunction(iguess)    
        
print 'minimize result:'
print minimize(objectiveFunction, iguess)