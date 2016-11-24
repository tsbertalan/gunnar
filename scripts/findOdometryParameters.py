'''From a set of experiments, estimate the true odometry parameters.'''
from math import sin, cos
metersPerInch = 2.54 / 100. 

# Initial guesses.
LEFT_CLICKS_PER_M = RIGHT_CLICKS_PER_M = 202.6 * 100
WHEEL_BASE = 11.5 * metersPerInch

# Ground truth translation for the experiments.
translationGroundTruth = (-56. * metersPerInch, 133.25 * metersPerInch, 0)

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

for enclFinal, encrFinal in [
    (65347, 66034),
                             ]:
    print getTranslation(enclFinal, encrFinal), translationGroundTruth
