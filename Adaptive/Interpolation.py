import numpy as np
import math
import random
import ProcessFeature
angles = []
areas = []
allowed_range=[-math.pi/2,math.pi/2]
def reset():
    del angles[:]
    del areas[:]

    # max egnage angle, must be smaller than INITIAL_ENGAGE_ANGLE
    allowed_range[0] = - math.pi / 4
    allowed_range[1] =  math.pi / 4  # max disengage angle
    pass


def getNextAngle(iteration, targetArea, predictedAngle, max_interations):


    #print areas,angles
    if iteration == 1:  # first we try the predicted angle for performance
        angle = predictedAngle
    elif iteration == 2: #measure for max engage angle
        angle=allowed_range[0]
    elif iteration == 4: #measure for min engage angle
        angle=allowed_range[1]
    elif len(angles) < 2:
        angle = allowed_range[0] + 0.00000001 + \
             (allowed_range[1] - allowed_range[0]) * random.random()
    elif iteration>6 and iteration<max_interations-1 and iteration %3 == 0 :  #try random every now and then
        angle = allowed_range[0] + 0.00000001 + \
             (allowed_range[1] - allowed_range[0]) * random.random()
    else:
        #we have more than 2 points, we can interpolate
        if targetArea < areas[0] or targetArea > areas[-1]: #outside known points -> do poly fit
            angle=np.poly1d(np.polyfit(areas, angles, 1))(targetArea)
            #print "outside1:", angle, areas,angles, targetArea
        else:
            angle = np.interp(targetArea, areas, angles)

    # not we have angle to potentially try, clamp it if necessary
    clamped = False
    if angle < allowed_range[0]:
        angle = allowed_range[0]
        clamped = True
    if angle > allowed_range[1]:
        angle = allowed_range[1]
        clamped = True

    return angle, clamped

## adds point keeping the incremental order of angles
def addPoint(iteration, area, angle):
    #print "ADD POINT:", iteration, "angle:", angle, "Area:", area, " IsAllowed:", isAllowed
    if len(areas) == 0:
        angles.append(angle)
        areas.append(area)
        return
    #insert interpolation pair at the right position
    if area > areas[-1]:
        angles.append(angle)
        areas.append(area)
        return
    for i in range(0, len(areas)):
        if area < areas[i]:
            angles.insert(i , angle)
            areas.insert(i, area)
            return
        if area == areas[i]: # update existing point
            #angles[i]=angle
            return
    #should not get to here
    print "ERROR: could not add point", " area:", area, " angle:", angle, " areas: ", areas, " angles: ", angles
    raise Exception('ERROR: could not add point')
