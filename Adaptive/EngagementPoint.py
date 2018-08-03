
import math
import pyclipper
import ProcessFeature
import time
from matplotlib.mlab import path_length

global currentPathIndex
global currentSegmentIndex
global segmentLength
global segmentPos


global boundary_paths
global stepScaled
global cp
global toolGeometry
global scale_factor
global last_gui_update
global pathLengths
global pathDistanceTraveled

pathLengths = []
currentPathIndex = 0
currentSegmentIndex = 0
segmentLength = 0
segmentPos = 0
pathDistanceTraveled=0

boundary_paths = []


def Initialize( boundary, toolRadiusScaled, p_stepScaled,p_scale_factor):
    global boundary_paths
    global currentPathIndex
    global currentSegmentIndex
    global segmentLength
    global segmentPos
    global stepScaled
    global cp
    global toolGeometry
    global scale_factor
    global last_gui_update
    global pathLengths
    global pathDistanceTraveled

    scale_factor = p_scale_factor
    #generate tool geometry
    cp = pyclipper.Pyclipper()
    of=pyclipper.PyclipperOffset()
    of.AddPath([[0,0],[0,0]], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    toolGeometry = of.Execute(toolRadiusScaled-1)[0]

    stepScaled = p_stepScaled
    boundary_paths = boundary
    currentPathIndex = 0
    currentSegmentIndex = 0
    segmentPos = 0
    segmentLength = calcSegmentLength(getCurrentSegment())
    pathDistanceTraveled=0
    last_gui_update = time.time()

    #calculate path lengths, used to find out if we reached full circle
    pathLengths=[0] * len(boundary_paths)
    for phi in range(0,len(boundary_paths)):
        pth = boundary_paths[phi]
        for si in range(0,len(pth)):
            seg = [pth[si-1], pth[si]]
            sl = calcSegmentLength(seg)
            pathLengths[phi]=pathLengths[phi]+sl

def getCurrentSegment():
    pth = boundary_paths[currentPathIndex]
    pt1 = pth[currentSegmentIndex-1]
    pt2 = pth[currentSegmentIndex]
    return [pt1, pt2]

def calcSegmentLength(seg):
    return math.sqrt((seg[1][0]-seg[0][0])*(seg[1][0]-seg[0][0]) + (seg[1][1]-seg[0][1])*(seg[1][1]-seg[0][1]))

def getSegmentPoint(seg, distance_from_start, length):
    return [seg[0][0] + (seg[1][0] - seg[0][0])*distance_from_start/length, seg[0][1] + (seg[1][1] - seg[0][1])*distance_from_start/length]


def translatePath(path, pt):
    output = []
    for p in path:
        output.append([p[0]+pt[0], p[1] + pt[1]])
    return output

def nextPath():
    global currentPathIndex
    global currentSegmentIndex
    global segmentLength
    global segmentPos
    global pathDistanceTraveled

    hasMore = True
    currentPathIndex = currentPathIndex + 1
    currentSegmentIndex=0
    if currentPathIndex >= len(boundary_paths):
        currentPathIndex = 0
        currentSegmentIndex = 0
        hasMore = False
    segmentPos=0
    segmentLength = calcSegmentLength(getCurrentSegment())  # new current segment length
    pathDistanceTraveled=0
    return hasMore

def moveForward(distance):
    global currentPathIndex
    global currentSegmentIndex
    global segmentLength
    global segmentPos
    global pathDistanceTraveled
    global pathLengths
    if not distance > 0:
        raise Exception("Distance must be positive")

    pathDistanceTraveled= pathDistanceTraveled+distance
    while segmentPos + distance >= segmentLength:
        currentSegmentIndex = currentSegmentIndex + 1
        if currentSegmentIndex >= len(boundary_paths[currentPathIndex]): #goto next path if neccesary
            currentSegmentIndex=0
        distance = distance - (segmentLength-segmentPos)  # remaining distance
        segmentPos = 0
        segmentLength = calcSegmentLength(getCurrentSegment())  # new current segment length
    segmentPos = segmentPos + distance

    #print "currentPathIndex:", currentPathIndex, " pathDistanceTraveled:",pathDistanceTraveled, " total len:", pathLengths[currentPathIndex]
    return pathDistanceTraveled<=pathLengths[currentPathIndex]+stepScaled

def getCurrentPoint():

    curSeg = getCurrentSegment()
    #print "segment:", curSeg, " currentPathIndex:", currentPathIndex, " currentSegmentIndex:",  currentSegmentIndex, " segmentLength:", segmentLength, " segmentPos:", segmentPos
    pt = getSegmentPoint(curSeg, segmentPos, segmentLength)
    directionV = [1.0*(curSeg[1][0]-curSeg[0][0])/segmentLength,
                  1.0*(curSeg[1][1]-curSeg[0][1])/segmentLength]

    return pt,directionV

def currentPointDistanceTo(pt):
    cpt,dr = getCurrentPoint()
    return math.sqrt((cpt[0]-pt[0])*(cpt[0]-pt[0]) +
                     (cpt[1]-pt[1])*(cpt[1]-pt[1]))


def moveToClosestPoint(pt):
    global currentPathIndex
    global currentSegmentIndex
    global segmentLength
    global segmentPos
    global pathDistanceTraveled
    minDistPathIndex= currentPathIndex
    minDistSegmentIndex= currentSegmentIndex
    minDistSegmentLength= segmentLength
    minDistSegmentPos = segmentPos
    minDistance = 1e32
    while True:
        while moveForward(stepScaled):
            dist = currentPointDistanceTo(pt)
            if dist < minDistance:
                minDistPathIndex = currentPathIndex
                minDistSegmentIndex = currentSegmentIndex
                minDistSegmentLength = segmentLength
                minDistSegmentPos = segmentPos
                minDistance = dist
        if not nextPath(): break

    currentPathIndex=minDistPathIndex
    currentSegmentIndex=minDistSegmentIndex
    segmentLength=minDistSegmentLength
    segmentPos = minDistSegmentPos
    pathDistanceTraveled=0


def moveForwardToCutThreshold(cleared, stepDistance, toReachArea, maxArea):
    global last_gui_update
    passes=0
    while True:
        if not moveForward(stepDistance): #reached the end of current path
            if not nextPath(): #reached the end of all paths
                passes=passes+1
                if passes>1: return False

        cpt, dr = getCurrentPoint()
        toolInPos = translatePath(toolGeometry,cpt) #move tool to current position
        # calculate cut area in the current poisiton
        cp.Clear()
        cp.AddPath(toolInPos, pyclipper.PT_SUBJECT, True)
        cp.AddPaths(cleared, pyclipper.PT_CLIP, True)
        cuttingPolygon=cp.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        tryCuttingArea = 0
        if time.time() - last_gui_update > ProcessFeature.GUI_UPDATE_PERIOD:
            ProcessFeature.sceneClearPaths("POLY")
            ProcessFeature.showFillTool("POLY", cpt, scale_factor, (0, 1, 0))
            ProcessFeature.sceneUpdateGui()
            last_gui_update = time.time()
        for poly in cuttingPolygon:
            tryCuttingArea = tryCuttingArea + math.fabs(pyclipper.Area(poly))

        if tryCuttingArea > toReachArea and tryCuttingArea<maxArea:
            return True


