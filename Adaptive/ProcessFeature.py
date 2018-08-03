import pyclipper
import Path
import math
## NEW VERSION
import Plot
from FreeCAD import Console
import FreeCADGui
from PySide import QtCore, QtGui
from pivy import coin
import numpy as np
import time
import Interpolation
import random
import EngagementPoint


global toolGeometry
global smallDotGeometry
global mediumDotGeometry
#enlargedToolGeometry
global optimalCutAreaPerDist
global cleared
global iteration_limit_count
global total_point_count
global total_iteration_count
global tool_enlarge_scaled
global toolRadiusScaled
global last_gui_update_time

last_gui_update_time = time.time()

total_iteration_count=0
tool_enlarge_scaled = 20

INITIAL_ENGAGE_ANGLE = math.pi / 32 #must be larger than interpolation negative max engage angle
GUI_UPDATE_PERIOD = 0.2
SHOW_MARKERS = False
def messageBox(msg):
    # Create a simple dialog QMessageBox
    # The first argument indicates the icon used: one of QtGui.QMessageBox.{NoIcon, Information, Warning, Critical, Question}
    diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning, 'Info', msg)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    diag.exec_()
    FreeCADGui.updateGui()


def confirmMessage(msg):
    FreeCADGui.updateGui()
    reply = QtGui.QMessageBox.question(None, "", msg, QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)
    if reply == QtGui.QMessageBox.Yes: return True
    return False


###################################################
# a progress marker thingy
###################################################
global sg
#global markerNode
#global markerTrans
global topZ
scenePathNodes = {}

def sceneInit(toolRadius):
    global sg
    sg = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()

def sceneDrawPaths(grpName, paths, scale_factor, color=(0,0,1), closed=False):
    for path in paths:
        sceneDrawPath(grpName, path, scale_factor, color, closed)


def sceneUpdateGui():
    global last_gui_update_time
    if time.time()-last_gui_update_time>GUI_UPDATE_PERIOD:
        FreeCADGui.updateGui()
        last_gui_update_time=time.time()

def anyValidPath(paths):
    for pth in paths:
        if len(pth)>0: return True
    return False


def sceneDrawPath(grpName, path, scale_factor, color=(0,0,1), closed=False):
    global sg
    global topZ
    global scenePathNodes
    coPoint = coin.SoCoordinate3()
    pts = []
  #make sure its closed
    if closed and len(path) > 1:
        pt = path[-1]
        pts.append([1.0*pt[0]/scale_factor, 1.0*pt[1]/scale_factor, topZ])
    for pt in path:
        pts.append([1.0*pt[0]/scale_factor,1.0*pt[1]/scale_factor, topZ])


    coPoint.point.setValues(0,len(pts),pts)
    ma = coin.SoBaseColor()
    ma.rgb = color
    li = coin.SoLineSet()
    li.numVertices.setValue(len(pts))
    pathNode = coin.SoSeparator()
    pathNode.addChild(coPoint)
    pathNode.addChild(ma)
    pathNode.addChild(li)
    sg.addChild(pathNode)
    if not scenePathNodes.has_key(grpName):
        scenePathNodes[grpName] = []
    scenePathNodes[grpName].append(pathNode)

def sceneDrawFilledPaths(grpName, paths, scale_factor, color=(0,0,1)):
    global sg
    global topZ
    global scenePathNodes

    for i in range(0, len(paths)):
        path= paths[i]
        pts=[]
        for pt in path:
            pts.append([1.0*pt[0]/scale_factor,1.0*pt[1]/scale_factor, topZ])
        coPoint = coin.SoCoordinate3()
        coPoint.point.setValues(0,len(pts),pts)
        ma = coin.SoMaterial()
        ma.diffuseColor = color
        if i==0:
            ma.transparency.setValue(0.9)
        else:
            ma.transparency.setValue(0.8)

        hints = coin.SoShapeHints()
        hints.faceType = coin.SoShapeHints.UNKNOWN_FACE_TYPE
        #hints.vertexOrdering = coin.SoShapeHints.CLOCKWISE

        li = coin.SoIndexedFaceSet()
        li.coordIndex.setValues(range(0,len(pts)))
        pathNode = coin.SoSeparator()
        pathNode.addChild(hints)
        pathNode.addChild(coPoint)
        pathNode.addChild(ma)
        pathNode.addChild(li)
        sg.addChild(pathNode)
        if not scenePathNodes.has_key(grpName):
            scenePathNodes[grpName] = []
        scenePathNodes[grpName].append(pathNode)

def sceneClean():
    global sg
    #sg.removeChild(markerNode)
    for k in scenePathNodes.keys():
        for p in scenePathNodes[k]:
            sg.removeChild(p)
    FreeCADGui.updateGui()

def sceneClearPaths(grpName):
    if scenePathNodes.has_key(grpName):
        for p in scenePathNodes[grpName]:
            sg.removeChild(p)


def closePath(path):
    if len(path)==0: return path
    lastPtIndex=len(path)-1
    if path[lastPtIndex][0] != path[0][0] or path[lastPtIndex][1] != path[0][1]:
        path.append([path[0][0],path[0][1]])
    return path

def genToolGeomery(toolRadiusScaled):
    of=pyclipper.PyclipperOffset()
    of.AddPath([[0, 0], [0, 0]], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    geo = of.Execute(toolRadiusScaled)[0]
    return pyclipper.CleanPolygon(geo)


def translatePath(path,pt):
    output = []
    for p in path:
        output.append([p[0]+pt[0],p[1] + pt[1]])
    return output

def translatePaths(paths, pt):
    outpaths=[]
    for path in paths:
        output = []
        for p in path:
            output.append([p[0] + pt[0], p[1] + pt[1]])
        outpaths.append(output)
    return outpaths

###########################################################
# Check if tool position if within allowed area
############################################################
def isOutsideCutRegion(toolPos,cut_region_tp, include_boundary=False):
    #check if we reached the end of cut area - end of this continuous pass
    for i in range(0,len(cut_region_tp)):
        pth =  cut_region_tp[i]
        pip = pyclipper.PointInPolygon(toolPos, pth)
        if pip == -1 and include_boundary:
            return True
        if i==0:
            if pip==0: return True # must be inside fist path
        else:
            if pip == 1: return True  # must not be inside all other (holes)
    return False

def centroid(path):
    c=[0,0]
    sa=0
    x0=0
    y0=0
    x1=0
    y1=0
    a=0
    cnt=len(path)
    path=closePath(path)    
    for i in range(0, cnt):
        x0=path[i][0]
        y0=path[i][1]
        x1=path[(i+1) % cnt][0]
        y1=path[(i+1 % cnt)][1]
        a=x0*y1-x1*y0
        sa = sa +a
        c[0]=c[0] + (x0+x1)*a
        c[1]=c[1] + (y0+y1)*a
    sa = 3*sa
    c[0] = c[0] / sa
    c[1] = c[1] / sa
    return c

def findStartPointRampEntry(op,feat_num, cut_region_tp, helixRadius, toolRadius,scale_factor):
    of=pyclipper.PyclipperOffset()
    #showPath(op,[outerPath],scale_factor)

    #searching for biggest area to cut, by decremental offseting from target cut_region

    #start offset is max x or y size of the stock/2
    maxLen = max(op.stock.Shape.BoundBox.XLength,op.stock.Shape.BoundBox.YLength)
    starting_offset= maxLen*scale_factor/2

    #try with some reasonable step
    step = toolRadius/4

    while starting_offset>=helixRadius:
        of.Clear()
        of.AddPaths(cut_region_tp, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
        offsetPaths=of.Execute(-(starting_offset))
        #showPath(op,offsetPaths,scale_factor)
        for path in offsetPaths:
            #find center
            pt=centroid(path)
            #sceneDrawPath("STP", path, scale_factor, (1,0,1))
            #showTool("STP", pt, scale_factor, (1,0,1))
            if not isOutsideCutRegion(pt,cut_region_tp):
                    of.Clear()
                    of.AddPath([pt,[pt[0]+1,pt[1]]], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
                    cleared_helix=of.Execute(helixRadius + toolRadius)
                    Console.PrintMessage("Start point: %f,%f\n"%(1.0*pt[0]/scale_factor,1.0*pt[1]/scale_factor))
                    return cleared_helix,pt
        starting_offset=starting_offset-step

    Console.PrintError("Unable to find starting point (for path no:%d)!\n"%feat_num)
    #sceneClearPaths("STP")
    return None,None

def getDirectionVAt(path, index):
    p1i=index-1
    p2i=index
    p3i=index+1
    p4i=index+2
    if p3i>len(path)-1: p3i=p3i-(len(path)-1)
    if p4i>len(path)-1: p4i=p4i - (len(path)-1)

    #find delta vectors between points
    pt1=path[p1i]
    pt2=path[p2i]
    pt3=path[p3i]
    pt4=path[p4i]


    v1 =[pt2[0]-pt1[0],pt2[1]-pt1[1]]
    v2 =[pt3[0]-pt2[0],pt3[1]-pt2[1]]
    v3 =[pt4[0]-pt3[0],pt4[1]-pt3[1]]
    #add those two vectors - for smoothing of angles at corners
    v=[v1[0]+v2[0]+v3[0],v1[1]+v2[1]+v3[1]]
    #v =[pt2[0]-pt1[0],pt2[1]-pt1[1]]
    #print pt2,pt1
    #print v
    #normalize
    d=math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]

def normalize(v):
    d=math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]

def getDirectionV(pt1,pt2):
    #find delta vector between points
    v =[pt2[0]-pt1[0],pt2[1]-pt1[1]]
    #normalize
    d=math.sqrt(v[0]*v[0] + v[1]*v[1])
    return [v[0]/d, v[1]/d]

def getAngle(v):
    return math.atan2(v[1], v[0])

def magnitude(v):
    return math.sqrt(v[0]*v[0] + v[1]*v[1])

#get angle between two vectors
def getAngle2v(v1,v2):
    try:
        d=(v1[0]*v2[0] + v1[1]*v2[1])
        m=(math.sqrt(v1[0]*v1[0] + v1[1]*v1[1]))*(math.sqrt(v2[0]*v2[0] + v2[1]*v2[1]))
        if m!=0:
            return math.acos(d/m)
        else:
            return math.pi/32
    except:
        #print "matherror",v1,v2
        return math.pi/4

def sub2v(v1,v2):
    return [v1[0] - v2[0], v1[1] - v2[1]]

def sumv(path):
    res = [0, 0]
    for pt in path:
        res[0] = res[0] + pt[0]
        res[1] = res[1] + pt[1]
    return res

def getIntersectionPointLWP(lineSegment,paths):
    l1=lineSegment #first line segment
    for pth in paths:
        if len(pth)>1:            
            for i in range(0, len(pth)):
                l2=[pth[i-1],pth[i]] # second line segment (path line)
                d=(l1[1][1]-l1[0][1])*(l2[1][0]-l2[0][0]) - (l2[1][1]-l2[0][1])*(l1[1][0]-l1[0][0])
                if d==0: # lines are parallel 
                    continue
                p1d=(l2[1][1]-l2[0][1])*(l1[0][0]-l2[0][0]) - (l2[1][0]-l2[0][0])*(l1[0][1]-l2[0][1])
                p2d=(l1[1][0]-l1[0][0])*(l2[0][1]-l1[0][1]) - (l1[1][1]-l1[0][1])*(l2[0][0]-l1[0][0])                
                #clamp 
                if d<0:
                    if (p1d<d or p1d>0): continue #not inside segment
                    if (p2d<d or p2d>0): continue #not inside segment
                else:
                    if (p1d<0 or p1d>d): continue #not inside segment
                    if (p2d<0 or p2d>d): continue #not inside segment
                return [l1[0][0] + (l1[1][0]-l1[0][0])*p1d/d,l1[0][1] + (l1[1][1]-l1[0][1])*p1d/d]
    #nothing found, return None
    return None

def rotate(v,rad):
    c=math.cos(rad)
    s=math.sin(rad)
    return [c*v[0] - s*v[1],s*v[0] + c*v[1]]


# get closest point on path to point
def getClosestPointOnPaths(paths,pt):
    closestPathIndex=0
    closestPtIndex=0
    minDistSq = 1000000000000
    closestPt=[]
    for pthi in range(0,len(paths)):
        path=paths[pthi]
        for i in range(0, len(path)):
            #line segment
            p1=path[i-1]
            p2=path[i]
            #length between points on the line segment
            lsq = (p2[0] - p1[0]) * (p2[0] - p1[0]) + (p2[1] - p1[1]) * (p2[1] - p1[1])
            if lsq == 0: #segment is very short take the distance to one of end points
                distSq = (pt[0] - p1[0]) * (pt[0] - p1[0]) + (pt[1] - p1[1]) * (pt[1] - p1[1])
                clp = p1
            else:
                #((point.x - this.start.x) * (this.end.x - this.start.x) + (point.y - this.start.y) * (this.end.y - this.start.y))
                #parameter of the closest point
                t = (((pt[0] - p1[0]) * (p2[0] - p1[0]) + (pt[1] - p1[1]) * (p2[1] - p1[1])))
                #clamp it
                if t > lsq:
                    t = lsq
                if t < 0: t = 0
                #point on line at t
                clp=[p1[0] + t*(p2[0]-p1[0])/lsq,p1[1] + t*(p2[1]-p1[1])/lsq]
                distSq=(pt[0]-clp[0])*(pt[0]-clp[0]) + (pt[1]-clp[1])*(pt[1]-clp[1])
            if distSq<minDistSq:
                closestPtIndex=i
                closestPathIndex=pthi
                minDistSq = distSq
                closestPt=clp
    return closestPt, math.sqrt(minDistSq)

def aproximateFactor(desiredArea,maxArea):
    if desiredArea>maxArea:
        return 1.0
    else:
        return (math.asin(2.0*desiredArea/maxArea-1.0) + math.pi/2)/math.pi

cache = {}
cache_hit_count = 0
cache_pot_count = 0

def getToolCuttingShape(toolPos,newToolPos,toolRadiusScaled):
    global cache_hit_count
    global cache_pot_count
    global cache

    subv = sub2v(newToolPos, toolPos)
    dist = magnitude(subv)
    if dist < 1:
        return [[]]

    subv = [int(subv[0]), int(subv[1])]  # make discrete to clipper resolution
    key = "K%d-%d" % (subv[0], subv[1])
    cache_pot_count = cache_pot_count + 1
    cache_hit = False
    if cache.has_key(key):
        toolCutShape = cache[key]
        cache_hit_count = cache_hit_count + 1
        cache_hit = True

    if not cache_hit:
        of.Clear()
        of.AddPath([[0, 0], subv], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
        toolCoverArea2 = of.Execute(toolRadiusScaled+1)[0]

        #difference between old tool cutting area and new tool covering area
        cp.Clear()
        cp.AddPath(toolCoverArea2, pyclipper.PT_SUBJECT, True)
        cp.AddPath(toolGeometry, pyclipper.PT_CLIP, True)
        toolCutShape = cp.Execute(
            pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        toolCutShape= pyclipper.CleanPolygons(toolCutShape)
        cache[key] = toolCutShape

    toolCoverArea = translatePaths(toolCutShape, toolPos)

    return toolCoverArea

def calcCutingArea(toolPos, newToolPos, toolRadiusScaled, cleared):

    dist = magnitude(sub2v(newToolPos, toolPos))
    if dist == 0:
        return 0, 0

    toolCoverArea = getToolCuttingShape(toolPos, newToolPos, toolRadiusScaled)

    if anyValidPath(toolCoverArea):
        #calculate area
        cp.Clear()
        cp.AddPaths(toolCoverArea,pyclipper.PT_SUBJECT, True)
        cp.AddPaths(cleared,pyclipper.PT_CLIP, True)
        cuttingPolygon=cp.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)

        cuttingArea=0
        cuttingAreaPerDist=0
        for poly in cuttingPolygon:
            cuttingArea=cuttingArea + math.fabs(pyclipper.Area(poly))
            cuttingAreaPerDist=cuttingArea/dist
        return cuttingArea,cuttingAreaPerDist
    else:
        return 0,0

def closeToOneOfPoints(pt, points, toleranceScaled):
    for p2 in points:
        if magnitude(sub2v(p2, pt)) <= toleranceScaled: return True
    return False



def appendToolPathCheckJump(of,cp,toolPaths,passToolPath,toolRadiusScaled,cleared,close=False):
      #appending pass toolpath to the list of cuts
    #checking the jump line for obstacles
    global output_point_count
    if len(passToolPath)<2: return
    jumpType=1 # assume no lift
    nextPoint = passToolPath[0]
    if len(toolPaths)!=0 and  len(toolPaths[-1])>0: #if this is first path: lift to safe height
        lastPath = toolPaths[-1]
        lastPoint = lastPath[-1]
        #make toolpath shape from last point to next point
        of=pyclipper.PyclipperOffset()
        of.AddPath([lastPoint,nextPoint], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
        toolShape=of.Execute(toolRadiusScaled-2)
        #check clearence
        cp.Clear()
        cp.AddPaths(toolShape,pyclipper.PT_SUBJECT, True)
        cp.AddPaths(cleared,pyclipper.PT_CLIP, True)
        crossing=cp.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        if len(crossing)>0:
           jumpType=2
        #print ("appending jump type",jumpType)
        toolPaths.append([[lastPoint[0], lastPoint[1], jumpType], [nextPoint[0], nextPoint[1], jumpType]])
    passToolPath = pyclipper.CleanPolygon(passToolPath,0.7)
    output_point_count=output_point_count + len(passToolPath)
    if close:
        passToolPath = closePath(passToolPath)
    toolPaths.append(passToolPath)

def showToolDir(grpName, tp, td, scale_factor, color):
    toolInPos = translatePath(toolGeometry, tp)
    sceneDrawPath(grpName, toolInPos, scale_factor, color, True)

    pt2 = [tp[0] + td[0] * toolRadiusScaled*3, tp[1] + td[1] * toolRadiusScaled*3]
    toolInPos = translatePath(smallDotGeometry, pt2)
    sceneDrawPath(grpName, toolInPos, scale_factor, color, True)
    sceneDrawPath(grpName, [tp,pt2], scale_factor,color)

def showTool(grpName, tp, scale_factor, color):
    toolInPos = translatePath(toolGeometry,tp)
    sceneDrawPaths(grpName, [toolInPos], scale_factor,color, True)

def showFillTool(grpName, tp, scale_factor, color):
    toolInPos = translatePath(toolGeometry,tp)
    sceneDrawFilledPaths(grpName, [toolInPos], scale_factor,color)

# def showSmallDot(grpName, tp, scale_factor, color):
#     geomInPos = translatePath(smallDotGeometry,tp)
#     sceneDrawPath(grpName, geomInPos, scale_factor,color)


#######################################################################
# Finds next optimal cutting point
#######################################################################
deflectionAngleHistory=[] #used for better guessing next initial angle
def findNextPoint(obj, op, of, cp, cleared, toolPos, toolDir, toolRadiusScaled, stepScaled,targetAreaPerDist, scale_factor):
    global toolGeometry
    global iteration_limit_count
    global total_point_count
    global total_iteration_count
    global deflection
    global tool_enlarge_scaled
        #find valid next tool pos in the pass
    cuttingAreaPerDist=0
    cuttingArea=0
    iteration=0
    isAllowed=True
    toolCoverArea=[]
    tryCuttingAreaPerDist=0
    tryCuttingArea=0
    MAX_ERROR=40/stepScaled + 2
    max_interations = 12

    Interpolation.reset()
    predictedAngle = 0.0
    if len(deflectionAngleHistory)>0: #calc average
        predictedAngle=reduce(lambda x, y: x + y, deflectionAngleHistory) / float(len(deflectionAngleHistory))

    while True:

        total_iteration_count=total_iteration_count+1
        iteration=iteration+1

        deflectionAngle, clamped = Interpolation.getNextAngle(
            iteration, targetAreaPerDist, predictedAngle, max_interations)

        tryToolDir = rotate(toolDir, deflectionAngle)
        tryToolPos = [toolPos[0] + int(tryToolDir[0] * stepScaled),
                      toolPos[1] + int(tryToolDir[1] * stepScaled)]

        tryCuttingArea,tryCuttingAreaPerDist = calcCutingArea(toolPos, tryToolPos, toolRadiusScaled,cleared)
        error = 1.0*(tryCuttingAreaPerDist - targetAreaPerDist)
        if (math.fabs(error) < MAX_ERROR) or (iteration > max_interations):
            if iteration>max_interations:
                iteration_limit_count=iteration_limit_count+1
                total_iteration_count=total_iteration_count-max_interations #  dont average those
                total_point_count = total_point_count - 1
            else:
                deflectionAngleHistory.append(deflectionAngle)
                if len(deflectionAngleHistory) > 10:
                    deflectionAngleHistory.pop(0)

            toolDir=tryToolDir
            toolPos=tryToolPos
            cuttingAreaPerDist=tryCuttingAreaPerDist
            cuttingArea = tryCuttingArea
            break



        Interpolation.addPoint(iteration, tryCuttingAreaPerDist, deflectionAngle)

    return toolPos, toolDir, cuttingAreaPerDist, cuttingArea, predictedAngle


####################################################
# expand cleared area
####################################################
def expandClearedArea(cp,toolInPosGeometry,cleared):
    try:
        cp.Clear()
        cp.AddPaths(cleared,pyclipper.PT_SUBJECT, True)
        cp.AddPath(toolInPosGeometry,pyclipper.PT_CLIP, True)
        cleared = cp.Execute(pyclipper.CT_UNION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        cleared = pyclipper.CleanPolygons(cleared)
        return cleared
    except:
        print toolInPosGeometry
        raise

output_point_count=0
def Execute(op,obj,feat_num,feat, scale_factor):
    global toolGeometry
    global enlargedToolGeometry
    global optimalCutAreaPerDist
    global iteration_limit_count
    global total_point_count
    global total_iteration_count
    global topZ
    global next_engagement_point_angle
    global tool_enlarge_scaled
    global smallDotGeometry
    global mediumDotGeometry
    global toolRadiusScaled
    global output_point_count
    global cache_hit_count
    global cache_pot_count
    global cache
    global cp
    global of

    import Interpolation
    #reload(Interpolation)
    import EngagementPoint
    #reload(EngagementPoint)

    toolDiaScaled=op.tool.Diameter*scale_factor
    toolRadiusScaled = toolDiaScaled / 2
    perf_start_time = time.time()
    perf_total_len = 0.0
    print "toolRadiusScaled: ", toolRadiusScaled
    helixDiameter = min(op.tool.Diameter, 1000.0 if obj.HelixDiameterLimit.Value==0.0 else obj.HelixDiameterLimit.Value )
    print "Helix diameter: ",helixDiameter
    helixDiameterScaled = helixDiameter*scale_factor
    stepOver=0.01*obj.StepOver # percentage to factor
    stepOverDistanceScaled = toolDiaScaled * stepOver
    finishPassOffset=1.0*obj.Tolerance
    next_engagement_point_angle=0
    cache_hit_count = 0
    cache_pot_count = 0
    cache={}
    output_point_count=0
    topZ = op.stock.Shape.BoundBox.ZMax

    #initialization
    of=pyclipper.PyclipperOffset()
    cp=pyclipper.Pyclipper()

    toleranceScaled = int(obj.Tolerance*scale_factor)
    tool_enlarge_scaled = 0 # toleranceScaled/8 +2

    stepScaled = 10

    toolGeometry = genToolGeomery(toolRadiusScaled+1)
    smallDotGeometry = genToolGeomery(0.2*scale_factor)
    mediumDotGeometry  = genToolGeomery(1.0*scale_factor)



    # intitalization = calculate slot cutting area per step i.e. 100% step over
    sceneInit(op.tool.Diameter/2.0)

    distScaled=toolRadiusScaled/2
    slotStep = translatePath(toolGeometry, [0,distScaled])
    cp.Clear()
    cp.AddPath(toolGeometry,pyclipper.PT_SUBJECT, True)
    cp.AddPath(slotStep,pyclipper.PT_CLIP, True)
    crossing=cp.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
    referenceCutArea = pyclipper.Area(crossing[0])
    fullSlotAreaPerDist = referenceCutArea/distScaled
    bound_extend =int(toolDiaScaled)

    #optimalCutAreaPerDistance (in scaled units)
    optimalCutAreaPerDist=stepOver*fullSlotAreaPerDist
    print "Optimal APD: ", optimalCutAreaPerDist, " Clipper scale factor: ", scale_factor,  " Tolerance: ", obj.Tolerance, " Tolerance scaled: ", toleranceScaled
    try:
        # find cut region toolpath polygons
        of.Clear()
        of.AddPaths(feat, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
        cut_region_tp=of.Execute(-int(finishPassOffset*scale_factor) - toolRadiusScaled)
        cut_region_tp = pyclipper.CleanPolygons(cut_region_tp)

        sceneDrawPaths("BOUNDARY",cut_region_tp,scale_factor,(1,0,0), True)

        cleared,startPoint=findStartPointRampEntry(op,feat_num, cut_region_tp, helixDiameterScaled/2, toolRadiusScaled,scale_factor)
        if cleared == None: return [], [0, 0]
        cleared = pyclipper.CleanPolygons(cleared)
        sceneDrawPath("START_POINT",translatePath(mediumDotGeometry, startPoint), scale_factor, (0,0,0), True)
        toolPos=startPoint

        of.Clear()
        of.AddPath([startPoint, startPoint],pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
        helixTp=of.Execute(helixDiameterScaled/2)
        sceneDrawPaths("TOOLPATH", helixTp,scale_factor,(0,0,1), True)

        EngagementPoint.Initialize(cut_region_tp, toolRadiusScaled, stepScaled, scale_factor)
        total_point_count=1
        total_iteration_count=0
        iteration_limit_count=0
        toolPaths=[]
        no_cut_count=0
        over_cut_count=0
        toolPos=[startPoint[0] , startPoint[1]- helixDiameterScaled/2]
        #toolDir = rotate([1.0,0.0], - INITIAL_ENGAGE_ANGLE)
        toolDir = [1.0,0.0]
        firstEngagePoint=True
        last_tool_gui_update = 0

        pas=0
        while True:
            pas=pas+1
            #print "pass:", pas
            if obj.StopProcessing:
                break
            #print ("finding next pass engange point... pas:", pas)
            #Console.PrintMessage("Pass: %d\n"%pas)

            if toolPos == None:
                Console.PrintMessage("next engage point not found\n")
                break #nothing more to cut

            passToolPath = []
            toClearPath = []
            cumulativeCuttingArea=0
            no_cut_count = 0
            reachedBoundaryPoint = None
            isOutside = False

            gyro = [toolDir]* 5
            toolDir = normalize(sumv(gyro))

            if SHOW_MARKERS:
                sceneClearPaths("ENGAGE")
                showToolDir("ENGAGE", toolPos, toolDir, scale_factor, (0, 1,0))


            engagePoint = toolPos
            #iteration through the points on path/pass
            i = 0
            deflectionAngle = math.pi  #init
            del deflectionAngleHistory[:]
            distToBoundary=0
            while True:  #points
                i = i + 1

                sceneUpdateGui()

                if obj.StopProcessing:
                    break
                #Console.PrintMessage("findNextPoint: %d =================================================================\n"%i)
                total_point_count=total_point_count+1
                ### IMPORTANT PART - finds the cut angle (new position stepScaled away from current with optimal cut area)
                toolDir = normalize(sumv(gyro))

                #distance to the boundary line
                #if distToBoundary<toolRadiusScaled:
                clp, distToBoundary = getClosestPointOnPaths(cut_region_tp, toolPos)

                distToEngagePoint=magnitude(sub2v(toolPos,engagePoint))

                relDistToBoundary = 2.0*distToBoundary / toolRadiusScaled
                minCutAreaPerDist = optimalCutAreaPerDist/3+1
                #if we are away from end boundary line, try makeing the optimal cut
                if relDistToBoundary > 1 or distToEngagePoint<toolRadiusScaled:
                    targetArea = optimalCutAreaPerDist
                else: #decrease the cut area if we are close to boundary, adds a little bit of smoothing to the end of cut
                    targetArea = relDistToBoundary * \
                        (optimalCutAreaPerDist-minCutAreaPerDist) + minCutAreaPerDist

                if distToBoundary < toolRadiusScaled or distToEngagePoint < toolRadiusScaled:
                    stepScaled = toleranceScaled*2
                elif math.fabs(deflectionAngle)>0.0001:
                    stepScaled = 4.0/deflectionAngle
                else:
                    stepScaled = toleranceScaled*4

                if stepScaled < toleranceScaled*2:
                    stepScaled = toleranceScaled*2
                if stepScaled > toolRadiusScaled/2:
                    stepScaled = toolRadiusScaled/2

                #
                # Find next point with optimal cut
                #

                bound_box = [[toolPos[0] - bound_extend, toolPos[1] - bound_extend], [toolPos[0] + bound_extend, toolPos[1] - bound_extend], [toolPos[0] + bound_extend, toolPos[1] + bound_extend], [toolPos[0] - bound_extend, toolPos[1] + bound_extend]]
                cp.Clear()
                cp.AddPath(bound_box, pyclipper.PT_SUBJECT, True)
                cp.AddPaths(cleared,pyclipper.PT_CLIP, True)
                cleared_bounded = cp.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
                if not anyValidPath(cleared_bounded): cleared_bounded = cleared

                newToolPos, newToolDir, cuttingAreaPerDist, cuttingArea, deflectionAngle = findNextPoint(
                    obj, op, of, cp, cleared_bounded, toolPos, toolDir, toolRadiusScaled, stepScaled, targetArea, scale_factor)
                #
                # CHECK if we reached the the boundary
                #
                if distToBoundary<toolRadiusScaled and isOutsideCutRegion(newToolPos, cut_region_tp, True):
                    isOutside=True
                    reachedBoundaryPoint = getIntersectionPointLWP([toolPos, newToolPos], cut_region_tp)

                    if reachedBoundaryPoint != None:
                        #print "reachedBoundaryPoint:", reachedBoundaryPoint
                        #showTool("TL", reachedBoundaryPoint, scale_factor, (1, 0, 0))
                        newToolPos = reachedBoundaryPoint
                    else:
                        newToolPos=toolPos

                    cuttingArea, cuttingAreaPerDist = calcCutingArea(
                        toolPos, newToolPos, toolRadiusScaled, cleared_bounded)


                if cuttingArea>3*cuttingAreaPerDist+10 and cuttingAreaPerDist>2*optimalCutAreaPerDist+10:
                    over_cut_count=over_cut_count+1
                    Console.PrintMessage("Break: over cut (%d %f/%f)\n" % (over_cut_count, cuttingAreaPerDist, optimalCutAreaPerDist))
                    break
                else:
                    over_cut_count = 0

                if len(toClearPath) == 0: toClearPath.append(toolPos)
                toClearPath.append(newToolPos)
                if firstEngagePoint:
                    if len(toClearPath) > 10:
                        of.Clear()
                        of.AddPath(toClearPath, pyclipper.JT_ROUND,
                                pyclipper.ET_OPENROUND)
                        toolCoverArea = of.Execute(toolRadiusScaled+1)[0]
                        cleared = expandClearedArea(cp, toolCoverArea, cleared)
                        toClearPath=[]

                if cuttingArea>0:
                    # cut is OK, record it
                    cumulativeCuttingArea=cumulativeCuttingArea+cuttingArea
                    if time.time() - last_tool_gui_update > GUI_UPDATE_PERIOD:
                        if SHOW_MARKERS:
                            sceneClearPaths("TP")
                            showFillTool("TP",newToolPos,scale_factor,(0,0,1))
                            showToolDir("TP", newToolPos, newToolDir, scale_factor, (0, 0, 1))

                        sceneClearPaths("PTP")
                        sceneDrawPath("PTP", passToolPath, scale_factor, (0, 0, 1))
                        # sceneClearPaths("CL_BOUNDED")
                        # sceneDrawPaths("CL_BOUNDED", cleared_bounded,scale_factor,(1,1,0),True)
                        last_tool_gui_update=time.time()
                    #append next point to toolpath

                    perf_total_len=perf_total_len+stepScaled
                    if i == 0: passToolPath.append(toolPos) #append first point to toolpath
                    passToolPath.append(newToolPos)
                    toolPos = newToolPos
                    gyro.append(newToolDir)
                    gyro.pop(0)
                    no_cut_count = 0
                else:  #no cut
                    #print "no cut:", no_cut_count
                    no_cut_count = no_cut_count + 1
                    newToolDir = toolDir
                    break
                    if no_cut_count > 1:
                        print "break: no cut"
                        break

                if isOutside:  # next valid cut not found
                   #print "Breaking: reached boundary"
                    break
                #distToBoundary = distToBoundary-stepScaled
                #END OF POINTS INTERATION

            #expand cleared area
            if len(toClearPath) > 0:
                of.Clear()
                of.AddPath(toClearPath, pyclipper.JT_ROUND,
                        pyclipper.ET_OPENROUND)
                toolCoverArea = of.Execute(toolRadiusScaled + 1)[0]
                cleared = expandClearedArea(cp, toolCoverArea, cleared)
                toClearPath=[]
            if cumulativeCuttingArea >  stepScaled * stepOver * referenceCutArea / 20: #did we cut something significant?
                sceneClearPaths("PTP")
                sceneDrawPath("TOOLPATH", passToolPath, scale_factor)
                appendToolPathCheckJump(of, cp, toolPaths, passToolPath, toolRadiusScaled, cleared)

            if over_cut_count>5:
                Console.PrintError("Break: WARNING: resulting toolpath may be incomplete! (Hint: try changing numeric precision or step over)\n")
                break
            #FIND NEXT ENGAGING POINT
            if firstEngagePoint:
                EngagementPoint.moveToClosestPoint(newToolPos)
                firstEngagePoint = False
            else:
                moveDistance = stepOverDistanceScaled/4 + 1
                if not EngagementPoint.moveForwardToCutThreshold(cleared, moveDistance, moveDistance*2, 1.5*optimalCutAreaPerDist * moveDistance):
                    break

            #clear around the engagement point
            toolPos, toolDir = EngagementPoint.getCurrentPoint()

        performance = 1.0 * perf_total_len/scale_factor/(time.time()-perf_start_time) # mm/s
        Console.PrintMessage("Passes: %s, Toolpaths: %d, Output Points: %d, Processed Points: %d, Avg.Iterations: %f, Exceeded: %d, Perf.: %f mm/s, cache_hit: %d/%d (%f perc.)\n" % (
            pas, len(toolPaths),output_point_count, total_point_count, 1.0*total_iteration_count/total_point_count, iteration_limit_count, performance, cache_hit_count, cache_pot_count, 100*cache_hit_count/(cache_pot_count+0.1)))

        #generate finishing pass
        sceneUpdateGui()
        passToolPath=[]
        of.Clear()
        of.AddPaths(feat, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
        finishing=of.Execute(-toolRadiusScaled)
        for passToolPath in finishing:
            appendToolPathCheckJump(of,cp,toolPaths, passToolPath, toolRadiusScaled, cleared, True)
    except:
        sceneClean()
        raise

    sceneClean()
    return toolPaths, startPoint

