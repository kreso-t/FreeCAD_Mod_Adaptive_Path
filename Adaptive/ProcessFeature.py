import pyclipper
import Path
import math
import Plot
from FreeCAD import Console
import time
import Interpolation
import random
import EngagementPoint
from GuiUtils import *
from GeomUtils import *

#contants
RELOAD_MODULES = True

#globals
total_iteration_count=0
output_point_count = 0
last_gui_update_time = time.time()
cache = {}
cache_hit_count = 0
cache_pot_count = 0
iteration_limit_count = 0
total_point_count = 0
scale_factor = 0

###########################################################
# Checks if tool position if within allowed area
############################################################

def isOutsideCutRegion(toolPos, cut_region_tp_polytree, include_boundary=False):
    #check if toolPos is outside cut area - i.e. used to check if we reached end of this continuous pass
    for node in cut_region_tp_polytree.Childs:
        pip = pyclipper.PointInPolygon(toolPos, node.Contour)
        if pip == -1 and include_boundary:
            return True
        if pip == 1:  #if inside the root path, check if not inside any of its holes
            for hole in node.Childs:
                pip = pyclipper.PointInPolygon(toolPos, hole.Contour)
                if pip == -1 and include_boundary:
                    return True
                if pip == 1: return True  # must not be inside all other (holes)
            return False
    return True


def findStartPoint(op,feat_num, cut_region_tp,cut_region_tp_polytree, helixRadius, toolRadius,scale_factor):
    #searching for biggest area to cut, by decremental offseting from target cut_region

    #start offset is max x or y size of the stock/2
    maxLen = max(op.stock.Shape.BoundBox.XLength,op.stock.Shape.BoundBox.YLength)
    starting_offset= maxLen*scale_factor/2

    #try with some reasonable step
    step = toolRadius/4

    while starting_offset >= helixRadius:
        of.Clear()
        of.AddPaths(cut_region_tp, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
        offsetPaths=of.Execute(-(starting_offset))
        #showPath(op,offsetPaths,scale_factor)
        for path in offsetPaths:
            #find center
            pt = centroid(path)
            #showTool("STP", pt, scale_factor, (1,0,1))
            if not isOutsideCutRegion(pt, cut_region_tp_polytree):
                    of.Clear()
                    of.AddPath([pt,[pt[0]+1,pt[1]]], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
                    cleared_helix=of.Execute(helixRadius + toolRadius)
                    Console.PrintMessage("Start point: %f,%f\n"%(1.0*pt[0]/scale_factor,1.0*pt[1]/scale_factor))
                    return cleared_helix,pt
            # sceneClearPaths("STP")
            # sceneDrawPath("STP", path, scale_factor, (1, 0, 1))
            # messageBox("Continue")

        starting_offset=starting_offset-step

    Console.PrintError("Unable to find starting point (for path no:%d)!\n"%feat_num)
    #sceneClearPaths("STP")
    return None,None


# caching the cutting shapes as they tend to repeat a lot (saves a lot of clipping)
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


def appendToolPathCheckCollision(of,cp,toolPaths,passToolPath,toolRadiusScaled,cleared,close=False):
    #appending pass toolpath to the list of cuts
    #checking the jump line for obstacles
    global output_point_count

    if len(passToolPath)<1: return
    jumpType=1 # assume no lift
    nextPoint = passToolPath[0]
    if len(toolPaths)!=0 and  len(toolPaths[-1])>0: #if this is first path: lift to safe height
        lastPath = toolPaths[-1]
        lastPoint = lastPath[-1]
        #make tool shape from last point to next point
        of=pyclipper.PyclipperOffset()
        of.AddPath([lastPoint,nextPoint], pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
        toolShape=of.Execute(toolRadiusScaled-2)
        #check clearance
        cp.Clear()
        cp.AddPaths(toolShape,pyclipper.PT_SUBJECT, True)
        cp.AddPaths(cleared,pyclipper.PT_CLIP, True)
        crossing=cp.Execute(pyclipper.CT_DIFFERENCE, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
        collisionArea = 0
        for path in crossing:
            collisionArea = collisionArea + math.fabs(pyclipper.Area(path))
        if collisionArea>0:
           jumpType=2
        toolPaths.append([[lastPoint[0], lastPoint[1], jumpType], [nextPoint[0], nextPoint[1], jumpType]])

    #passToolPath = pyclipper.CleanPolygon(passToolPath,0.7) # this messes up the start point (adds a point to the start of path)
    output_point_count=output_point_count + len(passToolPath)
    if len(passToolPath)>0:
        if close:
            passToolPath = closePath(passToolPath)
        toolPaths.append(passToolPath)

#######################################################################
# Finds next optimal cutting point
#######################################################################
deflectionAngleHistory=[] #used for better guessing next initial angle
def findNextPoint(obj, op, of, cp, cleared, toolPos, toolDir, toolRadiusScaled, stepScaled,targetAreaPerDist, scale_factor):
    global toolGeometry
    global iteration_limit_count
    global total_point_count
    global total_iteration_count
        #find valid next tool pos in the pass
    cuttingAreaPerDist=0
    cuttingArea=0
    iteration=0
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


def Execute(op,obj,feat_num,feat, p_scale_factor):
    global toolGeometry
    global optimalCutAreaPerDist
    global iteration_limit_count
    global total_point_count
    global total_iteration_count
    global topZ
    global toolRadiusScaled
    global output_point_count
    global cache_hit_count
    global cache_pot_count
    global cache
    global cp
    global of
    global scale_factor

    scale_factor = p_scale_factor

    if RELOAD_MODULES:
        reload(Interpolation)
        reload(EngagementPoint)

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
    finishPassOffset=1.0*obj.Tolerance/2
    cache_hit_count = 0
    cache_pot_count = 0
    cache={}
    output_point_count=0
    topZ = op.stock.Shape.BoundBox.ZMax

    #initialization
    of=pyclipper.PyclipperOffset()
    cp=pyclipper.Pyclipper()

    toleranceScaled = int(obj.Tolerance*scale_factor)

    stepScaled = 10

    toolGeometry = genToolGeometry(toolRadiusScaled+1)

    # intitalization = calculate slot cutting area per step i.e. 100% step over
    sceneInit(toolRadiusScaled, topZ, scale_factor)

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
        cut_region_tp = of.Execute(-int(finishPassOffset * scale_factor) - toolRadiusScaled)
        cut_region_tp_polytree =  of.Execute2(-int(finishPassOffset * scale_factor) - toolRadiusScaled)
        cut_region_tp = pyclipper.CleanPolygons(cut_region_tp)

        sceneDrawPaths("BOUNDARY",cut_region_tp,scale_factor,(1,0,0), True)

        cleared, startPoint = findStartPoint(
            op, feat_num, cut_region_tp, cut_region_tp_polytree, helixDiameterScaled / 2, toolRadiusScaled, scale_factor)

        if cleared == None: return [], [0, 0]
        cleared = pyclipper.CleanPolygons(cleared)
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
                sceneDrawToolDir("ENGAGE", toolPos, toolDir, scale_factor, (0, 1,0))


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
                toolDir = normalize(sumv(gyro))

                #distance to the boundary line
                #if distToBoundary<toolRadiusScaled:
                clp, distToBoundary = getClosestPointOnPaths(cut_region_tp, toolPos)

                distToEngagePoint=magnitude(sub2v(toolPos,engagePoint))

                relDistToBoundary = 2.0*distToBoundary / toolRadiusScaled
                minCutAreaPerDist = optimalCutAreaPerDist/3+1
                #if we are away from end boundary line, try making the optimal cut
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
                if distToBoundary<toolRadiusScaled and isOutsideCutRegion(newToolPos, cut_region_tp_polytree, True):
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
                            sceneDrawFilledTool("TP",newToolPos,scale_factor,(0,0,1))
                            sceneDrawToolDir("TP", newToolPos, newToolDir, scale_factor, (0, 0, 1))

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
                    # if no_cut_count > 1:
                    #     print "break: no cut"
                    #     break

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

            if cumulativeCuttingArea >  stepScaled * stepOver * referenceCutArea / 40: #did we cut something significant?
                sceneClearPaths("PTP")
                sceneDrawPath("TOOLPATH", passToolPath, scale_factor)
                appendToolPathCheckCollision(of, cp, toolPaths, passToolPath, toolRadiusScaled, cleared)

            if over_cut_count>5:
                Console.PrintError("Break: WARNING: resulting toolpath may be incomplete! (Hint: try changing numeric precision or StepOver)\n")
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
        Console.PrintMessage("Passes: %s, Toolpaths: %d, Output Points: %d, Processed Points: %d, Avg.Iterations: %f, Exceeded: %d, Perf.: %f mm/s, Cut shape cache hit: %d/%d (%f perc.)\n" % (
            pas, len(toolPaths),output_point_count, total_point_count, 1.0*total_iteration_count/total_point_count, iteration_limit_count, performance, cache_hit_count, cache_pot_count, 100*cache_hit_count/(cache_pot_count+0.1)))

        #generate finishing pass
        sceneUpdateGui()
        passToolPath=[]
        of.Clear()
        of.AddPaths(feat, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
        finishing = of.Execute2(-toolRadiusScaled)
        # add only paths containing the startPoint and their childs
        for child in finishing.Childs:
            if pyclipper.PointInPolygon(startPoint, child.Contour) != 0:
                appendToolPathCheckCollision(of, cp, toolPaths, child.Contour, toolRadiusScaled, cleared, True)
                for hole in child.Childs:
                    appendToolPathCheckCollision(of, cp, toolPaths, hole.Contour, toolRadiusScaled, cleared, True)

    except:
        sceneClean()
        raise

    sceneClean()
    return toolPaths, startPoint

