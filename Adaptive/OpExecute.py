import FreeCAD
import FreeCADGui
import Path
import Part
import PathScripts.PathGeom as PathGeom
import pyclipper
import Adaptive.ProcessFeature
import Adaptive.GenerateGCode
import time
from FreeCAD import Console
import json
import math
import cProfile

PROFILE = False

def _discretize(edge, flipDirection=False):
    pts=edge.discretize(Deflection=0.01)
    if flipDirection: pts.reverse()
    return pts

def IsEqualInXYPlane(e1, e2):
    return math.sqrt((e2.x-e1.x)*(e2.x-e1.x) +
              (e2.y - e1.y) * (e2.y - e1.y))<0.01


def _connectEdges(edges):
    ''' Makes the list of connected discretized paths '''
    # find edge
    lastPoint=None
    remaining = []

    pathArray = []
    combined = []
    #print "Input edges , remove duplicate projections to xy plane"
    for edge in edges:
        p1 = edge.valueAt(edge.FirstParameter)
        p2 = edge.valueAt(edge.LastParameter)
        duplicate = False
        for ex in remaining:
            exp1 = ex.valueAt(ex.FirstParameter)
            exp2 = ex.valueAt(ex.LastParameter)
            if IsEqualInXYPlane(exp1, p1) and IsEqualInXYPlane(exp2, p2):
                duplicate = True
            if IsEqualInXYPlane(exp1, p2) and IsEqualInXYPlane(exp2, p1):
                duplicate = True
        if not duplicate:
            remaining.append(edge)
    #print "remaining:", remaining

    newPath=True
    while len(remaining)>0:
        if newPath:
            #print "new iteration"
            edge=remaining[0]
            p1 = edge.valueAt(edge.FirstParameter)
            p2 = edge.valueAt(edge.LastParameter)
            #print edge, p1, p2
            if len(combined)>0: pathArray.append(combined)
            combined = []
            combined.append(_discretize(edge))
            remaining.remove(edge)
            lastPoint=p2
            newPath=False

        anyMatch=False
        for e in remaining:
            p1 = e.valueAt(e.FirstParameter)
            p2 = e.valueAt(e.LastParameter)
            #print "chk",e, p1, p2
            if IsEqualInXYPlane(lastPoint,p1):
                #print "last Point equal p1"
                combined.append(_discretize(e))
                remaining.remove(e)
                lastPoint=p2
                anyMatch=True
                break
            elif IsEqualInXYPlane(lastPoint,p2):
                #print "reversed"
                combined.append(_discretize(e,True))
                remaining.remove(e)
                lastPoint=p1
                anyMatch=True
                break
        if not anyMatch:
            newPath=True


    #make sure last path  is appended
    if len(combined)>0: pathArray.append(combined)
    combined = []
    return pathArray

def convertToClipper(pathArray,SCALE_FACTOR):
    clipperPaths=[]
    for path in pathArray:
        points=[]
        for edge in path:
             for pts in edge: #points
                points.append([pts.x,pts.y])
        if len(points)>0:
            scaled=pyclipper.scale_to_clipper(points, SCALE_FACTOR)
            clipperPaths.append(scaled)

    return pyclipper.SimplifyPolygons(clipperPaths)

def resolveTree(clipperPaths, processHoles,side):
    pairGraph=[]
    for ip1 in range(0, len(clipperPaths)):
        p1=clipperPaths[ip1]
        for ip2 in range(ip1+1,len(clipperPaths)):
            p2=clipperPaths[ip2]
            if pyclipper.PointInPolygon(p2[0],p1)!=0:
                pairGraph.append([ip1,ip2]) #ip1 parent,ip2 child
            elif pyclipper.PointInPolygon(p1[0],p2)!=0:
                pairGraph.append([ip2,ip1]) #ip2 parent,ip1 child
    #calc nesting levels
    nesting = [0]*len(clipperPaths)
    for ip1 in range(0, len(clipperPaths)): #for each path find the nesting level
        for pp in pairGraph:
            if pp[1] == ip1:
                nesting[ip1] = nesting[ip1]+1

    print "Nesting info: " , nesting
    #make feature paths first path is outer others are inner (direct holes)
    features = []

    for ip in range(0, len(clipperPaths)):
        if side=="Outside" and not processHoles:
            if nesting[ip] ==0: #add all paths with odd nesting as outside paths, odd will be added as holes
                features.append([ip])
        else:
            if nesting[ip] % 2 ==0: #add all paths with odd nesting as outside paths, odd will be added as holes
                features.append([ip])

    if processHoles or side=="Outside":
        for pair in pairGraph:
            ip1=pair[0]
            ip2=pair[1]
            for feat in features:
                if feat[0] == ip1 and  nesting[ip2] % 2 == 1 :
                    feat.append(ip2)
    #print(features)

    tree = []
    for feat in features:
        paths=[]
        for ip in feat:
            paths.append(clipperPaths[ip])
        tree.append(paths)

    return tree

def OpExecute(op,obj):
    Console.PrintMessage("*** Adaptive toolpath processing started...\n")
    #reload(Adaptive.ProcessFeature)
    #reload(Adaptive.GenerateGCode)

    #obj.ViewObject.Visibility = False  # hide toolpaths during calculation
    #hide old during recalculation
    obj.Path = Path.Path("(calculating...)")
    #store old visibility state
    job = op.getJob(obj)
    oldObjVisibility = obj.ViewObject.Visibility
    oldJobVisibility = job.ViewObject.Visibility

    obj.ViewObject.Visibility = False
    job.ViewObject.Visibility = False

    FreeCADGui.updateGui()
    try:
        Console.PrintMessage("Tool diam: %f \n"%op.tool.Diameter)

        obj.Stopped = False
        obj.StopProcessing = False
        if obj.Tolerance<0.001: obj.Tolerance=0.001

        SCALE_FACTOR=int(8.0/obj.Tolerance+0.5)
        #if SCALE_FACTOR<100: SCALE_FACTOR=100

        edges=[]
        for base, subs in obj.Base:
            #print (base,subs)
            for sub in subs:
                shape=base.Shape.getElement(sub)
                for edge in shape.Edges:
                    edges.append(edge)

        pathArray=_connectEdges(edges)
        #print "pathArray:",pathArray
        if obj.Side == "Outside":
            stockBB = op.stock.Shape.BoundBox
            v=[]
            v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
            v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMin,0))
            v.append(FreeCAD.Vector(stockBB.XMax,stockBB.YMax,0))
            v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMax,0))
            v.append(FreeCAD.Vector(stockBB.XMin,stockBB.YMin,0))
            pathArray.append([v])

        clipperPaths=convertToClipper(pathArray,SCALE_FACTOR)

        #print clipperPaths

        features=resolveTree(clipperPaths,obj.ProcessHoles,obj.Side)

        # put here all properties that influence calculation of adaptive base paths,
        inputStateObject = {
            "tool": op.tool.Diameter,
            "tolerance": obj.Tolerance,
            "geometry" : features,
            "stepover" :obj.StepOver,
            "effectiveHelixDiameter": min(op.tool.Diameter,1000.0 if obj.HelixDiameterLimit.Value==0.0 else obj.HelixDiameterLimit.Value )
        }

        inputStateStr= json.dumps(inputStateObject)

        inputStateChanged=False


        if obj.AdaptiveOutputState !=None and obj.AdaptiveOutputState != "":
            outputStateObj = json.loads(obj.AdaptiveOutputState)
            hasOutputState=True
        else:
            hasOutputState=False
            outputStateObj=[]

        if obj.AdaptiveInputState != inputStateStr:
            inputStateChanged=True
            hasOutputState=False
            outputStateObj=[]

        start=time.time()
        for i in range(0 , len(features)):
            feat = features[i]
            if inputStateChanged or not hasOutputState:
                if PROFILE:
                    profiler = cProfile.Profile()
                    baseToolPaths, startPoint= profiler.runcall(Adaptive.ProcessFeature.Execute,
                                     op, obj, i, feat, SCALE_FACTOR)
                    profiler.print_stats(sort='cumtime')
                else:
                    baseToolPaths,startPoint=Adaptive.ProcessFeature.Execute(op,obj,i,feat, SCALE_FACTOR)
                outputState = {
                    "baseToolPaths": baseToolPaths,
                    "startPoint" : startPoint
                }
                outputStateObj.append(outputState)
            else:
                Console.PrintMessage("State not changed, using cached base paths\n")
            Adaptive.GenerateGCode.Execute(op, obj, outputStateObj[i]["baseToolPaths"], outputStateObj[i]["startPoint"], SCALE_FACTOR)


        if not obj.StopProcessing:
            Console.PrintMessage("*** Done. Elapsed: %f sec\n\n" %(time.time()-start))
            obj.AdaptiveOutputState = json.dumps(outputStateObj)
            obj.AdaptiveInputState=inputStateStr
        else:
            Console.PrintMessage("*** Processing cancelled (after: %f sec).\n\n" %(time.time()-start))
    finally:
        obj.ViewObject.Visibility = oldObjVisibility
        job.ViewObject.Visibility = oldJobVisibility
