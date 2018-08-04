import Path
import math

def Execute(op,obj,adaptiveToolpath, startPoint, scale_factor):
    if startPoint == None: return
    if adaptiveToolpath==None: return
    if len(adaptiveToolpath) == 0: return
    if len(adaptiveToolpath[0]) == 0: return

    adaptiveStartPoint = [1.0*adaptiveToolpath[0][0][0]/scale_factor, 1.0*adaptiveToolpath[0][0][1]/scale_factor]

    helixDiameter = min(op.tool.Diameter,1000.0 if obj.HelixDiameterLimit.Value==0.0 else obj.HelixDiameterLimit.Value )

    minLiftDistance = op.tool.Diameter

    helixRadius = helixDiameter/2.0
    startPoint = [1.0*startPoint[0]/scale_factor, 1.0*startPoint[1]/scale_factor]
    startAngle = math.atan2(adaptiveStartPoint[1] - startPoint[1], adaptiveStartPoint[0] - startPoint[0])
    #print startPoint, adaptiveStartPoint,startAngle
    lx=startPoint[0]
    ly=startPoint[1]

    stepDown = obj.StepDown.Value
    if stepDown<0.1 : stepDown=0.1
    #print ("StepDown:", stepDown)

    passStartDepth=obj.StartDepth.Value
    length = 2*math.pi * helixRadius

    if obj.HelixAngle<1: obj.HelixAngle=1

    helixAngleRad = math.pi * obj.HelixAngle/180.0
    depthPerOneCircle=length * math.tan(helixAngleRad)
    #print "Helix step down per full circle:" , depthPerOneCircle, " (len:" , length , ")"
    stepUp =  obj.LiftDistance.Value
    if stepUp<0:
        stepUp=0

    step=0
    while passStartDepth>obj.FinalDepth.Value and step<1000:
        step=step+1

        passEndDepth=passStartDepth-stepDown
        if passEndDepth<obj.FinalDepth.Value: passEndDepth=obj.FinalDepth.Value

        r = helixRadius - 0.01
        #spiral ramp
        passDepth = (passStartDepth - passEndDepth)
        maxfi =  passDepth / depthPerOneCircle *  2 * math.pi   #- math.pi/8
        fi = 0
        offsetFi =-maxfi + startAngle-math.pi/16


        helixStart = [startPoint[0] + r * math.cos(offsetFi), startPoint[1] + r * math.sin(offsetFi)]
        #rapid move to start point
        op.commandlist.append(Path.Command(
            "G0", {"X": helixStart[0], "Y": helixStart[1], "Z": obj.ClearanceHeight.Value}))
        #rapid move to safe height
        op.commandlist.append(Path.Command(
            "G0", {"X": helixStart[0], "Y": helixStart[1], "Z": obj.SafeHeight.Value}))
        #move to safe height
        op.commandlist.append(Path.Command("G1", {
                              "X": helixStart[0], "Y": helixStart[1], "Z": passStartDepth, "F": op.vertFeed}))


        while fi<maxfi:
            x = startPoint[0] + r * math.cos(fi+offsetFi)
            y = startPoint[1] + r * math.sin(fi+offsetFi)
            z = passStartDepth - fi / maxfi * (passStartDepth - passEndDepth)
            op.commandlist.append(Path.Command("G1", { "X": x, "Y":y, "Z":z, "F": op.vertFeed}))
            fi=fi+math.pi/16
        lx=0
        ly=0
        #add adaptive paths
        for pi in range(0, len(adaptiveToolpath)):
            toolPath = adaptiveToolpath[pi]
            for pt in toolPath:
                x=1.0*pt[0]/scale_factor
                y = 1.0*pt[1]/scale_factor
                dist=math.sqrt((x-lx)*(x-lx) + (y-ly)*(y-ly))
                if len(pt)==3: #positioning move
                    jumpType = pt[2]
                    if jumpType==2: #clearance
                        if lx!=x or ly!=y:
                            op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.ClearanceHeight.Value}))
                        op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":obj.ClearanceHeight.Value}))
                    elif jumpType == 1:  # lift
                        if dist > minLiftDistance:
                            if lx!=x or ly!=y:
                                op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":passEndDepth+stepUp}))
                            op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":passEndDepth+stepUp}))
                    else:
                        if lx!=x or ly!=y:
                            op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.ClearanceHeight.Value}))
                        op.commandlist.append(Path.Command("G0", { "X": x, "Y":y, "Z":passEndDepth}))
                else: #engaging move
                    op.commandlist.append(Path.Command("G1", { "X": x, "Y":y, "Z":passEndDepth, "F": op.horizFeed}))
                lx=x
                ly=y
        passStartDepth=passEndDepth
    #return to safe height
    op.commandlist.append(Path.Command("G0", { "X": lx, "Y":ly, "Z":obj.SafeHeight.Value}))


