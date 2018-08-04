from PySide import QtCore, QtGui
from pivy import coin
import FreeCADGui

def messageBox(msg):
    # Create a simple dialog QMessageBox
    # The first argument indicates the icon used: one of QtGui.QMessageBox.{NoIcon, Information, Warning, Critical, Question}
    diag = QtGui.QMessageBox(QtGui.QMessageBox.Warning, 'Info', msg)
    diag.setWindowModality(QtCore.Qt.ApplicationModal)
    diag.exec_()
    FreeCADGui.updateGui()


def confirmMessage(msg):
    FreeCADGui.updateGui()
    reply = QtGui.QMessageBox.question(
        None, "", msg, QtGui.QMessageBox.Yes | QtGui.QMessageBox.No, QtGui.QMessageBox.No)
    if reply == QtGui.QMessageBox.Yes:
        return True
    return False


###################################################
# a progress marker thingy
###################################################
scenePathNodes = {}
def sceneInit(toolRadius):
    global sg
    sg = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()


def sceneDrawPaths(grpName, paths, scale_factor, color=(0, 0, 1), closed=False):
    for path in paths:
        sceneDrawPath(grpName, path, scale_factor, color, closed)


def sceneUpdateGui():
    global last_gui_update_time
    if time.time()-last_gui_update_time > GUI_UPDATE_PERIOD:
        FreeCADGui.updateGui()
        last_gui_update_time = time.time()


def sceneDrawPath(grpName, path, scale_factor, color=(0, 0, 1), closed=False):
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
        pts.append([1.0*pt[0]/scale_factor, 1.0*pt[1]/scale_factor, topZ])

    coPoint.point.setValues(0, len(pts), pts)
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


def sceneDrawFilledPaths(grpName, paths, scale_factor, color=(0, 0, 1)):
    global sg
    global topZ
    global scenePathNodes

    for i in range(0, len(paths)):
        path = paths[i]
        pts = []
        for pt in path:
            pts.append([1.0*pt[0]/scale_factor, 1.0*pt[1]/scale_factor, topZ])
        coPoint = coin.SoCoordinate3()
        coPoint.point.setValues(0, len(pts), pts)
        ma = coin.SoMaterial()
        ma.diffuseColor = color
        if i == 0:
            ma.transparency.setValue(0.9)
        else:
            ma.transparency.setValue(0.8)

        hints = coin.SoShapeHints()
        hints.faceType = coin.SoShapeHints.UNKNOWN_FACE_TYPE
        #hints.vertexOrdering = coin.SoShapeHints.CLOCKWISE

        li = coin.SoIndexedFaceSet()
        li.coordIndex.setValues(range(0, len(pts)))
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


def showToolDir(grpName, tp, td, scale_factor, color):
    toolInPos = translatePath(toolGeometry, tp)
    sceneDrawPath(grpName, toolInPos, scale_factor, color, True)

    pt2 = [tp[0] + td[0] * toolRadiusScaled *
           3, tp[1] + td[1] * toolRadiusScaled*3]
    toolInPos = translatePath(smallDotGeometry, pt2)
    sceneDrawPath(grpName, toolInPos, scale_factor, color, True)
    sceneDrawPath(grpName, [tp, pt2], scale_factor, color)


def showTool(grpName, tp, scale_factor, color):
    toolInPos = translatePath(toolGeometry, tp)
    sceneDrawPaths(grpName, [toolInPos], scale_factor, color, True)


def showFillTool(grpName, tp, scale_factor, color):
    toolInPos = translatePath(toolGeometry, tp)
    sceneDrawFilledPaths(grpName, [toolInPos], scale_factor, color)

