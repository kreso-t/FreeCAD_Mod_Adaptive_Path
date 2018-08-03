import FreeCAD
import FreeCADGui
import AdaptivePath
import AdaptivePathGui
import Adaptive_rc
print("Loading adaptive PATH")

def SetupOperation(name,
                   resName,
                   objFactory,
                   opPageClass,
                   pixmap,
                   menuText,
                   toolTip,
                   accelKey=None):
    '''SetupOperation(name, objFactory, opPageClass, pixmap, menuText, toolTip, accelKey=None)
    Creates an instance of CommandPathOp with the given parameters and registers the command with FreeCAD.
    When activated it creates a model with proxy (by invoking objFactory), assigns a view provider to it
    (see ViewProvider in this module) and starts the editor specifically for this operation (driven by opPageClass).
    This is an internal function that is automatically called by the initialisation code for each operation.
    It is not expected to be called manually.
    '''

    import Adaptive_rc
    import PathScripts.PathOpGui as PathOpGui
    res = PathOpGui.CommandResources(resName, objFactory, opPageClass, pixmap, menuText, accelKey, toolTip)

    command = PathOpGui.CommandPathOp(res)
    FreeCADGui.addCommand("Path_%s" % name.replace(' ', '_'), command)
    return command


Command = SetupOperation(
        'Adaptive 2.5D',
        'Pocket Shape',
        AdaptivePath.Create,
        AdaptivePathGui.TaskPanelOpPage,
        ':/Path-Adaptive.svg',
        "Adaptive 2.5D",
        "Creates a adaptive path from a face or faces")
