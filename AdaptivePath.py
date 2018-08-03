import PathScripts.PathOp as PathOp
import FreeCAD
import Adaptive.OpExecute

class AdaptivePathOp(PathOp.ObjectOp):
    def opFeatures(self, obj):
        '''opFeatures(obj) ... returns the OR'ed list of features used and supported by the operation.
        The default implementation returns "FeatureTool | FeatureDeptsh | FeatureHeights | FeatureStartPoint"
        Should be overwritten by subclasses.'''
        return PathOp.FeatureTool | PathOp.FeatureBaseEdges | PathOp.FeatureDepths | PathOp.FeatureStepDown | PathOp.FeatureHeights | PathOp.FeatureStartPoint | PathOp.FeatureBaseGeometry

    def initOperation(self, obj):
        '''initOperation(obj) ... implement to create additional properties.
        Should be overwritten by subclasses.'''
        print ("AdaptivePathOp initOperation")
        #obj.addProperty("App::PropertyLength", "Dirty", "Adaptive",'Test feature')
        obj.addProperty("App::PropertyEnumeration", "Side", "Adaptive", "Side of selected faces that tool should cut")
        obj.Side = ['Outside', 'Inside']  # side of profile that cutter is on in relation to direction of profile
        obj.addProperty("App::PropertyFloat", "Tolerance", "Adaptive",  "Clearing tolerance")
        obj.addProperty("App::PropertyPercent", "StepOver", "Adaptive", "Percent of cutter diameter to step over on each pass")
        obj.addProperty("App::PropertyDistance", "LiftDistance", "Adaptive", "Lift distance for rapid moves")
        obj.addProperty("App::PropertyBool", "ProcessHoles", "Adaptive","Process holes as well as the face outline")
        obj.addProperty("App::PropertyBool", "Stopped",
                        "Adaptive", "Stop processing")
        obj.setEditorMode('Stopped', 2) #hide this property

        obj.addProperty("App::PropertyBool", "StopProcessing",
                                  "Adaptive", "Stop processing")
        obj.setEditorMode('StopProcessing', 2)  # hide this property

        obj.addProperty("App::PropertyString", "AdaptiveInputState", "Internal","Iternal input state")
        obj.addProperty("App::PropertyString", "AdaptiveOutputState", "Internal","Iternal output state")
        obj.setEditorMode('AdaptiveInputState', 2) #hide this property
        obj.setEditorMode('AdaptiveOutputState', 2) #hide this property
        obj.addProperty("App::PropertyAngle", "HelixAngle", "Adaptive",  "Helix ramp entry angle (degrees)")
        obj.addProperty("App::PropertyLength", "HelixDiameterLimit", "Adaptive", "Limit helix entry diameter, if limit larger than tool diameter or 0, tool diameter is used")

    def opExecute(self, obj):
        '''opExecute(obj) ... called whenever the receiver needs to be recalculated.
        See documentation of execute() for a list of base functionality provided.
        Should be overwritten by subclasses.'''
        #reload(Adaptive.OpExecute)
        Adaptive.OpExecute.OpExecute(self, obj)

    def opSetDefaultValues(self, obj):
        obj.Side="Inside"
        obj.Tolerance = 0.1
        obj.StepOver = 20
        obj.LiftDistance=1.0
        obj.ProcessHoles = True
        obj.Stopped = False
        obj.StopProcessing = False
        obj.HelixAngle = 5
        obj.HelixDiameterLimit = 0.0
        obj.AdaptiveInputState =""
        obj.AdaptiveOutputState = ""


def Create(name):
    '''Create(name) ... Creates and returns a Pocket operation.'''
    obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", "Adaptive_Pocket_Operation")
    proxy = AdaptivePathOp(obj)
    return obj
