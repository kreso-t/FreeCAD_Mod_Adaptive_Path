
import FreeCAD
import FreeCADGui
import PathScripts.PathLog as PathLog
import PathScripts.PathGui as PathGui
import PathScripts.PathOpGui as PathOpGui
from PySide import QtCore, QtGui

class TaskPanelOpPage(PathOpGui.TaskPanelPage):
    def initPage(self, obj):
        self.setTitle("Adaptive pocket operation")

    def getIcon(self, icon):
        '''getIcon(icon) ... return icon for page or None.'''
        return ":/Path-Adaptive.svg"

    def getForm(self):
        form = QtGui.QWidget()
        layout = QtGui.QVBoxLayout()

        #tool contoller
        hlayout = QtGui.QHBoxLayout()
        form.toolController = QtGui.QComboBox()
        #form.side.addItem("Inside")
        #form.side.addItem("Outside")
        form.toolControllerLabel=QtGui.QLabel("Tool controller")
        hlayout.addWidget(form.toolControllerLabel)
        hlayout.addWidget(form.toolController)
        layout.addLayout(hlayout)

        #cut region
        formLayout = QtGui.QFormLayout()
        form.side = QtGui.QComboBox()
        form.side.addItem("Inside")
        form.side.addItem("Outside")
        form.side.setToolTip("Cut inside or outside of the selected face")
        formLayout.addRow(QtGui.QLabel("Cut Region"),form.side)

        form.StepOver = QtGui.QSpinBox()
        form.StepOver.setMinimum(10)
        form.StepOver.setMaximum(60)
        form.StepOver.setSingleStep(1)
        form.StepOver.setValue(20)
        form.StepOver.setToolTip("Tool step over percentage")
        formLayout.addRow(QtGui.QLabel("Step Over Percent"),form.StepOver)

        form.Tolerance = QtGui.QSlider(QtCore.Qt.Horizontal)
        form.Tolerance.setMinimum(5)
        form.Tolerance.setMaximum(15)
        form.Tolerance.setTickInterval(1)
        form.Tolerance.setValue(10)
        form.Tolerance.setTickPosition(QtGui.QSlider.TicksBelow)
        form.Tolerance.setToolTip("Influences calculation performace vs stability and accuracy")
        formLayout.addRow(QtGui.QLabel("Precision vs Performance"),form.Tolerance)

        form.HelixAngle = QtGui.QDoubleSpinBox()
        form.HelixAngle.setMinimum(0.1)
        form.HelixAngle.setMaximum(90)
        form.HelixAngle.setSingleStep(0.1)
        form.HelixAngle.setValue(5)
        form.HelixAngle.setToolTip("Angle of the helix ramp entry")
        formLayout.addRow(QtGui.QLabel("Helix ramp angle"),form.HelixAngle)

        form.HelixDiameterLimit = QtGui.QDoubleSpinBox()
        form.HelixDiameterLimit.setMinimum(0.0)
        form.HelixDiameterLimit.setMaximum(90)
        form.HelixDiameterLimit.setSingleStep(0.1)
        form.HelixDiameterLimit.setValue(0)
        form.HelixDiameterLimit.setToolTip("If non zero it limits the size helix diameter, otherwise the tool radius is taken as the helix diameter")
        formLayout.addRow(QtGui.QLabel("Helix max diameter"),form.HelixDiameterLimit)

        form.LiftDistance = QtGui.QDoubleSpinBox()
        form.LiftDistance.setMinimum(0.0)
        form.LiftDistance.setMaximum(1000)
        form.LiftDistance.setSingleStep(0.1)
        form.LiftDistance.setValue(1.0)
        form.LiftDistance.setToolTip("How much to lift the tool up during the rapid repositioning moves (used when no obstacles)")
        formLayout.addRow(QtGui.QLabel("Lift distance"),form.LiftDistance)




        form.ProcessHoles = QtGui.QCheckBox()
        form.ProcessHoles.setChecked(True)
        formLayout.addRow(QtGui.QLabel("Process Holes"),form.ProcessHoles)



        layout.addLayout(formLayout)

        form.StopButton=QtGui.QPushButton("Stop")
        form.StopButton.setCheckable(True)
        layout.addWidget(form.StopButton)
        #button
        # form.button = QtGui.QPushButton("Update")
        # layout.addWidget(form.button)

        form.setLayout(layout)
        return form

    def getSignalsForUpdate(self, obj):

        '''getSignalsForUpdate(obj) ... return list of signals for updating obj'''
        signals = []
        #signals.append(self.form.button.clicked)
        signals.append(self.form.side.currentIndexChanged)
        signals.append(self.form.toolController.currentIndexChanged)
        signals.append(self.form.StepOver.valueChanged)
        signals.append(self.form.Tolerance.valueChanged)
        signals.append(self.form.HelixAngle.valueChanged)
        signals.append(self.form.HelixDiameterLimit.valueChanged)
        signals.append(self.form.LiftDistance.valueChanged)

        signals.append(self.form.ProcessHoles.stateChanged)
        signals.append(self.form.StopButton.toggled)
        return signals
    def setFields(self, obj):
        self.selectInComboBox(obj.Side, self.form.side)
        self.form.StepOver.setValue(obj.StepOver)
        self.form.Tolerance.setValue(int(obj.Tolerance*100))
        self.form.HelixAngle.setValue(obj.HelixAngle)
        self.form.HelixDiameterLimit.setValue(obj.HelixDiameterLimit)
        self.form.LiftDistance.setValue(obj.LiftDistance)

        self.form.ProcessHoles.setChecked(obj.ProcessHoles)
        self.setupToolController(obj, self.form.toolController)
        self.form.StopButton.setChecked(obj.Stopped)
        obj.setEditorMode('AdaptiveInputState', 2) #hide this property
        obj.setEditorMode('AdaptiveOutputState', 2) #hide this property
        obj.setEditorMode('StopProcessing', 2)  # hide this property
        obj.setEditorMode('Stopped', 2)  # hide this property


        #print "setFields"


    def getFields(self, obj):
        if obj.Side != str(self.form.side.currentText()):
            obj.Side = str(self.form.side.currentText())

        obj.StepOver = self.form.StepOver.value()
        obj.Tolerance = 1.0*self.form.Tolerance.value()/100.0
        obj.HelixAngle = self.form.HelixAngle.value()
        obj.HelixDiameterLimit = self.form.HelixDiameterLimit.value()
        obj.LiftDistance = self.form.LiftDistance.value()

        obj.ProcessHoles = self.form.ProcessHoles.isChecked()
        obj.Stopped = self.form.StopButton.isChecked()
        if(obj.Stopped):
            self.form.StopButton.setChecked(False)  #reset the button
            obj.StopProcessing=True

        self.updateToolController(obj, self.form.toolController)
        obj.setEditorMode('AdaptiveInputState', 2) #hide this property
        obj.setEditorMode('AdaptiveOutputState', 2) #hide this property


