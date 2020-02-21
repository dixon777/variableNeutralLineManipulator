import math, os, logging
from uuid import UUID
from collections.abc import Iterable

import numpy as np
import pyrr

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, QTimer, Qt
from PyQt5.QtGui import QColor, QDoubleValidator, QIcon, QIntValidator
from PyQt5.QtWidgets import QApplication, QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QOpenGLWidget, QPushButton, QScrollArea, QSlider, QTextEdit, QVBoxLayout, QWidget

from .state_management import StateManagement
from .result_graph_widget import ResultGraphWidget
from .helper import *


def tryParse(v, funcs, default=None):
    if not isinstance(funcs, Iterable):
        funcs = [funcs]
    
    for f in funcs:
        try:
            v = f(v)
        except:
            return default
    return v

def tryParseInt(v, default=None):
    return tryParse(v, int, default)
          
def tryParseFloat(v, default=None):
    return tryParse(v, float, default)

class QNumEdit(QLineEdit):
    def __init__(self, initVal=0, textEditCB=None, parent=None):
        super().__init__(str(initVal), parent=parent)
        self.textChanged.connect(textEditCB)
    
    def label(self):
        return ""
        
class QIntEdit(QNumEdit):
    def __init__(self, initVal=0, minVal=None, maxVal=None, textEditCB=None, parent=None):
        super().__init__(initVal, textEditCB=textEditCB, parent=parent)
        self.setValidator(QIntValidator(int(minVal), int(maxVal)))

    @property
    def label(self):
        return f"Range: ({self.validator().bottom()}, {self.validator().top()})"
        
class QFloatEdit(QNumEdit):
    def __init__(self, initVal=0.0, minVal=None, maxVal=None, decimal=None, textEditCB=None, parent=None):
        super().__init__(initVal, textEditCB=textEditCB, parent=parent)
        self.setValidator(QDoubleValidator(float(minVal), float(maxVal), int(decimal)))
        
    @property
    def label(self):
        return f"Range: ({self.validator().bottom()}, {self.validator().top()}) [decimals: {self.validator().decimals()}]"
        


class ConfigWidget(QWidget):
    def __init__(self, idModelPair, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.id, self.model = idModelPair
        self.formLayout = QFormLayout()
        self._configForm()
        
        self.errorLayout = QVBoxLayout()
        self._configErrorLayout()
        
        verticalWrapperLayout = QVBoxLayout()
        verticalWrapperLayout.addLayout(self.formLayout)
        verticalWrapperLayout.addLayout(self.errorLayout)
        
        box = QGroupBox(f"{self.id}")
        box.setLayout(verticalWrapperLayout)
        
        removedButton = QPushButton(QIcon.fromTheme("list-remove"), "Remove")
        removedButton.clicked.connect(lambda: StateManagement().removeSegmentConfigSrc.on_next(self.id))
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        mainLayout.addWidget(removedButton)
        self.setLayout(mainLayout)
        
    def _configForm(self):
        numJointEdit = QIntEdit(self.model.numJoints, minVal=0, maxVal=100, textEditCB=lambda v: self._updateModelParam("numJoints", tryParseInt(v)))
        ringLengthEdit = QFloatEdit(self.model.ringLength, minVal=0, maxVal=100, decimal=2, textEditCB=lambda v: self._updateModelParam("ringLength", tryParseFloat(v)))
        orientationEdit = QFloatEdit(self.model.orientationBF,  minVal=-180, maxVal=180, decimal=2, textEditCB=lambda v: self._updateModelParam("orientationBF", tryParse(v, (float, math.radians)) ))
        curveRadiusEdit = QFloatEdit(self.model.curveRadius, minVal=0, maxVal=100, decimal=2, textEditCB=lambda v: self._updateModelParam("curveRadius", tryParseFloat(v)))
        tendonDistFromAxisEdit = QFloatEdit(self.model.tendonHorizontalDistFromAxis, minVal=0, maxVal=100, decimal=2, textEditCB=lambda v: self._updateModelParam("tendonHorizontalDistFromAxis", tryParseFloat(v)))
        
        pairs = {
            "1 DoF (tick) / 2 DoFs (empty)": QCheckBox(),
            f"Num joints:\n {numJointEdit.label}": numJointEdit,
            f"Ring length (mm):\n {ringLengthEdit.label}": ringLengthEdit,
            f"Orientation (deg):\n {orientationEdit.label}": orientationEdit,
            f"Curve radius (mm):\n {curveRadiusEdit.label}": curveRadiusEdit,
            f"Tendon distance from axis (mm):\n {tendonDistFromAxisEdit.label}": tendonDistFromAxisEdit,
        }
        for k, v in pairs.items():
            self.formLayout.addRow(k, v)
            if isinstance(v, QCheckBox):
                v.setChecked(self.model.is1DoF)
                v.stateChanged.connect(lambda v: self._updateModelParam("is1DoF", v))
            
    def _updateModelParam(self, key, val):
        self.model.__dict__[key] = val
        StateManagement().updateSegmentSrc.on_next((self.id, self.model))
        
        
    def _configErrorLayout(self):
        StateManagement().updateSegmentSink.subscribe(self._showErrorsCB)
        
    def _showErrorsCB(self, collection):
        removeAllWidgetsFromLayout(self.errorLayout)
                
        for v in collection.errors.values():
            if v is None:
                continue
            self.errorLayout.addWidget(QLabel(str(v)))
        
        
        
class ConfigListWidget(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
        self.idSegmentConfigWidgetMap = {}
        
        self.mainLayout = QVBoxLayout()
        self.mainLayout.setAlignment(Qt.AlignTop)
        self.buttonGroupLayout = QHBoxLayout()
        self._configButtonGroupLayout()
        
        self.segmentConfigListLayout = QVBoxLayout()
        self.segmentConfigListLayout.setAlignment(Qt.AlignTop)
        wrapperWidget = QWidget()
        wrapperWidget.setLayout(self.segmentConfigListLayout)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setWidget(wrapperWidget)
        
        self.generateButton = QPushButton(QIcon(), "Generate Manipulator")
        self.generateButton.clicked.connect(lambda: StateManagement().generateSegmentsSrc.on_next(None))
       
        
        self.mainLayout.addLayout(self.buttonGroupLayout)
        self.mainLayout.addWidget(scrollArea)
        self.mainLayout.addWidget(self.generateButton)
        self.setLayout(self.mainLayout)
        
        StateManagement().addSegmentConfigSink.subscribe(on_next=lambda idModelPair: self._addSegmentConfig(idModelPair))
        StateManagement().removeSegmentConfigSink.subscribe(on_next=lambda key: self._removeSegmentConfig(key))
        
    def _configButtonGroupLayout(self):
        addButton = QPushButton("Add Segment Config")
        addButton.clicked.connect(lambda: StateManagement().addSegmentConfigSrc.on_next(None))

        self.buttonGroupLayout.addWidget(addButton)
        
    def _addSegmentConfig(self, idModelPair):
        self.idSegmentConfigWidgetMap[idModelPair[0]] = ConfigWidget(idModelPair)
        self.segmentConfigListLayout.addWidget(self.idSegmentConfigWidgetMap[idModelPair[0]])
        
    def _removeSegmentConfig(self, val):
        if isinstance(val, UUID):
            w = self.idSegmentConfigWidgetMap.get(val)
            if w:
                w.setParent(None)
                self.segmentConfigListLayout.removeWidget(w)
                del self.idSegmentConfigWidgetMap[val]
                
        # elif isinstance(val, (QWidget)):
        #     val.setParent(None)
        #     self.segmentConfigListLayout.removeWidget(val)
        # else:
        #     removeFromLayout(self.segmentConfigListLayout, -1)
        
    def minimumSizeHint(self):
        return QSize(200,200)
    
class TensionInputWidget(QWidget):
    def __init__(self, index, ring, knobTendonModels, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.formLayout = QFormLayout()
        self.index = index
        self.knobTendonModels = knobTendonModels
        self._configFormLayout()
        
        box = QGroupBox(f"{self.index+1}-th segment")
        box.setLayout(self.formLayout)
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        self.setLayout(mainLayout)
        
    def _configFormLayout(self):
        for i, tm in enumerate(self.knobTendonModels):
            w = QFloatEdit(0, 0, 100, 2, self._updateTensionWrapper((self.index, i)))
            self.formLayout.addRow(f"{round(math.degrees(tm.orientationBF), 2)} deg:", w)
            
    # Dummy but required for enforcing indicesPair to be a constant instead of dynamically evaluated
    def _updateTensionWrapper(self, indicesPair):
        return lambda v: self._updateTension(indicesPair, tryParseFloat(v, 0.0))
    
    def _updateTension(self, indexPair, value):
        StateManagement().updateTensionsSrc.on_next((indexPair, value))
        
        
class TensionInputListWidget(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
        self.inputListLayout = QVBoxLayout()
        self.inputListLayout.setAlignment(Qt.AlignTop)
        wrapperWidget = QWidget()
        wrapperWidget.setLayout(self.inputListLayout)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setWidget(wrapperWidget)
        
        computeButton = QPushButton("Compute Static State")
        computeButton.clicked.connect(StateManagement().computeTensionsSrc.on_next)
        
        self.mainLayout = QVBoxLayout()
        self.mainLayout.setAlignment(Qt.AlignTop)
        self.mainLayout.addWidget(scrollArea)
        self.mainLayout.addWidget(computeButton)
        self.setLayout(self.mainLayout)
        
        
        StateManagement().retriveKnobTendonModels.subscribe(self._showInputs)
        
    def _showInputs(self, knobTendonModelCompositeList):
        removeAllWidgetsFromLayout(self.inputListLayout)
        
        # if error is received
        if isinstance(knobTendonModelCompositeList, Exception):
            self.inputListLayout.addWidget(QLabel(str(knobTendonModelCompositeList)))
            return
        
        # if no error occurs
        for i, (r, tms) in enumerate(knobTendonModelCompositeList):
            self.inputListLayout.addWidget(TensionInputWidget(i, r, tms))
            
    def minimumSizeHint(self):
        return QSize(400,300)
    

class ResultTextWidget(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.mainLayout = QVBoxLayout()
        self.text = QTextEdit("Result:\n Not computed yet")
        
        self.mainLayout.addWidget(self.text)
        self.setLayout(self.mainLayout)
        
        StateManagement().computeTensionsSink.subscribe(self._showResult)
        
        
    
    def _showResult(self, res):
        self.text.setText(f"Result:\n")
        if res.error:
            self.text.append(f"Error: {res.error.__repr__()}")
            return
        self.text.append("  Joint angles (from proximal to distal):")
        for i, s in enumerate(res.states):
            self.text.append(f"    {i+1}-th joint: {math.degrees(s.bottomJointAngle)} deg")
        self.text.append(f"")
        self.text.append(f"  TF:\n{res.getTF(side='tr')}\n")
        self.text.append(f"  Tendon lengths:")
        for i, ts in enumerate(res.computeTendonLengths()):
            self.text.append(f"    {i+1}-th segment:")
            for j, t in enumerate(ts):
                self.text.append(f"     {j+1}-th tendon: {t}")
        
        
class MainWindow(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
        layout = QGridLayout()
        layout.addWidget(ResultGraphWidget(self),0,0)
        layout.addWidget(ResultTextWidget(self), 1,0)
        layout.addWidget(ConfigListWidget(self),0,1)
        layout.addWidget(TensionInputListWidget(self),1,1)
        self.setLayout(layout)
        

class App():
    @staticmethod
    def run():
        import sys
        app = QApplication(sys.argv)
        w = MainWindow()
        w.show()
        return app.exec_()
    
