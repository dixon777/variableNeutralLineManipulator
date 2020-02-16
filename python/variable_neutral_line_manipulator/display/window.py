import math, os, logging
from uuid import UUID

import numpy as np
import pyrr

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, QTimer, Qt
from PyQt5.QtGui import QColor, QDoubleValidator, QIcon, QIntValidator
from PyQt5.QtWidgets import QApplication, QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QOpenGLWidget, QPushButton, QScrollArea, QSlider, QTextEdit, QVBoxLayout, QWidget

from .state_management import StateManagement
from .result_graph import ResultGraphWidget

def removeFromLayout(layout, i):
    count = layout.count()
    i = i if i >= 0 else count + i
    if i >= count or i < 0:
        return False
    item = layout.itemAt(i)
    if item is None:
        return False
    widget = item.widget()
    if widget is None:
        layout.removeItem(item)
    else:
        layout.removeWidget(widget)
        widget.setParent(None)

def safeInt(v, default=0):
    try:
        v = int(v)
    except:
        v = int(default)
    return v
          
def safeFloat(v, default=0.0):
    try:
        v = float(v)
    except:
        v = float(default)
    return v

class QNumEdit(QLineEdit):
    def __init__(self, initVal=0, textEditCB=None, parent=None):
        super().__init__(str(initVal), parent=parent)
        self.textChanged.connect(textEditCB)
        


class QIntEdit(QNumEdit):
    def __init__(self, initVal=0, minVal=None, maxVal=None, textEditCB=None, parent=None):
        super().__init__(initVal, textEditCB=textEditCB, parent=parent)
        self.setValidator(QIntValidator(safeInt(minVal), safeInt(maxVal)))
        
class QFloatEdit(QNumEdit):
    def __init__(self, initVal=0.0, minVal=None, maxVal=None, decimal=None, textEditCB=None, parent=None):
        super().__init__(initVal, textEditCB=textEditCB, parent=parent)
        self.setValidator(QDoubleValidator(safeFloat(minVal), safeFloat(maxVal), safeInt(decimal)))
        


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
        pairs = {
            "1 DoF (tick) / 2 DoFs (empty)": QCheckBox(),
            "Num joints:": QIntEdit(self.model.numJoints, minVal=0, maxVal=100, textEditCB=self._updateNumJoints),
            "Ring length (mm):": QFloatEdit(self.model.ringLength, minVal=0, maxVal=100, decimal=2, textEditCB=self._updateRingLength),
            "Orientation (deg):": QFloatEdit(math.degrees(self.model.orientationBF),  minVal=-180, maxVal=180, decimal=2, textEditCB=lambda s: self._updateOrientationBF(s)),
            "Curve radius (mm):": QFloatEdit(self.model.curveRadius, minVal=0, maxVal=100, decimal=2, textEditCB=lambda s: self._updateCurveRadius(s)),
            "Tendon distance from axis (mm):":QFloatEdit(self.model.tendonHorizontalDistFromAxis, minVal=0, maxVal=100, decimal=2, textEditCB=lambda s: self._updateTendonDistFromAxis(s)),
        }
        for k, v in pairs.items():
            self.formLayout.addRow(k, v)
            if isinstance(v, QCheckBox):
                v.setChecked(self.model.is1DoF)
                v.stateChanged.connect(self._updateIs1DoF)
            
    def _notifyUpdate(self):
        print("N")
        StateManagement().updateSegmentSrc.on_next((self.id, self.model))
        
        
    def _updateIs1DoF(self,s):
        self.model.is1DoF = s
        self._notifyUpdate()
                   
    def _updateNumJoints(self,s):
        self.model.numJoints = safeInt(s)
        self._notifyUpdate()
        
    def _updateRingLength(self,s):
        self.model.ringLength = safeFloat(s)
        self._notifyUpdate()
        
    def _updateOrientationBF(self,s):
        self.model.orientationBF = math.radians(safeFloat(s))
        self._notifyUpdate()
        
    def _updateCurveRadius(self,s):
        self.model.curveRadius = safeFloat(s)
        self._notifyUpdate()
        
    def _updateTendonDistFromAxis(self,s):
        self.model.tendonHorizontalDistFromAxis = safeFloat(s)
        self._notifyUpdate()
        
    def _configErrorLayout(self):
        StateManagement().updateSegmentSink.subscribe(self._showErrors)
        
    def _showErrors(self, collection):
        ws = [self.errorLayout.itemAt(i).widget() for i in range(self.errorLayout.count())]
        for w in ws:
            if isinstance(w, QLabel):
                w.setParent(None)
                self.errorLayout.removeWidget(w)
                
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
        return QSize(400,300)
    
class TensionInputWidget(QWidget):
    def __init__(self, index, ring, knobTendonModels, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.formLayout = QFormLayout()
        self.index = index
        self.knobTendonModels = knobTendonModels
        self._configFormLayout()
        
        box = QGroupBox(f"{self.index}")
        box.setLayout(self.formLayout)
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        self.setLayout(mainLayout)
        
    def _configFormLayout(self):
        for i, tm in enumerate(self.knobTendonModels):
            print((self.index, i))
            w = QFloatEdit(0, 0, 100, 2, self._setUpdateTensionCB((self.index, i)))
            self.formLayout.addRow(f"{math.degrees(tm.orientationBF)} deg:", w)
            
    def _setUpdateTensionCB(self, indicePair):
        return lambda v: self._updateTension(indicePair, safeFloat(v))
    
    def _updateTension(self, indexPair, value):
        print(indexPair)
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
        
        
        StateManagement().retriveKnobTendonModels.subscribe(self._renewInputs)
        
    def _renewInputs(self, knobTendonModelCompositeList):
        l = [self.inputListLayout.itemAt(i).widget() for i in range(self.inputListLayout.count())]
    
        for w in l:
            w.setParent(None)
            self.inputListLayout.removeWidget(w)
            
        if isinstance(knobTendonModelCompositeList, Exception):
            self.inputListLayout.addWidget(QLabel(str(knobTendonModelCompositeList)))
            return
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
        self.text.append("  Joint angles:")
        for i, s in enumerate(res.states):
            self.text.append(f"    {i}: {math.degrees(s.bottomJointAngle)} deg")
        self.text.append(f"  TF:\n{res.getTF(side='tr')}")
        
        
class Window(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
        layout = QGridLayout()
        layout.addWidget(ResultGraphWidget(),0,0)
        layout.addWidget(ResultTextWidget(), 1,0)
        layout.addWidget(ConfigListWidget(),0,1)
        layout.addWidget(TensionInputListWidget(),1,1)
        self.setLayout(layout)
        

class App():
    @staticmethod
    def run():
        import sys
        app = QApplication(sys.argv)
        w = Window()
        w.show()
        return app.exec_()
    
        
def main():
    App.run(Window)
    
    
    
if __name__ == "__main__":
    main()