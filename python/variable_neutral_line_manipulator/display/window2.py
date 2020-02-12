import math, os, logging

import numpy as np
import pyrr
import rx
from rx.subject import Subject

from PyQt5.QtCore import pyqtSignal, QPoint, QSize, QTimer, Qt
from PyQt5.QtGui import QColor, QDoubleValidator, QIcon, QIntValidator
from PyQt5.QtWidgets import QApplication, QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLineEdit, QOpenGLWidget, QPushButton, QScrollArea, QSlider, QVBoxLayout, QWidget


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
               
    
class StateManagement():
    def __init__(self):
        self.addSegmentConfig = Subject()
        self.removeSegmentConfig = Subject()
        self.notifyGenerateSegments = Subject()
        self.passConfigForSegmentGeneration = Subject()
        
    def __del__(self):
        self.addSegmentConfig.dispose()
        self.removeSegmentConfig.dispose()
        self.notifyGenerateSegments.dispose()
        self.passConfigForSegmentGeneration.dispose()
        
    
sm  = StateManagement()




class QIntEdit(QLineEdit):
    def __init__(self, initVal=0, minVal=None, maxVal=None, parent=None):
        super().__init__(str(initVal), parent=parent)
        self.setValidator(QIntValidator(int(minVal), int(maxVal)))
        
class QFloatEdit(QLineEdit):
    def __init__(self, initVal=0.0, minVal=None, maxVal=None, decimal=None, parent=None):
        super().__init__(str(initVal), parent=parent)
        self.setValidator(QDoubleValidator(float(minVal), float(maxVal), int(decimal)))
        


class ConfigWidget(QWidget):
    def __init__(self, title="Placeholder", parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.formLayout = QFormLayout()
        self._configForm()
        
        box = QGroupBox(title)
        box.setLayout(self.formLayout)
        
        removedButton = QPushButton(QIcon.fromTheme("list-remove"), "Remove")
        removedButton.clicked.connect(lambda: sm.removeSegmentConfig.on_next(self))
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        mainLayout.addWidget(removedButton)
        self.setLayout(mainLayout)
        
    def _configForm(self):
        pairs = {
            "is 2 DoF?:": QCheckBox(""),
            "No. of rings:": QIntEdit("", minVal=0, maxVal=100),
            "Ring length (mm):": QFloatEdit("", minVal=0, maxVal=100, decimal=2),
            "Orientation (deg):": QFloatEdit("", minVal=-180, maxVal=180, decimal=2),
            "Curve radius (mm):": QFloatEdit("", minVal=0, maxVal=100, decimal=2),
            "Cable distance from axis (mm):":QFloatEdit("", minVal=0, maxVal=100, decimal=2),
        }
        for k, v in pairs.items():
            self.formLayout.addRow(k, v)
           
        
class ConfigListWidget(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
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
        
        self.generateButton = QPushButton(QIcon(), "Generate")
        self.generateButton.clicked.connect(lambda: sm.notifyGenerateSegments.on_next(None))
       
        
        self.mainLayout.addLayout(self.buttonGroupLayout)
        self.mainLayout.addWidget(scrollArea)
        self.mainLayout.addWidget(self.generateButton)
        self.setLayout(self.mainLayout)
        
        sm.addSegmentConfig.subscribe(on_next=lambda _: self._addSegmentConfig())
        sm.removeSegmentConfig.subscribe(on_next=lambda x: self._removeSegmentConfig(x))
        
    def _configButtonGroupLayout(self):
        addButton = QPushButton(QIcon("list-add"), "Add")
        addButton.clicked.connect(lambda: sm.addSegmentConfig.on_next(None))
        removeButton = QPushButton(QIcon("list-remove"), "Remove")
        removeButton.clicked.connect(lambda: sm.removeSegmentConfig.on_next(None))

        self.buttonGroupLayout.addWidget(addButton)
        self.buttonGroupLayout.addWidget(removeButton)
        
    def _addSegmentConfig(self):
        self.segmentConfigListLayout.addWidget(ConfigWidget(f"Segment {self.segmentConfigListLayout.count()}"))
        
    def _removeSegmentConfig(self, val):
        if isinstance(val, (QWidget)):
            val.setParent(None)
            self.segmentConfigListLayout.removeWidget(val)
        else:
            removeFromLayout(self.segmentConfigListLayout, -1)
        
    def minimumSizeHint(self):
        return QSize(400,300)
    
class VisualResultWidget(QWidget):
    pass

class Window(QWidget):
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        
        layout = QGridLayout()
        
        layout.addWidget(ConfigListWidget(),0,1,1,2)
        self.setLayout(layout)
        
        
def main():
    global sm
    import sys
    app = QApplication(sys.argv)
    win = Window()
    win.show()
    
    res = app.exec_()
    del sm
    sys.exit(res)
    
if __name__ == "__main__":
    main()