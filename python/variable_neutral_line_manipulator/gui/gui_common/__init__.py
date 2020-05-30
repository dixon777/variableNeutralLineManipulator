from collections.abc import Iterable

from PySide2.QtCore import QPoint, QSize, QTimer, Qt
from PySide2.QtGui import QColor, QDoubleValidator, QIcon, QIntValidator
from PySide2.QtWidgets import QApplication, QCheckBox, QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLayout, QLineEdit, QOpenGLWidget, QPushButton, QScrollArea, QSizePolicy, QSlider, QTextEdit, QVBoxLayout, QWidget

from .common import *

from .protocol import *
from .ranges import *

def removeFromLayout(layout, i):
    """
        Remove i-th item
    """
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
    return True

def removeAllWidgetsFromLayout(layout:QLayout, types=None):
    l = [layout.itemAt(i).widget() for i in range(layout.count())]
    
    for w in l:
        if not types or isinstance(w, types):
            w.deleteLater()
            # w.setParent(None) # prefer deleteLater to bypass the bug caused by consecutive signal from editfinsihing (lose focus and press enter at the same time)
            layout.removeWidget(w)

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

class QCheckBoxWithCB(QCheckBox):
    def __init__(self, initChecked=False, changeCB=None):
        super().__init__()
        self.setChecked(initChecked)
        if changeCB:
            self.stateChanged.connect(changeCB)

class QNumEdit(QLineEdit):
    def __init__(self, initVal=0, editCB=None, parent=None):
        super().__init__(str(initVal), parent=parent)
        self.editingFinished.connect(lambda: editCB(self.text()))
        
    def setText(self, val):
        super().setText(str(val))
    
    def label(self):
        return ""
        
class QIntEdit(QNumEdit):
    def __init__(self, initVal=0, minVal=None, maxVal=None, editCB=None, parent=None):
        super().__init__(initVal, editCB=editCB, parent=parent)
        self.setValidator(QIntValidator(int(minVal), int(maxVal)))

    @property
    def label(self):
        return f"Range: ({self.validator().bottom()}, {self.validator().top()})"
        
class QFloatEdit(QNumEdit):
    def __init__(self, initVal=0.0, minVal=None, maxVal=None, decimal=None, editCB=None, parent=None):
        super().__init__(initVal, editCB=editCB, parent=parent)
        self.setValidator(QDoubleValidator(float(minVal), float(maxVal), int(decimal)))
        
    @property
    def label(self):
        return f"Range: ({self.validator().bottom()}, {self.validator().top()}) [decimals: {self.validator().decimals()}]"
        

