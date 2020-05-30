import math
from ..gui_common import *
from ..backend import *

class TensionInputWidget(QWidget):
    def __init__(self, index, knobTendons, userUpdateCB, parent=None):
        super().__init__(parent=parent)
        self.userUpdateCB = userUpdateCB
        
        self.index = index
        self.knobTendons = knobTendons
        
        self.formLayout = QFormLayout()
        self._configFormLayout()
        
        box = QGroupBox(f"{self.index+1}-th segment")
        box.setLayout(self.formLayout)
        
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        self.setLayout(mainLayout)
        
    def _configFormLayout(self):
        for i, tm in enumerate(self.knobTendons):
            w = QFloatEdit(tm.tension, 0, 100, 2, self._updateTensionWrapper(i))
            self.formLayout.addRow(f"{round(math.degrees(tm.orientation), 2)} deg:", w)
            
    def _updateTensionWrapper(self, i):
        def __inner(val):
            self.userUpdateCB(self.index, i, float(val))
        return __inner
        
        
class TensionInputListWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.tensionInputsList = None
        
        self.inputListLayout = QVBoxLayout()
        self.inputListLayout.setAlignment(Qt.AlignTop)
        wrapperWidget = QWidget()
        wrapperWidget.setLayout(self.inputListLayout)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setWidget(wrapperWidget)
        
        computeButton = QPushButton("Compute state")
        computeButton.clicked.connect(StateManagement().request_compute_state)
        
        self.mainLayout = QVBoxLayout()
        self.mainLayout.setAlignment(Qt.AlignTop)
        self.mainLayout.addWidget(scrollArea)
        self.mainLayout.addWidget(computeButton)
        self.setLayout(self.mainLayout)
        
        
        StateManagement().tension_inputs_stream.subscribe(self._showInputs)
        
    def _showInputs(self, tensionInputsList:TensionInputListDisplayModel):
        self.tensionInputsList = tensionInputsList
        removeAllWidgetsFromLayout(self.inputListLayout)
        
        # if error is received
        if isinstance(tensionInputsList, ErrorDict):
            self.inputListLayout.addWidget(QLabel(str(tensionInputsList)))
            return
        
        def __userUpdateConfigPublish(i, j, tension):
            self.tensionInputsList.updateTension(i, j, tension)
            StateManagement().request_update_tensions(self.tensionInputsList)
        
        # if no error occurs
        for i, tensionModels in enumerate(tensionInputsList.values):
            self.inputListLayout.addWidget(TensionInputWidget(i, tensionModels, __userUpdateConfigPublish))
            
    def minimumSizeHint(self):
        return QSize(400,300)
    