import math 

from ..gui_common import *
from ..backend import *

from uuid import UUID

class SegmentConfigWidget(QWidget):
    def __init__(self, 
                 index,
                 configModel:SegmentConfigDisplayModel, 
                 updateCB,
                 parent=None):
        super().__init__(parent=parent)
        self.configModel:SegmentConfigDisplayModel = configModel
        self.updateCB = updateCB
        
        self.formLayout = QFormLayout()
        self._configForm()
        
        # self.errorLayout = QVBoxLayout()
        # self._configErrorLayout()
        
        verticalWrapperLayout = QVBoxLayout()
        verticalWrapperLayout.addLayout(self.formLayout)
        # verticalWrapperLayout.addLayout(self.errorLayout)
        
        box = QGroupBox(f"{index+1}-th segment")
        box.setLayout(verticalWrapperLayout)
        
        removedButton = QPushButton(QIcon.fromTheme("list-remove"), "Remove")
        removedButton.clicked.connect(lambda: StateManagement().request_remove_segment_config(self.configModel.key))
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(box)
        mainLayout.addWidget(removedButton)
        self.setLayout(mainLayout)
        
    def _configForm(self):
        DoFSettingEdit = QCheckBoxWithCB(initChecked=self.configModel.is_2_DoF, changeCB=lambda v: self._configUpdatedByUser("is_2_DoF", v==2))
        
        numJointEdit = QIntEdit(self.configModel.n_joints , minVal=0, maxVal=100, editCB=lambda v: self._configUpdatedByUser("n_joints", tryParseInt(v)))
        diskLengthEdit = QFloatEdit(self.configModel.disk_length , minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("disk_length", tryParseFloat(v)))
        orientationEdit = QFloatEdit(math.degrees(self.configModel.orientationBF),  minVal=-180, maxVal=180, decimal=2, editCB=lambda v: self._configUpdatedByUser("orientationBF", tryParse(v, [float, math.radians]) ))
        curveRadiusEdit = QFloatEdit(self.configModel.curve_radius, minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("curve_radius", tryParseFloat(v)))
        tendonDistFromAxisEdit = QFloatEdit(self.configModel.tendon_dist_from_axis, minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("tendon_dist_from_axis", tryParseFloat(v)))
        endDiskLengthEdit = QFloatEdit(self.configModel.end_disk_length , minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("end_disk_length", tryParseFloat(v)))

        pairs = {
            "1 DoF (empty) / 2 DoFs (tick)": DoFSettingEdit,
            f"Num joints:\n {numJointEdit.label}": numJointEdit,
            f"Ring length (mm):\n {diskLengthEdit.label}": diskLengthEdit,
            f"Orientation (deg):\n {orientationEdit.label}": orientationEdit,
            f"Curve radius (mm):\n {curveRadiusEdit.label}": curveRadiusEdit,
            f"Tendon distance from axis (mm):\n {tendonDistFromAxisEdit.label}": tendonDistFromAxisEdit,
            f"End disk length (mm):\n {endDiskLengthEdit.label}": endDiskLengthEdit,
        }
        for k, v in pairs.items():
            self.formLayout.addRow(k, v)
            
    def _configUpdatedByUser(self, key, val):
        self.configModel.__dict__[key] = val
        self.updateCB(self.configModel)
        
        
    # def _configErrorLayout(self):
    #     StateManagement().updateSegmentSink.subscribe(self._showErrorsCB)
        
    # def _showErrorsCB(self, collection):
    #     removeAllWidgetsFromLayout(self.errorLayout)
                
    #     for v in collection.errors.values():
    #         if v is None:
    #             continue
    #         self.errorLayout.addWidget(QLabel(str(v)))

class SegmentConfigsWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.manipulatorConfig = None
        
        self.mainLayout = QVBoxLayout()
        self.mainLayout.setAlignment(Qt.AlignTop)
        
        addButton = QPushButton("Add Segment Config")
        addButton.clicked.connect(StateManagement().request_add_segment_config)
        
        textInputFormLayout = QFormLayout()
        self.baseDiskLengthEdit = QFloatEdit(0.0 , minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("base_disk_length", tryParseFloat(v)))
        self.outerDiameterEdit = QFloatEdit(0.0 , minVal=0, maxVal=100, decimal=2, editCB=lambda v: self._configUpdatedByUser("outer_diameter", tryParseFloat(v)))
        textInputPairs = {
            f"Base disk length (mm)\n {self.baseDiskLengthEdit.label}": self.baseDiskLengthEdit,
            f"Outer diameter (mm)\n{self.outerDiameterEdit.label}": self.outerDiameterEdit,
        }
        for k,v in textInputPairs.items():
            textInputFormLayout.addRow(k, v)
            
        
        self.segmentConfigListLayout = QVBoxLayout()
        self.segmentConfigListLayout.setAlignment(Qt.AlignTop)
        intermediateConfigListWidget = QWidget()
        intermediateConfigListWidget.setLayout(self.segmentConfigListLayout)
        scrollArea = QScrollArea()
        scrollArea.setWidgetResizable(True)
        scrollArea.setWidget(intermediateConfigListWidget)
        
        generateButton = QPushButton(QIcon(), "Generate manipulator")
        generateButton.clicked.connect(StateManagement().request_generate_manipulator)
       
        
        self.mainLayout.addWidget(addButton)
        self.mainLayout.addLayout(textInputFormLayout)
        self.mainLayout.addWidget(scrollArea)
        self.mainLayout.addWidget(generateButton)
        self.setLayout(self.mainLayout)
        
        StateManagement().segment_configs_stream.subscribe(on_next=self._updateSegmentConfigs)
        StateManagement().request_init_segment_configs()

    # def _addSegmentConfig(self, idModelPair):
    #     self.idSegmentConfigWidgetMap[idModelPair[0]] = SegmentConfigWidget(idModelPair)
    #     self.segmentConfigListLayout.addWidget(self.idSegmentConfigWidgetMap[idModelPair[0]])
        
    # def _removeSegmentConfig(self, val):
    #     if isinstance(val, UUID):
    #         w = self.idSegmentConfigWidgetMap.get(val)
    #         if w:
    #             w.setParent(None)
    #             self.segmentConfigListLayout.removeWidget(w)
    #             del self.idSegmentConfigWidgetMap[val]
        
    def _updateSegmentConfigs(self, manipulatorConfig:ManipulatorConfigDisplayModel):
        self.manipulatorConfig:ManipulatorConfigDisplayModel = manipulatorConfig
        removeAllWidgetsFromLayout(self.segmentConfigListLayout)
        
        # Update overall param
        self.baseDiskLengthEdit.setText(self.manipulatorConfig.base_disk_length)
        self.outerDiameterEdit.setText(self.manipulatorConfig.outer_diameter)
         
        def _segmentConfigUpdatedByUser(segmentConfig:SegmentConfigDisplayModel):
            self.manipulatorConfig.segment_models[segmentConfig.key] = segmentConfig
            StateManagement().request_update_segment_config(self.manipulatorConfig)
            
        for i, config in enumerate(self.manipulatorConfig.segment_models.values()):
            self.segmentConfigListLayout.addWidget(SegmentConfigWidget(
                i,
                config,
                updateCB=_segmentConfigUpdatedByUser
            ))        
    
    def _configUpdatedByUser(self, k, v):
        self.manipulatorConfig.__dict__[k] = v
        StateManagement().request_update_segment_config(self.manipulatorConfig)
        
    
    def minimumSizeHint(self):
        return QSize(400,400)