import math, os, logging
from uuid import UUID

import numpy as np
import pyrr

from .gui_common import *

from .widgets.segment_config_widget import *
from .widgets.tension_input_list_widget import TensionInputListWidget
from .widgets.result_graph_widget import ResultGraphWidget
from .widgets.result_text_widget import *

        
class MainWindow(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        
        layout = QGridLayout()
        layout.addWidget(ResultGraphWidget(self),0,0)
        layout.addWidget(ResultTextWidget(self), 1,0)
        layout.addWidget(SegmentConfigsWidget(self),0,1)
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
    
