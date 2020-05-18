import math
from .gui_common import *


class ResultTextWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.mainLayout = QVBoxLayout()
        self.text = QTextEdit("Result:\n Not computed yet")
        
        self.mainLayout.addWidget(self.text)
        self.setLayout(self.mainLayout)
        
        StateManagement().text_result_stream.subscribe(self._showResult)
        
    def _showResult(self, s):
        self.text.setText(s)