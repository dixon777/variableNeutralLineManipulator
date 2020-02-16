import numpy as np

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

class ResultGraphWidget(QWidget):
    """
        For graphs
    """
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        mainLayout = QVBoxLayout()
        
        self.canvas = FigureCanvas(Figure(figsize=(1,1)))
        mainLayout.addWidget(self.canvas)
        self.setLayout(mainLayout)
        
        self.ax = self.canvas.figure.subplots()
        t = np.linspace(0, 10, 501)
        self.ax.plot(t, np.tan(t), ".")
        # self.ax.title("Not in use")
        
    def _updateGraph(self):
        self.ax.clear()
        
    
    def minimumSize(self):
        return QSize(400,300)