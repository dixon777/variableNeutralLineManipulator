import math
import numpy as np

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtWidgets import QSizePolicy, QVBoxLayout, QWidget

from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from matplotlib.figure import Figure

from mpl_toolkits.mplot3d import Axes3D

from .state_management import StateManagement
from .ranges import Range3d, enforceRange
from .plot import *


class ResultGraphWidget(QWidget):
    """
        For graphs
    """
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        super().__init__(parent)
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.canvas = FigureCanvas(fig)
        self.ax = fig.add_subplot(111, projection='3d')
        FigureCanvas.updateGeometry(self.canvas) # Not sure whether it is necessary
        
        mainLayout = QVBoxLayout()
        mainLayout.addWidget(self.canvas)
        self.setLayout(mainLayout)
        # sizePolicy = QSizePolicy()
        # # sizePolicy.setHeightForWidth(True)
        # sizePolicy.setHorizontalPolicy(QSizePolicy.Expanding)
        # sizePolicy.setVerticalPolicy(QSizePolicy.MinimumExpanding)
        # self.setSizePolicy(sizePolicy)
        
        self.result = None
        StateManagement().computeTensionsSink.subscribe(self._updateResult)
        StateManagement().graphResizeUpdateSink.subscribe(lambda _: self._updateGraph)
        
        self._updateGraph()
        
        
    def _updateResult(self, res):
        self.result = res
        self._updateGraph()
        
    def _updateGraph(self):
        if not self.result:
            return
        self.ax.clear()
        
        range3d = Range3d()
        for i, s in enumerate(self.result.states):
            cylindricalRadius = max([tms.tendonModel.horizontalDistFromAxis for tms in s.tendonModelStates])
            rg = RingPlotGeometry.fromRing(s.ring, cylindricalRadius)
            plotRingRF(self.ax, rg, self.result.getTF(i), range=range3d)
        enforceRange(self.ax, range3d)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.canvas.draw()
            
    
    def resizeEvent(self, event):
        width = event.size().width()
        height = event.size().height()
        if width > height:
            self.resize(height, height)
        elif width < height:
            self.resize(width, width)
        else:
            StateManagement().graphResizeUpdateSrc.on_next(None)
        
    def minimumSizeHint(self):
        return QSize(400,400)