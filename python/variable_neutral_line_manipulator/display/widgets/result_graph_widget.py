import math
import numpy as np

from .gui_common import *

from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from matplotlib.figure import Figure

from mpl_toolkits.mplot3d import Axes3D

from ..ranges import Range3d, enforceRange
from ..plot import *
from ...math_model import ManipulatorState


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
        
        
        sizePolicy = QSizePolicy()
        # sizePolicy.setHeightForWidth(True)
        sizePolicy.setHorizontalPolicy(QSizePolicy.Expanding)
        sizePolicy.setVerticalPolicy(QSizePolicy.MinimumExpanding)
        self.setSizePolicy(sizePolicy)
        
        StateManagement().graph_stream.subscribe(self._updateGraph)
        
        self._updateGraph()
        
        mainLayout = QHBoxLayout()
        mainLayout.addWidget(self.canvas)
        self.setLayout(mainLayout)
        
        
    def _updateGraph(self, manipulator_state:ManipulatorState=None):
        self.ax.clear()
        
        if manipulator_state:
            max_ranges = Range3d()
            for i, s in enumerate(manipulator_state.TFs_DF):
                plot_TFs(self.ax, manipulator_state, max_ranges)
                # cylindricalRadius = max([tms.tendonModel.horizontalDistFromAxis for tms in s.tendonModelStates])
                # rg = RingPlotGeometry.fromRing(s.ring, cylindricalRadius)
                # plotRingDF(self.ax, rg, self.result.getTF(i), range=equal_scale_range3d)
            enforceRange(self.ax, max_ranges)
            
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
        # else:
        #     StateManagement().graphResizeUpdateSrc.on_next(None)
        
    def minimumSizeHint(self):
        return QSize(400,400)