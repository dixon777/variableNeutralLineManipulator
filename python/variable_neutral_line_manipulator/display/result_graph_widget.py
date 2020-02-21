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

class RingPlotGeometry():
    def __init__(self, 
                 length, 
                 cylindricalRadius,
                 orientationBF, 
                 bottomCurveRadius=None, 
                 topOrientationRF=None, 
                 topCurveRadius=None):
        self.length = length
        self.cylindricalRadius = cylindricalRadius
        self.orientationBF = orientationBF
        self.topOrientationRF =topOrientationRF
        self.bottomCurveRadius = bottomCurveRadius
        self.topCurveRadius = topCurveRadius
    
    @staticmethod
    def fromRing(ring, cylindricalRadius):
        return RingPlotGeometry(
            length=ring.length,
            cylindricalRadius = cylindricalRadius,
            orientationBF = ring.orientationBF,
            topOrientationRF = ring.topOrientationRF,
            bottomCurveRadius = ring.bottomCurveRadius,
            topCurveRadius = ring.topCurveRadius,
        )



def plotRing(ax, ring:RingPlotGeometry, transform=np.identity(4), radialDivision=30, angleDivision=30, range:Range3d=None):
    r = np.linspace(0, ring.cylindricalRadius, radialDivision)
    theta = np.linspace(0, math.pi*2, angleDivision)
    
    rM, thetaM = np.meshgrid(r, theta)
    
    x = np.multiply(rM, np.cos(thetaM))
    y = np.multiply(rM, np.sin(thetaM))
    
    # Define bottom surf
    if ring.bottomCurveRadius is not None:
        zBottom = -np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(thetaM-ring.orientationBF)) ** 2) + ring.bottomCurveRadius - ring.length/2
    else:
        zBottom = -ring.length/2*np.ones(np.shape(x))
    
    # Define top surf
    if ring.topCurveRadius is not None:
        topOrientationRF = ring.topOrientationRF if ring.topOrientationRF is not None else 0.0
        zTop = np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(thetaM-topOrientationRF-ring.orientationBF)) ** 2) - ring.bottomCurveRadius + ring.length/2
    else:
        zTop = ring.length/2*np.ones(np.shape(x))
    
  
    a = np.dot(transform, np.reshape(np.array((x,y,zBottom, np.ones(np.shape(x)))), (4,-1)))
    b = np.dot(transform, np.reshape(np.array((x,y,zTop, np.ones(np.shape(x)))), (4,-1)))
    
    xBottom = np.reshape(a[0,:], rM.shape)
    yBottom = np.reshape(a[1,:], rM.shape)
    zBottom = np.reshape(a[2,:], rM.shape)
    xTop = np.reshape(b[0,:], rM.shape)
    yTop =  np.reshape(b[1,:], rM.shape)
    zTop = np.reshape(b[2,:], rM.shape)
    
    # Define body surf
    xBody = np.array((xTop[:,-1],xBottom[:,-1]))
    yBody = np.array((yTop[:,-1],yBottom[:,-1]))
    zBody = np.array((zTop[:,-1],zBottom[:,-1]))
    
    if range:
        xs = xBody.flatten()
        ys = yBody.flatten()
        zs = zBody.flatten()
        range.update(x=(min(xs), max(xs)), y=(min(ys), max(ys)), z=(min(zs), max(zs)))
    
    ax.plot_surface(xBody, yBody, zBody)
    ax.plot_surface(xBottom,yBottom,zBottom)
    ax.plot_surface(xTop,yTop,zTop)
    

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
            plotRing(self.ax, rg, self.result.getTF(i), range=range3d)
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