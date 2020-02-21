import math
import numpy as np

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

from mpl_toolkits.mplot3d import Axes3D

from .state_management import StateManagement

class RingGeometry():
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
        return RingGeometry(
            length=ring.length,
            cylindricalRadius = cylindricalRadius,
            orientationBF = ring.orientationBF,
            topOrientationRF = ring.topOrientationRF,
            bottomCurveRadius = ring.bottomCurveRadius,
            topCurveRadius = ring.topCurveRadius,
        )



def plotRing(ax, ring:RingGeometry, transform=np.identity(4), radialDivision=30, angleDivision=30):
    r = np.linspace(0, ring.cylindricalRadius, radialDivision)
    theta = np.linspace(0, math.pi*2, angleDivision)
    
    rM, thetaM = np.meshgrid(r, theta)
    
    x = np.multiply(rM, np.cos(thetaM))
    y = np.multiply(rM, np.sin(thetaM))
    
    # Plot bottom surf
    if ring.bottomCurveRadius is not None:
        zBottom = -np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(thetaM-ring.orientationBF)) ** 2) + ring.bottomCurveRadius - ring.length/2
    else:
        zBottom = -ring.length/2*np.ones(np.shape(x))
    
    # Plot top surf
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
    xBody = np.array((xTop[:,-1],xBottom[:,-1]))
    yBody = np.array((yTop[:,-1],yBottom[:,-1]))
    zBody = np.array((zTop[:,-1],zBottom[:,-1]))
    
    ax.plot_surface(xBody, yBody, zBody)
    ax.plot_surface(xBottom,yBottom,zBottom)
    ax.plot_surface(xTop,yTop,zTop)

class ResultGraphWidget(QWidget):
    """
        For graphs
    """
    def __init__(self, parent=None, flags=Qt.WindowFlags()):
        super().__init__(parent=parent, flags=flags)
        self.result = None
        StateManagement().computeTensionsSink.subscribe(self._updateResult)
        
        mainLayout = QVBoxLayout()
        
        self.canvas = FigureCanvas(Figure(figsize=(1,1)))
        mainLayout.addWidget(self.canvas)
        self.setLayout(mainLayout)
        
        self.ax = self.canvas.figure.add_subplot(111, projection='3d')
        # t = np.linspace(0, 10, 501)
        # self.ax.plot(t, np.tan(t), ".")
        # self.ax.title("Not in use")
        
        
    def _updateResult(self, res):
        self.result = res
        self._updateGraph()
        
    def _updateGraph(self):
        if not self.result:
            return
        print("update")
        self.ax.clear()
        for i, s in enumerate(self.result.states):
            cylindricalRadius = max([tms.tendonModel.horizontalDistFromAxis for tms in s.tendonModelStates])
            rg = RingGeometry.fromRing(s.ring, cylindricalRadius)
            plotRing(self.ax, rg, self.result.getTF(i))
            
    
    def minimumSize(self):
        return QSize(400,300)
    
    def resizeEvent(self, QResizeEvent):
        self._updateGraph()