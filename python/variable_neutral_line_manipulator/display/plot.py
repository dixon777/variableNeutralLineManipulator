import math
import numpy as np

from .ranges import Range3d

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



def plotRingRF(ax, ring:RingPlotGeometry, transform=np.identity(4), radialDivision=25, angleDivision=25, range:Range3d=None):
    """
        Without defining arg "transform", a ring object is plotted with its base curvature rotates about the axis parallel to the x-axis
    """
    r = np.linspace(0, ring.cylindricalRadius, radialDivision)
    theta = np.linspace(0, math.pi*2, angleDivision)
    
    rM, thetaM = np.meshgrid(r, theta)
    
    x = np.multiply(rM, np.cos(thetaM))
    y = np.multiply(rM, np.sin(thetaM))
    
    # Define bottom surf
    if ring.bottomCurveRadius is not None:
        zBottom = -np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(thetaM)) ** 2) + ring.bottomCurveRadius - ring.length/2
    else:
        zBottom = -ring.length/2*np.ones(np.shape(x))
    
    # Define top surf
    if ring.topCurveRadius is not None:
        topOrientationRF = ring.topOrientationRF if ring.topOrientationRF is not None else 0.0
        zTop = np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(thetaM-topOrientationRF)) ** 2) - ring.bottomCurveRadius + ring.length/2
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
    