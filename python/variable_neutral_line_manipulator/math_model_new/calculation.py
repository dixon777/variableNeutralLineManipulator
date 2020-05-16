from math import sin, cos, sqrt
import numpy as np

import pyrr.matrix33 as m3

def m3MatrixRotation(axis, radian):
    return m3.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()

def distalToProximalFrame(distalDiskBottomVecRF: np.ndarray,
                       topJointAngle: float,
                       topOrientationRF: float) -> np.ndarray:
    """
        Change distal disk's bottom reaction in distal disk's frame to disk's top reaction in disk's frame
    """
    return np.dot(m3MatrixRotation((0, 0, 1.0), topOrientationRF), 
                  np.dot(m3MatrixRotation((1.0, 0, 0), topJointAngle), -distalDiskBottomVecRF))


def evalTopGuideForce(tensionInDisk: float,
                    topJointAngle: float,
                    topOrientationRF: float) -> np.ndarray:
    """
        Evaluate force component at any top end of tendon guide
    """
    halfJointAngle = topJointAngle/2
    return np.array((
        tensionInDisk*sin(halfJointAngle)*sin(topOrientationRF),
        -tensionInDisk*sin(halfJointAngle)*cos(topOrientationRF),
        tensionInDisk*cos(halfJointAngle)
    ))
    
def evalBottomGuideForce(tensionInDisk: float,
                        bottomJointAngle: float) -> np.ndarray:
    """
        Evaluate force component at any bottom end of tendon guide
    """
    halfJointAngle = bottomJointAngle/2
    return np.array((
        0,
        -tensionInDisk*sin(halfJointAngle),
        -tensionInDisk*cos(halfJointAngle)
    ))

def evalTopGuideEndDisp(length: float,
                        topCurveRadius: float,
                        tendonDistFromAxis: float,
                        tendonOrientationRF: float,
                        topOrientationRF: float) -> np.ndarray:
    """
        Evaluate displacement from disk center to any top end of tendon guide
    """
    horizontalDispAlongCurve = tendonDistFromAxis*sin(tendonOrientationRF - topOrientationRF)
    return np.array((
        tendonDistFromAxis*cos(tendonOrientationRF),
        tendonDistFromAxis*sin(tendonOrientationRF),
        sqrt(topCurveRadius**2 - horizontalDispAlongCurve**2) + length/2 - topCurveRadius
    ))
    
def evalBottomGuideEndDisp(length: float,
                            bottomCurveRadius: float,
                            tendonDistFromAxis: float,
                            tendonOrientationRF: float)-> np.ndarray:
    """
        Evaluate displacement from disk center to any bottom end of tendon guide
    """
    horizontalDispAlongCurve = tendonDistFromAxis*sin(tendonOrientationRF)
    return np.array((
        tendonDistFromAxis*cos(tendonOrientationRF),
        tendonDistFromAxis*sin(tendonOrientationRF),
        -sqrt(bottomCurveRadius**2 - horizontalDispAlongCurve**2) + bottomCurveRadius - length/2
    ))
    
def evalTopContactDisp(length: float,
                    topCurveRadius: float,
                    topJointAngle: float, 
                    topOrientationRF: float):
    """
        Evaluate displacement from disk center to center of top contacting line
    """
    halfJointAngle = topJointAngle/2
    return np.array((
        topCurveRadius*sin(halfJointAngle)*sin(topOrientationRF),
        -topCurveRadius*sin(halfJointAngle)*cos(topOrientationRF),
        topCurveRadius*cos(halfJointAngle) + length/2 - topCurveRadius
    ))
    
def evalBottomContactDisp(length: float,
                        bottomCurveRadius: float,
                        bottomJointAngle: float):
    """
        Evaluate displacement from disk center to center of bottom contacting line
    """
    halfJointAngle = bottomJointAngle/2
    return np.array((
        0,
        -bottomCurveRadius*sin(halfJointAngle),
        -bottomCurveRadius*cos(halfJointAngle) + bottomCurveRadius - length/2
    ))
    
    
