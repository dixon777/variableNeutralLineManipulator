from math import sin, cos, sqrt
import numpy as np

import pyrr.matrix33 as m3
import pyrr.matrix44 as m4

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
                        topcurve_radius: float,
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
        sqrt(topcurve_radius**2 - horizontalDispAlongCurve**2) + length/2 - topcurve_radius
    ))
    
def evalBottomGuideEndDisp(length: float,
                            bottomcurve_radius: float,
                            tendonDistFromAxis: float,
                            tendonOrientationRF: float)-> np.ndarray:
    """
        Evaluate displacement from disk center to any bottom end of tendon guide
    """
    horizontalDispAlongCurve = tendonDistFromAxis*sin(tendonOrientationRF)
    return np.array((
        tendonDistFromAxis*cos(tendonOrientationRF),
        tendonDistFromAxis*sin(tendonOrientationRF),
        -sqrt(bottomcurve_radius**2 - horizontalDispAlongCurve**2) + bottomcurve_radius - length/2
    ))
    
def evalTopContactDisp(length: float,
                    topcurve_radius: float,
                    topJointAngle: float, 
                    topOrientationRF: float):
    """
        Evaluate displacement from disk center to center of top contacting line
    """
    halfJointAngle = topJointAngle/2
    return np.array((
        topcurve_radius*sin(halfJointAngle)*sin(topOrientationRF),
        -topcurve_radius*sin(halfJointAngle)*cos(topOrientationRF),
        topcurve_radius*cos(halfJointAngle) + length/2 - topcurve_radius
    ))
    
def evalBottomContactDisp(length: float,
                        bottomcurve_radius: float,
                        bottomJointAngle: float):
    """
        Evaluate displacement from disk center to center of bottom contacting line
    """
    halfJointAngle = bottomJointAngle/2
    return np.array((
        0,
        -bottomcurve_radius*sin(halfJointAngle),
        -bottomcurve_radius*cos(halfJointAngle) + bottomcurve_radius - length/2
    ))

def m4MatrixTranslation(vec):
    return m4.create_from_translation(np.array(vec)*1.0).transpose() 

def m4MatrixRotation(axis, radian):
    return m4.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()

def evalTFProximalTopToDistalBottom(jointAngle: float, curveRadius: float):
    rot = m4MatrixRotation((1.0,0,0), jointAngle/2)
    return np.matmul(rot, 
                     np.matmul(m4MatrixTranslation((0.0,0,2*curveRadius*(1-cos(jointAngle/2)))),
                     rot)) # (Wrong) Trick: m_rot * m_trans * m_rot = m_rot * (m_trans + m_rot)
    
    
