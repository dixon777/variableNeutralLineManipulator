import math

import numpy as np

def toTFVec(vec):
    return np.concatenate((vec[:3],(1,)))

def vecToSkewMatrix(vec):
    return np.array(((0, -vec[2], vec[1]),( vec[2], 0, -vec[0]),(-vec[1], vec[0], 0)))

def getRMFromAxis(axisVector: np.ndarray, theta):
    axisVector = axisVector / np.sum(axisVector)
    skewM = vecToSkewMatrix(axisVector)
    R = np.identity(3) + math.sin(theta)*skewM + (1-math.cos(theta)) * np.matmul(skewM,skewM)
    return R
    
    
def composeTF(RM=np.identity(3), t=np.zeros(3)):
    T = np.identity(4)
    T[0:3, 0:3] = RM
    T[0:3, 3] = t
    return T


# def getRMFromProximalToDistal(jointBendingAngle:float, distalRingOrientationRingFrame:float):
#     return np.matmul(
#         getRMFromAxis(axisVector=(0,0,1), theta=distalRingOrientationRingFrame),
#         getRMFromAxis(axisVector=(1,0,0), theta=jointBendingAngle)
#     )
    
    
def getTFProximalTopToDistalBottom(jointAngle, curveRadius):
    rm = composeTF(RM=getRMFromAxis((1,0,0), jointAngle/2))
    return np.matmul(
        np.matmul(
            rm,
            composeTF(t=np.array((0, 0, 2*curveRadius*(1-math.cos(jointAngle/2))))),
        ),
        rm
    )