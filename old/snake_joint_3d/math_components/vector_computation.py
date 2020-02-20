import math

import numpy as np

def vecToSkewMatrix(vec):
    return np.array(((0, -vec[2], vec[1]),( vec[2], 0, -vec[0]),(-vec[1], vec[0], 0)))

def getRMFromAxis(axisVector: np.ndarray, theta):
    axisVector = axisVector / np.sum(axisVector)
    skewM = vecToSkewMatrix(axisVector)
    R = np.identity(3) + math.sin(theta)*skewM + (1-math.cos(theta)) * np.matmul(skewM,skewM)
    return R
    
    
def composeTF(rotationMatrix=np.identity(3), translationVector=np.zeros((3,1))):
    T = np.identity(4)
    T[0:3, 0:3] = rotationMatrix
    T[0:3, 3] = translationVector
    return T


def getRMFromProximalToDistal(jointBendingAngle:float, distalRingOrientationRingFrame:float):
    return np.matmul(
        getRMFromAxis(axisVector=(0,0,1), theta=distalRingOrientationRingFrame),
        getRMFromAxis(axisVector=(1,0,0), theta=jointBendingAngle)
    )
    