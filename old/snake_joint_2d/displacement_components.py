import numpy as np
import math as cal

def endPieceKnobDisplacementFromCentroid(length: float, curvatureRadius: float, curvatureAngle: float, isLeft: bool):
    return np.array(((-1 if isLeft else 1)*curvatureRadius * cal.sin(curvatureAngle/2), length/2))

def cornerDisplacementFromCentroid(length: float, curvatureRadius: float, curvatureAngle: float, isTop:bool, isLeft:bool) -> np.ndarray:
    return np.array((0, (1 if isTop else -1)*(length/2-curvatureRadius))) + np.array(((-1 if isLeft else 1) * curvatureRadius*cal.sin(curvatureAngle/2), (1 if isTop else -1) * curvatureRadius*cal.cos(curvatureAngle/2)))

def contactingDisplacementFromCentroid(length: float, curvatureRadius: float, jointBendingAngle:float, isTop:bool) -> np.ndarray:
    return np.array((0,  (1 if isTop else -1)*(length/2-curvatureRadius))) + np.array((-curvatureRadius*cal.sin(jointBendingAngle/2), (1 if isTop else -1) * curvatureRadius*cal.cos(jointBendingAngle/2)))
