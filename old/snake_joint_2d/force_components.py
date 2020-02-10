import numpy as np
import math as cal

def tensionHoldToLoad(tensionHold:float, frictionCoefficient:float, bendingAngle:float) -> float:
    return tensionHold*cal.exp(-frictionCoefficient*bendingAngle)

def tensionLoadToHold(tensionLoad:float, frictionCoefficient:float, bendingAngle:float) -> float:
    return tensionLoad*cal.exp(frictionCoefficient*bendingAngle)

def cableBendingReactionComponent(tensionLoad:float, tensionHold:float, isTop:bool, jointBendingAngle:float)->np.ndarray:
    return np.array((-tensionLoad*cal.sin(jointBendingAngle/2) if isTop else -tensionHold*cal.sin(jointBendingAngle/2), tensionLoad*cal.cos(jointBendingAngle/2)-tensionHold if isTop else tensionLoad-tensionHold*cal.cos(jointBendingAngle/2)))
    
def endPieceTensionComponent(tension:float):
    return np.array((0,-tension))

def contactingNormalForceComponent(jointBendingAngle:float, isTop:bool, magnitude:float)-> np.ndarray:
    halfBendingAngle = jointBendingAngle/2
    return magnitude*np.array([cal.sin(halfBendingAngle), (-1 if isTop else 1)*cal.cos(halfBendingAngle)])

def contactingFrictionComponent(jointBendingAngle:float, isTop:bool, magnitude:float)-> np.ndarray:
    halfBendingAngle = jointBendingAngle/2
    return magnitude*np.array([(1 if isTop else -1)*cal.cos(halfBendingAngle), cal.sin(halfBendingAngle)])