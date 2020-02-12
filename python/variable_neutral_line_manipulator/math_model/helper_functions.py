import math
import numpy as np
import pyrr.matrix44 as m4

# Not required if model is assumed frictionless between ring and tendon
def evalCapstan(tensionEnd: float, fricCoef: float, totalAngle: float) -> float:
    """
        Evaluate Capstan equ.
        @param tensionEnd = Resultant tension
    """
    return tensionEnd*math.exp(fricCoef*totalAngle)
    

def allWithin(a,b,threshold=0.0001) -> bool:
    return np.all(np.abs(np.array(a) - np.array(b)) < threshold)

