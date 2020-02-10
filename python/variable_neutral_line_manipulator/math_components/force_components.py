import math

import numpy as np

from . import vector_computation as vc


def evalCapstan(tensionEnd: float, fricCoef: float, totalAngle: float) -> float:
    return tensionEnd*math.exp(fricCoef*totalAngle)


def evalTopCableGuideComp(tensionInRing: float,
                          fricCoef: float,
                          topJointAngle: float,
                          topOrientationRF: float):
    halfJointAngle = topJointAngle/2
    tensionLoad = evalCapstan(tensionInRing, fricCoef, -halfJointAngle)
    return np.array((
        tensionLoad*math.sin(halfJointAngle)*math.sin(topOrientationRF),
        -tensionLoad*math.sin(halfJointAngle)*math.cos(topOrientationRF),
        tensionLoad*math.cos(halfJointAngle) - tensionInRing
    ))


def evalBottomCableGuideComp(tensionInRing: float,
                             fricCoef: float,
                             bottomJointAngle: float) -> np.ndarray:
    halfJointAngle = bottomJointAngle/2
    tensionHold = evalCapstan(tensionInRing, fricCoef, halfJointAngle)
    return np.array((
        0,
        -tensionHold*math.sin(halfJointAngle),
        tensionInRing - tensionHold*math.cos(halfJointAngle)
    ))


def evalKnobComp(tension: float):
    return np.array((0, 0, -tension))


def evalTopContactComp(DRBottomContactCompDRF: np.ndarray,
                       topJointAngle: float,
                       topOrientationRF: float) -> np.ndarray:
    return np.dot(vc.getRMFromAxis((0, 0, 1), topOrientationRF), np.dot(vc.getRMFromAxis((1, 0, 0), topJointAngle), -DRBottomContactCompDRF))


"""
    Include cable in the wire guide as part of the free body
"""


def evalTopCableGuideCompWithCable(tensionInRing: float,
                            topJointAngle: float,
                            topOrientationRF: float,
                            fricCoef: float) -> np.ndarray:
    halfJointAngle = topJointAngle/2
    tensionLoad = evalCapstan(tensionInRing, fricCoef, halfJointAngle)
    return np.array((
        tensionLoad*math.sin(halfJointAngle)*math.sin(topOrientationRF),
        -tensionLoad*math.sin(halfJointAngle)*math.cos(topOrientationRF),
        tensionLoad*math.cos(halfJointAngle)
    ))


def evalBottomCableGuideCompWithCable(tensionInRing: float,
                             bottomJointAngle: float,
                             fricCoef: float) -> np.ndarray:
    halfJointAngle = bottomJointAngle/2
    tensionHold = evalCapstan(tensionInRing, fricCoef, -halfJointAngle)
    return np.array((
        0,
        -tensionHold*math.sin(halfJointAngle),
        -tensionHold*math.cos(halfJointAngle)
    ))


"""
For visualization
"""
# def evalBottomContactComp(forceComponent:np.ndarray,
#                         bottomJointAngle:float,
#                         cableOrientationInRF: float,
#                         isTop:bool):
#     if not isTop:
#         a = np.dot(vc.getRMFromAxis((1,0,0), bottomJointAngle/2), forceComponent)
#     else:
#         a = np.dot(vc.getRMFromAxis((1,0,0), -bottomJointAngle/2), np.dot(vc.getRMFromAxis((0,0,1), -cableOrientationInRF),forceComponent))
#     return {"N": a[2], "Fa": a[0], "Fc": a[1]}

# def compute3dReactionMomentFromRFComponent(momentComponent: np.ndarray,
#                                             jointAngle:float,
#                                             cableOrientationInRF: float,
#                                             isTop:bool):
#     if  np.size(momentComponent,axis=0) == 2:
#         np.append(momentComponent, 0)

#     if not isTop:
#         a = np.dot(vc.getRMFromAxis((1,0,0), jointAngle/2), momentComponent)
#     else:
#         a = np.dot(vc.getRMFromAxis((1,0,0), -jointAngle/2), np.dot(vc.getRMFromAxis((0,0,1), -cableOrientationInRF),momentComponent))
#     return {"Nm": a[2], "Fcm": a[1]}
"""
    Instead of treating normal and frictional forces independently, they are combined and represeneted in force components and moment component respectively
"""
# def compute3dBottomContactingForceComponentInRF(N:float, Frx: float, Fry: float, jointAngle:float)-> np.ndarray:
#     halfJointAngle = jointAngle
#     return np.array((
#         Frx,
#         N*math.sin(halfJointAngle) + Fry*math.cos(halfJointAngle),
#         N*math.cos(halfJointAngle) - Fry*math.sin(halfJointAngle),
#     ))


# def compute3dContactingNormalForceComponentInRF(magnitude:float, jointAngle:float, isTop:bool, isAlternate:bool)-> np.ndarray:
#     if not isTop:
#         return np.array((
#             0,
#             magnitude*math.sin(jointAngle/2),
#             magnitude*math.cos(jointAngle/2)
#         ))
#     elif not isAlternate:
#         return np.array((
#             0,
#             magnitude*math.sin(jointAngle/2),
#             -magnitude*math.cos(jointAngle/2)
#         ))
#     return np.array((
#         -magnitude*math.sin(jointAngle/2),
#         0,
#         -magnitude*math.cos(jointAngle/2)
#     ))

# def contacting3dFrictionForceXAxisComponentInRF(magnitude:float, jointAngle:float, isTop:bool, isAlternate:bool)-> np.ndarray:
#     if not isTop:
#         return np.array((
#             magnitude,
#             0,
#             0,
#         ))
#     elif not isAlternate:
#         return np.array((
#             magnitude,
#             0,
#             0,
#         ))
#     return np.array((

#     ))


# def compute3dContactingNormalMomentComponet(magnitude:float, jointAngle:float, isTop:bool, isAlternate:bool)-> np.ndarray:
