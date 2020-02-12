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

# Not required for model with tendons within tendon guide as part of free body
# def evalTopTendonGuideComp(tensionInRing: float,
#                           fricCoef: float,
#                           topJointAngle: float,
#                           topOrientationRF: float):
#     halfJointAngle = topJointAngle/2
#     tensionLoad = evalCapstan(tensionInRing, fricCoef, -halfJointAngle)
#     return np.array((
#         tensionLoad*math.sin(halfJointAngle)*math.sin(topOrientationRF),
#         -tensionLoad*math.sin(halfJointAngle)*math.cos(topOrientationRF),
#         tensionLoad*math.cos(halfJointAngle) - tensionInRing
#     ))


# def evalBottomTendonGuideComp(tensionInRing: float,
#                              fricCoef: float,
#                              bottomJointAngle: float) -> np.ndarray:
#     halfJointAngle = bottomJointAngle/2
#     tensionHold = evalCapstan(tensionInRing, fricCoef, halfJointAngle)
#     return np.array((
#         0,
#         -tensionHold*math.sin(halfJointAngle),
#         tensionInRing - tensionHold*math.cos(halfJointAngle)
#     ))


def evalKnobComp(tension: float):
    """
        Evaluate force component at a knob in ring's frame
    """
    return np.array((0, 0, -tension))


def changeContactCompFrame(DRBottomContactCompDRF: np.ndarray,
                       topJointAngle: float,
                       topOrientationRF: float) -> np.ndarray:
    """
        Change distal ring's bottom reaction in distal ring's frame to ring's top reaction in ring's frame
    """
    return np.dot(m4.create_from_axis_rotation((0, 0, 1), topOrientationRF), 
                  np.dot(m4.create_from_axis_rotation((1, 0, 0), topJointAngle), -DRBottomContactCompDRF))


def evalTopGuideCompWithTendon(tensionInRing: float,
                            topJointAngle: float,
                            topOrientationRF: float,
                            fricCoef: float) -> np.ndarray:
    """
        Evaluate force component at any top end of tendon guide
    """
    halfJointAngle = topJointAngle/2
    tensionLoad = evalCapstan(tensionInRing, fricCoef, halfJointAngle)
    return np.array((
        tensionLoad*math.sin(halfJointAngle)*math.sin(topOrientationRF),
        -tensionLoad*math.sin(halfJointAngle)*math.cos(topOrientationRF),
        tensionLoad*math.cos(halfJointAngle)
    ))


def evalBottomGuideCompWithTendon(tensionInRing: float,
                             bottomJointAngle: float,
                             fricCoef: float) -> np.ndarray:
    """
        Evaluate force component at any bottom end of tendon guide
    """
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
def getBottomNormalFrictionScalars(jointAngle:float,
                                   forceComp:np.ndarray=None,
                                   momentComp:np.ndarray=None):
    d = {}
    if forceComp:
        a = np.dot(m4.create_from_axis_rotation((1,0,0), jointAngle/2), forceComp)
        d += {"Fr": a[0], "Fc": a[1], "N": a[2]}
        
    if momentComp:
        a = np.dot(m4.create_from_axis_rotation((1,0,0), jointAngle/2), forceComp)
        d += {"Frm": a[0], "Fcm": a[1], "Nm": a[2]}
    return d
    
# def getTopNormalFrictionScalars(comp:np.ndarray,
#                                 jointAngle:float,
#                                 tendonOrientationRF: float):
#     a = np.dot(m4.create_from_axis_rotation((1,0,0), -jointAngle/2), np.dot(m4.create_from_axis_rotation((0,0,1), -tendonOrientationRF),comp))
#     return {"Fr": a[0], "Fc": a[1], "N": a[2]}

# def compute3dReactionMomentFromRFComponent(momentComponent: np.ndarray,
#                                             jointAngle:float,
#                                             tendonOrientationInRF: float,
#                                             isTop:bool):
#     if  np.size(momentComponent,axis=0) == 2:
#         np.append(momentComponent, 0)

#     if not isTop:
#         a = np.dot(m4.create_from_axis_rotation((1,0,0), jointAngle/2), momentComponent)
#     else:
#         a = np.dot(m4.create_from_axis_rotation((1,0,0), -jointAngle/2), np.dot(m4.create_from_axis_rotation((0,0,1), -tendonOrientationInRF),momentComponent))
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
