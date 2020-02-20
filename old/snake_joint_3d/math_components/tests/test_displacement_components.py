import math, random as rd

import numpy as np
import pytest

from ..displacement_components import *


def test_evalDispBetweenCenters():
    curveRadius = 5
    ringLength = 4
    res = evalDispBetweenCenters(curveRadius, ringLength, isTop=True)
    assert(res == ringLength/2-curveRadius)
    res = evalDispBetweenCenters(curveRadius, ringLength, isTop=False)
    assert(res == curveRadius-ringLength/2)

# # def test_evalHorizontalDispFromCableToAxis():
#     # curvatureRadius = 5
#     # curvatureSpanAngle = math.pi/4
#     # res = evalHorizontalDispFromCableToAxis(curvatureRadius=curvatureRadius, 
#     #                                         curvatureSpanAngle=curvatureSpanAngle)
#     # assert(res == curvatureRadius*math.sin(curvatureSpanAngle/2))
    
    
# def test_evalKnobDisp():
#     ringLength = 5
#     knobLength = 3
#     horizontalDispFromCableToAxis = 1
#     cableOrientationInRingFrame = 2
#     res = evalKnobDisp(ringLength=ringLength, 
#                                          knobLength=knobLength,
#                                          horizontalDispFromCableToAxis=horizontalDispFromCableToAxis,
#                                          cableOrientationInRingFrame=cableOrientationInRingFrame)
#     assert(all(res==np.array((
#         horizontalDispFromCableToAxis*math.cos(cableOrientationInRingFrame),
#         horizontalDispFromCableToAxis*math.sin(cableOrientationInRingFrame),
#         knobLength - ringLength/2
#     ))))
    
# def test_compute3dCableGuideEndDisplacementRingFrame():
#     ringLength = 5
#     curvatureRadius = 3
#     horizontalDispFromCableToAxis = 1
#     cableOrientationInRingFrame = 2
#     distalRingOrientationInRingFrame = 3
#     for isTop in (True, False):
#         res = compute3dCableGuideEndDisplacementRingFrame(ringLength=ringLength, 
#                                         curvatureRadius=curvatureRadius,
#                                         horizontalDispFromCableToAxis=horizontalDispFromCableToAxis,
#                                         cableOrientationInRingFrame=cableOrientationInRingFrame,
#                                         isTop=isTop,
#                                         distalRingOrientationInRingFrame=distalRingOrientationInRingFrame)
        
#         x = horizontalDispFromCableToAxis*math.cos(cableOrientationInRingFrame)
#         y = horizontalDispFromCableToAxis*math.sin(cableOrientationInRingFrame)   
#         if isTop:
#             z = math.sqrt(curvatureRadius**2 - (horizontalDispFromCableToAxis*math.sin(cableOrientationInRingFrame - distalRingOrientationInRingFrame))**2)
#         else:
#             z = -math.sqrt(curvatureRadius**2 - (horizontalDispFromCableToAxis*math.sin(cableOrientationInRingFrame))**2)
#         assert(all(res==np.array((
#             x,
#             y,
#             z + evalDispBetweenCenters(ringLength=ringLength, curvatureRadius=curvatureRadius, isTop=isTop)
#         ))))


def test_evalTopContactDisp():
    ringLength = rd.random()*100
    curveRadius = rd.random()*100
    jointAngle = math.pi*2*rd.random()
    topOrientationRF = math.pi/3*rd.random()
    
    res = evalTopContactDisp(ringLength=ringLength,
                            topCurveRadius=curveRadius,
                            topJointAngle=jointAngle,
                            topOrientationRF=topOrientationRF)
    
    assert(all(res==np.array((
        curveRadius*math.sin(jointAngle/2)*math.sin(topOrientationRF),
        -curveRadius*math.sin(jointAngle/2)*math.cos(topOrientationRF),
        curveRadius*math.cos(jointAngle/2) + evalDispBetweenCenters(curveRadius, ringLength, True)
    ))))
    
            
# def test_compute3dContactingPointDisplacementRingFrame():
#     ringLength = 5
#     curvatureRadius = 6
#     jointBendingAngle = math.pi*2/3
#     distalRingOrientationInRingFrame = 6
#     for isTop in (True, False):
            
#         res = compute3dContactingPointDisplacementRingFrame(ringLength=ringLength,
#                                             curvatureRadius=curvatureRadius,
#                                             jointBendingAngle=jointBendingAngle,
#                                             distalRingOrientationInRingFrame=distalRingOrientationInRingFrame,
#                                             isTop=isTop)
        
#         if isTop:
#             x = curvatureRadius*math.sin(jointBendingAngle/2)*math.sin(distalRingOrientationInRingFrame)
#             y = -curvatureRadius*math.sin(jointBendingAngle/2)*math.cos(distalRingOrientationInRingFrame)
#             z = curvatureRadius*math.cos(jointBendingAngle/2)
#         else:
#             x = 0
#             y = -curvatureRadius*math.sin(jointBendingAngle/2)
#             z = -curvatureRadius*math.cos(jointBendingAngle/2)
#         assert(all(res==np.array((
#             x,
#             y,
#             z + evalDispBetweenCenters(ringLength=ringLength, curvatureRadius=curvatureRadius, isTop=isTop)
#         ))))