import math

import numpy as np

from .. import vector_computation as vc, helper_functions as hf
from ..force_components import *


def test_compute3dCableBendingReactionComponentRingFrame():
    tensionLoad = 55
    tensionHold = 99
    jointBendingAngle = 2
    distalRingOrientationRingFrame = 3
    
    for isTop in (True, False):
        res = compute3dCableBendingReactionComponentRingFrame(tensionLoad=tensionLoad,
                                                              tensionHold=tensionHold,
                                                              jointBendingAngle=jointBendingAngle,
                                                              distalRingOrientationRingFrame=distalRingOrientationRingFrame,
                                                              isTop=isTop)
        if isTop:
            loadComponent = np.array((
                tensionLoad*math.sin(jointBendingAngle/2)*math.sin(distalRingOrientationRingFrame),
                -tensionLoad*math.sin(jointBendingAngle/2)*math.cos(distalRingOrientationRingFrame),
                tensionLoad*math.cos(jointBendingAngle/2)
            ))
            holdComponent = np.array((
                0,0,-tensionHold
            ))
        else:
            loadComponent = np.array((
                0,0,tensionLoad
            ))
            holdComponent = np.array((
                0,
                -tensionHold*math.sin(jointBendingAngle/2),
                -tensionHold*math.cos(jointBendingAngle/2)
            ))
        assert(all(res==(loadComponent + holdComponent)))
        
def test_compute3dKnobTensionComponentRingFrame():
    for t in range(10):
        t *=0.02
        res = compute3dKnobTensionComponentRingFrame(tension=t)
        assert(all(res==np.array((0,0,-t))))
        
def test_compute3dTopContactingComponentRingFrame():
    distalRingBottomReactionComponentDistalRingFrame = np.array((3.3,9.8,0.1))
    topJointBendingAngle = 3
    distalRingOrientationRingFrame = 1
    
    res = compute3dTopContactingComponentRingFrame(distalRingBottomReactionComponentDistalRingFrame=distalRingBottomReactionComponentDistalRingFrame,
                                                   topJointBendingAngle=topJointBendingAngle,
                                                   distalRingOrientationRingFrame=distalRingOrientationRingFrame)
    
    compare = np.dot(vc.getRMFromProximalToDistal(jointBendingAngle=topJointBendingAngle, distalRingOrientationRingFrame=distalRingOrientationRingFrame), distalRingBottomReactionComponentDistalRingFrame)
    assert(hf.allWithin(res,compare, threshold=0.00001))
    

def test_compute3dTopContactingComponentRingFrame():
    distalRingBottomReactionComponentDistalRingFrame = np.array((3.3,9.8,0.1))
    topJointBendingAngle = 3
    distalRingOrientationRingFrame = 1
    
    res = compute3dTopContactingComponentRingFrame(distalRingBottomReactionComponentDistalRingFrame=distalRingBottomReactionComponentDistalRingFrame,
                                                   topJointBendingAngle=topJointBendingAngle,
                                                   distalRingOrientationRingFrame=distalRingOrientationRingFrame)
    
    compare = np.dot(vc.getRMFromProximalToDistal(jointBendingAngle=topJointBendingAngle, distalRingOrientationRingFrame=distalRingOrientationRingFrame), distalRingBottomReactionComponentDistalRingFrame)
    assert(hf.allWithin(res,compare, threshold=0.00001))