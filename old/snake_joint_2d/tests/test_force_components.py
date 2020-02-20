import math

import snake_joint_2d.force_components as hf

def test_tensionHoldToLoad():
    res = hf.tensionHoldToLoad(tensionHold=15, frictionCoefficient=0.1, bendingAngle=math.pi/4)
    assert(round(res,5) == 13.86698)
    res = hf.tensionHoldToLoad(tensionHold=30, frictionCoefficient=0.3, bendingAngle=math.pi*3/2)
    assert(round(res,5) == 7.29713)
    
def test_tensionLoadToHold():
    res = hf.tensionLoadToHold(tensionLoad=15, frictionCoefficient=0.1, bendingAngle=math.pi/4)
    assert(round(res,5) == 16.22560)
    res = hf.tensionLoadToHold(tensionLoad=30, frictionCoefficient=0.3, bendingAngle=math.pi*3/2)
    assert(round(res,5) == 123.33621)
    
def test_cableBendingReactionComponent():
    tensionLoad = 10
    tensionHold = 15
    jointBendingAngle = math.pi
    res = hf.cableBendingReactionComponent(tensionLoad=tensionLoad, tensionHold=tensionHold, isTop = True, jointBendingAngle=jointBendingAngle)
    assert(all(res==[-tensionLoad*math.sin(jointBendingAngle/2), tensionLoad*math.cos(jointBendingAngle/2) - tensionHold]))
    res = hf.cableBendingReactionComponent(tensionLoad=tensionLoad, tensionHold=tensionHold, isTop = False, jointBendingAngle=jointBendingAngle)
    assert(all(res==[-tensionHold*math.sin(jointBendingAngle/2), tensionLoad - tensionHold*math.cos(jointBendingAngle/2)]))

def test_endPieceTensionComponent():
    res = hf.endPieceTensionComponent(tension=10)
    assert(all(res==(0,-10)))


def test_contactingNormalForceComponent():
    jointBendingAngle = math.pi/3    
    magnitude = 30
    res = hf.contactingNormalForceComponent(jointBendingAngle=jointBendingAngle, isTop=True, magnitude=magnitude)
    assert(all(res==(magnitude*math.sin(jointBendingAngle/2), -magnitude*math.cos(jointBendingAngle/2))))
    res = hf.contactingNormalForceComponent(jointBendingAngle=jointBendingAngle, isTop=False, magnitude=magnitude)
    assert(all(res==(magnitude*math.sin(jointBendingAngle/2), magnitude*math.cos(jointBendingAngle/2))))
    
def test_contactingFrictionComponent():
    jointBendingAngle = math.pi/3    
    magnitude = 30
    res = hf.contactingFrictionComponent(jointBendingAngle=jointBendingAngle, isTop=True, magnitude=magnitude)
    assert(all(res==(magnitude*math.cos(jointBendingAngle/2), magnitude*math.sin(jointBendingAngle/2))))
    res = hf.contactingFrictionComponent(jointBendingAngle=jointBendingAngle, isTop=False, magnitude=magnitude)
    assert(all(res==(-magnitude*math.cos(jointBendingAngle/2), magnitude*math.sin(jointBendingAngle/2))))
  