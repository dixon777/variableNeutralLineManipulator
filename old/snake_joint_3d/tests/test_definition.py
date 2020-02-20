import pytest
import math
import numpy as np
import random as rd

from snake_joint_3d.definition import *
from snake_joint_3d.force_components import *

@pytest.fixture
def segment2DoF():
    return SnakeJointSegment(is1DoF=False, 
                 numJoints=5,
                 ringLength=3, 
                 orientationBaseFrame=1, 
                 distanceFromAxis=2.5, 
                 knobLength=2,
                 curvatureRadius=4)
 
@pytest.fixture
def segment1DoF():
    return SnakeJointSegment(is1DoF=True, 
                 numJoints=2,
                 ringLength=2, 
                 orientationBaseFrame=1, 
                 distanceFromAxis=2, 
                 knobLength=2,
                 curvatureRadius=3)    

@pytest.fixture
def arm21(segment1DoF, segment2DoF):
    return SnakeJointArm(frictionCoefficient=0.3, segments=[segment2DoF, segment1DoF])
  
@pytest.fixture  
def ring2DoF():
    return SnakeJointRing(
            frictionCoefficient=0.3,
            length=5,
            orientationBaseFrame=1/3,
            bottomCurvatureRadius=3,
            topCurvatureRadius=5,
            topOrientationRingFrame=2,
            knobCableLocations=[ 
                CableLocation(orientationBaseFrame=orientation, distanceFromAxis=2,knobLength=3) for orientation in np.array((-math.pi/2, 0, math.pi/2, math.pi)) + 1/3
            ])
    
def test_segment1DoF(segment1DoF: SnakeJointSegment):
    segment = segment1DoF
    assert(segment.is1DoF)
    for i in range(segment.numJoints):
        assert(segment.getRingLength(i) == 2)
        assert(segment.getOrientationBaseFrame(i) == segment.orientationBaseFrame)
        assert(segment.getCurvatureRadius(i) == 3)
        if i+1 < segment.numJoints:
            assert(segment.getCableLocations(i) == [])
        else:
            cableLocations = segment.getCableLocations(i)
            assert(len(cableLocations)==2)
            cableRelativeOrientation = (-math.pi/2, math.pi/2)
            for i, cl in enumerate(cableLocations):
                assert(cl.orientationBaseFrame == segment.orientationBaseFrame + cableRelativeOrientation[i])
                assert(cl.distanceFromAxis == 2)
                assert(cl.knobLength == 2)      
    
def test_segment2DoF(segment2DoF: SnakeJointSegment):
    segment = segment2DoF
    assert(not segment.is1DoF)
    for i in range(segment.numJoints):
        assert(segment.getRingLength(i) == 3)
        assert(segment.getOrientationBaseFrame(i) == segment.orientationBaseFrame + (0 if i%2 ==0 else math.pi/2))
        assert(segment.getCurvatureRadius(i) == 4)
        if i+1 < segment.numJoints:
            assert(segment.getCableLocations(i) == [])
        else:
            cableLocations = segment.getCableLocations(i)
            assert(len(cableLocations)==4)
            cableRelativeOrientation = (-math.pi/2, 0, math.pi/2, math.pi)
            for i, cl in enumerate(cableLocations):
                assert(cl.orientationBaseFrame == segment.orientationBaseFrame + cableRelativeOrientation[i])
                assert(cl.distanceFromAxis == 2.5)
                assert(cl.knobLength == 2)


def test_ArmAttributes(segment1DoF, segment2DoF, arm21):
    assert(arm21.frictionCoefficient == 0.3)
    assert(len(arm21.segments)==2)
    assert(segment2DoF == arm21.segments[0])
    assert(segment1DoF == arm21.segments[1])

def test_cableState():
    ts = [rd.random()*20 for _ in range(10)]
    ds = [rd.random()*20 for _ in range(10)]
    os = [rd.random()*20 for _ in range(10)]
    for t, d, o in zip(ts, ds,os):
        cl = CableLocation(orientationBaseFrame=o, distanceFromAxis=d,knobLength=4)
        state = CableState(cableLocation=cl, tensionInRing=t, isKnob=False)
        assert(state.cableLocation == cl)
        assert(state.tensionInRing == t)
        assert(not state.isKnob)
    
def test_ringGenerateCableStates(ring2DoF):
    knobTensions = [rd.random()*5 for _ in range(len(ring2DoF.knobCableLocations))]
    cableStates = ring2DoF.getKnobCableState(knobTensions=knobTensions)
    for kt, cl, cs in zip(knobTensions, ring2DoF.knobCableLocations, cableStates):
        assert(cs.cableLocation==cl)
        assert(cs.tensionInRing == kt)
        assert(cs.isKnob)
        
def test_toProximalState():
    ts = [rd.random()*20 for _ in range(10)]
    ds = [rd.random()*20 for _ in range(10)]
    os = [rd.random()*20 for _ in range(10)]
    for t, d, o in zip(ts, ds,os):
        cl = CableLocation(orientationBaseFrame=o, distanceFromAxis=d,knobLength=4)
        cs = CableState(cableLocation=cl, tensionInRing=t, isKnob=True).toProximalRing(frictionCoefficient=0.4, jointBendingAngle=5)
        assert(cs.tensionInRing == tensionLoadToHold(tensionLoad=t, frictionCoefficient=0.4, bendingAngle=5))
        assert(not cs.isKnob)
    


    
    
def test_generateRings(segment1DoF, segment2DoF, arm21):
    rings = arm21.generateRings()
    assert(len(rings) == segment1DoF.numJoints + segment2DoF.numJoints)
    
    for (i,r) in enumerate(rings):
        if i < segment2DoF.numJoints:
            assert(r.length == segment2DoF.getRingLength(i))
            assert(r.bottomCurvatureRadius == segment2DoF.getCurvatureRadius(i))
            assert(r.frictionCoefficient == arm21.frictionCoefficient)
            assert(r.orientationBaseFrame == segment2DoF.getOrientationBaseFrame(i))
            if i+1 == segment2DoF.numJoints:
                assert(r.topOrientationRingFrame == (segment1DoF.getOrientationBaseFrame(0) - segment2DoF.getOrientationBaseFrame(i)))
                assert(r.topCurvatureRadius == segment1DoF.getCurvatureRadius(0))
                cableRelativeOrientation = (-math.pi/2, 0, math.pi/2, math.pi)
                for (i, cl) in enumerate(r.knobCableLocations):
                    assert(cl.orientationBaseFrame == segment2DoF.orientationBaseFrame + cableRelativeOrientation[i])
                    assert(cl.distanceFromAxis == segment2DoF.distanceFromAxis)
                    assert(cl.knobLength == segment2DoF.knobLength) 
            else:
                assert(r.topOrientationRingFrame == (segment2DoF.getOrientationBaseFrame(i+1) - segment2DoF.getOrientationBaseFrame(i)))
                assert(r.topCurvatureRadius == segment2DoF.getCurvatureRadius(i+1))
                assert(r.knobCableLocations == [])
        
        else:
            i -= segment2DoF.numJoints
            assert(r.length == segment1DoF.getRingLength(i))
            assert(r.bottomCurvatureRadius == segment1DoF.getCurvatureRadius(i))
            assert(r.frictionCoefficient == arm21.frictionCoefficient)
            assert(r.orientationBaseFrame == segment1DoF.getOrientationBaseFrame(i))
            if i+1 == segment1DoF.numJoints:
                assert(r.topOrientationRingFrame == None)
                assert(r.topCurvatureRadius == None)
                cableRelativeOrientation = (-math.pi/2, math.pi/2)
                for (i, cl) in enumerate(r.knobCableLocations):
                    assert(cl.orientationBaseFrame == segment1DoF.orientationBaseFrame + cableRelativeOrientation[i])
                    assert(cl.distanceFromAxis == segment1DoF.distanceFromAxis)
                    assert(cl.knobLength == segment1DoF.knobLength) 
            else:
                assert(r.topOrientationRingFrame == (segment1DoF.getOrientationBaseFrame(i+1) - segment1DoF.getOrientationBaseFrame(i)))
                assert(r.topCurvatureRadius == segment1DoF.getCurvatureRadius(i+1))
                assert(r.knobCableLocations == [])
        
            