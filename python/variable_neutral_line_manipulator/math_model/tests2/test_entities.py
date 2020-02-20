import pytest
import math
import numpy as np
import random as rd

from ..entities import *

@pytest.fixture
def segment2DoF():
    return Segment(is1DoF=False, 
                 numJoints=5,
                 ringLength=3, 
                 orientationBF=1, 
                 cableHorizontalDistFromAxis=2.5, 
                 knobLength=2,
                 curveRadius=4)
 
@pytest.fixture
def segment1DoF():
    return Segment(is1DoF=True, 
                 numJoints=2,
                 ringLength=2, 
                 orientationBF=1, 
                 cableHorizontalDistFromAxis=2, 
                 knobLength=2,
                 curveRadius=3)    

  
@pytest.fixture  
def ring2DoF():
    return Ring(
            fricCoefRingCable=0.3,
            length=5,
            orientationBF=1/3,
            bottomCurveRadius=3,
            topCurveRadius=5,
            topOrientationRF=2,
            knobCableLocations=[ 
                CableLocation(orientationBF=orientation, horizontalDistFromAxis=2,knobLength=3) for orientation in np.array((-math.pi/2, 0, math.pi/2, math.pi)) + 1/3
            ])
    
def test_segment1DoF(segment1DoF: Segment):
    segment = segment1DoF
    assert(segment.is1DoF)
    for i in range(segment.numJoints):
        assert(segment.getRingLength(i) == 2)
        assert(segment.getOrientationBF(i) == segment.orientationBF)
        assert(segment.getCurveRadius(i) == 3)
        if i+1 < segment.numJoints:
            assert(segment.getKnobCableLocations(i) == [])
        else:
            cableLocations = segment.getKnobCableLocations(i)
            assert(len(cableLocations)==2)
            cableRelativeOrientation = (-math.pi/2, math.pi/2)
            for i, cl in enumerate(cableLocations):
                assert(cl.orientationBF == segment.orientationBF + cableRelativeOrientation[i])
                assert(cl.horizontalDistFromAxis == 2)
                assert(cl.knobLength == 2)      
    
def test_segment2DoF(segment2DoF: Segment):
    segment = segment2DoF
    assert(not segment.is1DoF)
    for i in range(segment.numJoints):
        assert(segment.getRingLength(i) == 3)
        assert(segment.getOrientationBF(i) == segment.orientationBF + (0 if i%2 ==0 else math.pi/2))
        assert(segment.getCurveRadius(i) == 4)
        if i+1 < segment.numJoints:
            assert(segment.getKnobCableLocations(i) == [])
        else:
            cableLocations = segment.getKnobCableLocations(i)
            assert(len(cableLocations)==4)
            cableRelativeOrientation = (-math.pi/2, 0, math.pi/2, math.pi)
            for i, cl in enumerate(cableLocations):
                assert(cl.orientationBF == segment.orientationBF + cableRelativeOrientation[i])
                assert(cl.horizontalDistFromAxis == 2.5)
                assert(cl.knobLength == 2)




    
    
def test_generateRings(segment1DoF, segment2DoF):
    fricCoefRingCable = 0.3
    rings = generateRings([segment2DoF, segment1DoF], fricCoefRingCable)
    assert(len(rings) == segment1DoF.numJoints + segment2DoF.numJoints)
    
    for (i,r) in enumerate(rings):
        if i < segment2DoF.numJoints:
            assert(r.length == segment2DoF.getRingLength(i))
            assert(r.bottomCurveRadius == segment2DoF.getCurveRadius(i))
            assert(r.fricCoefRingCable == fricCoefRingCable)
            assert(r.orientationBF == segment2DoF.getOrientationBF(i))
            if i+1 == segment2DoF.numJoints:
                assert(r.topOrientationRF == (segment1DoF.getOrientationBF(0) - segment2DoF.getOrientationBF(i)))
                assert(r.topCurveRadius == segment1DoF.getCurveRadius(0))
                cableRelativeOrientation = (-math.pi/2, 0, math.pi/2, math.pi)
                for (i, cl) in enumerate(r.knobCableLocations):
                    assert(cl.orientationBF == segment2DoF.orientationBF + cableRelativeOrientation[i])
                    assert(cl.horizontalDistFromAxis == segment2DoF.cableHorizontalDistFromAxis)
                    assert(cl.knobLength == segment2DoF.knobLength) 
            else:
                assert(r.topOrientationRF == (segment2DoF.getOrientationBF(i+1) - segment2DoF.getOrientationBF(i)))
                assert(r.topCurveRadius == segment2DoF.getCurveRadius(i+1))
                assert(r.knobCableLocations == [])
        
        else:
            i -= segment2DoF.numJoints
            assert(r.length == segment1DoF.getRingLength(i))
            assert(r.bottomCurveRadius == segment1DoF.getCurveRadius(i))
            assert(r.fricCoefRingCable == fricCoefRingCable)
            assert(r.orientationBF == segment1DoF.getOrientationBF(i))
            if i+1 == segment1DoF.numJoints:
                assert(r.topOrientationRF == None)
                assert(r.topCurveRadius == None)
                cableRelativeOrientation = (-math.pi/2, math.pi/2)
                for (i, cl) in enumerate(r.knobCableLocations):
                    assert(cl.orientationBF == segment1DoF.orientationBF + cableRelativeOrientation[i])
                    assert(cl.horizontalDistFromAxis == segment1DoF.cableHorizontalDistFromAxis)
                    assert(cl.knobLength == segment1DoF.knobLength) 
            else:
                assert(r.topOrientationRF == (segment1DoF.getOrientationBF(i+1) - segment1DoF.getOrientationBF(i)))
                assert(r.topCurveRadius == segment1DoF.getCurveRadius(i+1))
                assert(r.knobCableLocations == [])
        
            