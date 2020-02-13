from typing import List
import math

import numpy as np


class TendonModel():
    """
        Hold tendon location details along the arm
        Current configuration assumes that the displacement between tendon and axis is constant throughout the whole manipulator
        Such configuration is for simplicity
        But in fact, it can be configured in any  physically compatible way.
        
        orientationBF = Orientation in base frame
        horizontalDistFromAxis = horizontal dist from axis
        knob length = knob length from the base frame
    """
    def __init__(self, orientationBF: float, horizontalDistFromAxis: float, knobLength: float):
        self.orientationBF = orientationBF
        self.horizontalDistFromAxis = horizontalDistFromAxis
        self.knobLength = knobLength


class RingModel():
    """
        Hold details of specific rings
        Warning: Should not be defined manually (but by calling Arm.generateRings()) unless for testing or debugging
    """
    def __init__(self,
                 length: float,
                 orientationBF: float,
                 bottomCurveRadius: float = None,
                 topCurveRadius: float = None,
                 topOrientationRF: float = None,
                 knobTendonModels: List[TendonModel] = [],
                 fricCoefRingTendon: float = 0.0):
        self.length = float(length)
        self.bottomCurveRadius = None if bottomCurveRadius is None else float(bottomCurveRadius)
        self.topCurveRadius = None if topCurveRadius is None else float(topCurveRadius)
        self.orientationBF = float(orientationBF)
        self.topOrientationRF =  None if topOrientationRF is None else float(topOrientationRF)
        self.knobTendonModels = knobTendonModels
        self.fricCoefRingTendon = float(fricCoefRingTendon)

    def numKnobs(self):
        return len(self.knobTendonModels)


class SegmentModel():
    """
        Define each manipulator segments' parameters. Each segment has a set of tendon knobs attached to its end ring
        For N joints, the segment consists of: j1, r1, j2, r2, ..., j(n-1), r(n-1), j(n), r(n), where j and r are the annotations
        for joint and ring
        Should be passed into class Arm for composition of full arm
    """
    def __init__(self, is1DoF: bool,
                 numJoints: float,
                 ringLength: float,
                 orientationBF: float,
                 tendonHorizontalDistFromAxis: float,
                 knobLength: float,
                 curveRadius: float):
        self.is1DoF = is1DoF
        self.numJoints = int(numJoints)
        self.ringLength = float(ringLength)
        self.orientationBF = float(orientationBF)
        self.tendonHorizontalDistFromAxis = float(tendonHorizontalDistFromAxis)
        self.knobLength = float(knobLength)
        self.curveRadius = float(curveRadius)

    def getOrientationBF(self, i):
        return self.orientationBF + (0 if self.is1DoF or i % 2 == 0 else math.pi/2)

    def getRingLength(self, i):
        return self.ringLength if isinstance(self.ringLength, float) else self.ringLength[i]

    def getCurveRadius(self, i):
        return self.curveRadius if isinstance(self.curveRadius, float) else self.curveRadius[i]

    def getKnobTendonLocations(self, i):
        if i+1 < self.numJoints:
            return []

        tendonOrientationsBF = self.orientationBF + \
            np.array((-math.pi/2, math.pi/2)
                     if self.is1DoF else (-math.pi/2, 0, math.pi/2, math.pi))
        return [TendonModel(orientationBF=tendonOrientation,
                              horizontalDistFromAxis=self.tendonHorizontalDistFromAxis,
                              knobLength=self.knobLength,
                              ) for tendonOrientation in tendonOrientationsBF]


def generateRingModels(segments: List[SegmentModel], fricCoefRingTendon=0.0) -> List[RingModel]:
    rings = []
    for segmentIndex, segment in enumerate(segments):
        for jointIndex in range(segment.numJoints):
            rings.append(RingModel(
                fricCoefRingTendon=fricCoefRingTendon,
                length=segment.getRingLength(jointIndex),
                orientationBF=segment.getOrientationBF(jointIndex),
                bottomCurveRadius=segment.getCurveRadius(jointIndex),
                topCurveRadius=segment.getCurveRadius(jointIndex+1) if jointIndex+1 < segment.numJoints else
                segments[segmentIndex+1].getCurveRadius(0) if segmentIndex+1 < len(segments) else
                None,
                topOrientationRF=segment.getOrientationBF(jointIndex+1) - segment.getOrientationBF(jointIndex) if jointIndex+1 < segment.numJoints else
                segments[segmentIndex+1].getOrientationBF(0) - segment.getOrientationBF(jointIndex) if segmentIndex+1 < len(segments) else
                None,
                knobTendonModels=segment.getKnobTendonLocations(jointIndex),
            ))
    return rings



            
            