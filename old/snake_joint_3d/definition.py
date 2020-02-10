from typing import List
import math

import numpy as np


class CableLocation():
    """
        Hold cable location details along the arm
        Current configuration assumes that it is identical through every ring
    """

    def __init__(self, orientationBF: float, horizontalDistFromAxis: float, knobLength: float):
        self.orientationBF = orientationBF
        self.horizontalDistFromAxis = horizontalDistFromAxis
        self.knobLength = knobLength


class SnakeJointRing():
    """
        Hold details of specific rings
        Warning: Should not be defined manually (but by calling SnakeJointArm.generateRings()) unless for testing or debugging
    """

    def __init__(self,
                 fricCoef: float,
                 length: float,
                 orientationBF: float,
                 bottomCurveRadius: float,
                 topCurveRadius: float = None,
                 topOrientationRF: float = None,
                 knobCableLocations: List[CableLocation] = []):
        self.length = float(length)
        self.bottomCurveRadius = float(bottomCurveRadius)
        self.topCurveRadius = None if topCurveRadius is None else float(
            topCurveRadius)
        self.fricCoef = float(fricCoef)
        self.orientationBF = float(orientationBF)
        self.topOrientationRF = None if topOrientationRF is None else float(
            topOrientationRF)
        self.knobCableLocations = knobCableLocations

    def numKnobs(self):
        return len(self.knobCableLocations)


class SnakeJointSegment():
    """
        Define each manipulator segments' parameters. Each segment has a set of cable knobs attached to its end ring
        For N joints, the segment consists of: j1, r1, j2, r2, ..., j(n-1), r(n-1), j(n), r(n), where j and r are the annotations
        for joint and ring
        Should be passed into SnakeJointArm for composition of full arm
    """

    def __init__(self, is1DoF: bool,
                 numJoints: float,
                 ringLength: float,
                 orientationBF: float,
                 cableHorizontalDistFromAxis: float,
                 knobLength: float,
                 curveRadius: float):
        self.is1DoF = is1DoF
        self.numJoints = int(numJoints)
        self.ringLength = float(ringLength)
        self.orientationBF = float(orientationBF)
        self.cableHorizontalDistFromAxis = float(cableHorizontalDistFromAxis)
        self.knobLength = float(knobLength)
        self.curveRadius = float(curveRadius)

    def getOrientationBF(self, i):
        return self.orientationBF + (0 if self.is1DoF or i % 2 == 0 else math.pi/2)

    def getRingLength(self, i):
        return self.ringLength if isinstance(self.ringLength, float) else self.ringLength[i]

    def getCurveRadius(self, i):
        return self.curveRadius if isinstance(self.curveRadius, float) else self.curveRadius[i]

    def getKnobCableLocations(self, i):
        if i+1 < self.numJoints:
            return []

        cableOrientationsBF = self.orientationBF + \
            np.array((-math.pi/2, math.pi/2)
                     if self.is1DoF else (-math.pi/2, 0, math.pi/2, math.pi))
        return [CableLocation(orientationBF=cableOrientation,
                              horizontalDistFromAxis=self.cableHorizontalDistFromAxis,
                              knobLength=self.knobLength,
                              ) for cableOrientation in cableOrientationsBF]

    def __eq__(self, obj):
        if not isinstance(obj, SnakeJointSegment):
            return False
        return self.is1DoF == obj.is1DoF and \
            self.ringLength == obj.ringLength and \
            self.curveRadius == obj.curveRadius
        # TODO: Fill the rest


