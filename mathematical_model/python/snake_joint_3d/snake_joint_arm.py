from .states import *
from .solvers import *

class SnakeJointArm():
    def __init__(self,
                 fricCoef: float,
                 segments: List[SnakeJointSegment]):
        self.fricCoef = float(fricCoef)
        self.segments = segments
        self.cacheRings = None
        
        

    def generateRings(self) -> List[SnakeJointRing]:
        if self.cacheRings:
            return cache
        rings = []
        for segmentIndex, segment in enumerate(self.segments):
            for jointIndex in range(segment.numJoints):
                rings.append(SnakeJointRing(
                    fricCoef=self.fricCoef,
                    length=segment.getRingLength(jointIndex),
                    orientationBF=segment.getOrientationBF(jointIndex),
                    bottomCurveRadius=segment.getCurveRadius(jointIndex),
                    topCurveRadius=segment.getCurveRadius(jointIndex+1) if jointIndex+1 < segment.numJoints else
                    self.segments[segmentIndex+1].getCurveRadius(0) if segmentIndex+1 < len(self.segments) else
                    None,
                    topOrientationRF=segment.getOrientationBF(jointIndex+1) - segment.getOrientationBF(jointIndex) if jointIndex+1 < segment.numJoints else
                    self.segments[segmentIndex+1].getOrientationBF(0) - segment.getOrientationBF(jointIndex) if segmentIndex+1 < len(self.segments) else
                    None,
                    knobCableLocations=segment.getKnobCableLocations(jointIndex),
                ))
        self.cacheRings = [r for r in rings]
        return rings

    def computeFromEndTensions(self, endTensions: List[List[float]]) -> SnakeJointResult:
        tensionIndex = -1
        lastState = None
        for r in reversed(self.generateRings()):
            if r.numKnobs():
                endTension = endTensions[tensionIndex]
                tensionIndex -= 1
            else:
                endTension = None
            func = defineBottomJointAngleFunc(r, lastState, endTension)
            try:
                checkBounds(func, evalBottomAngleBound(r, evalRingCableStates(r, lastState, endTension)))
            except Exception as error:
                return SnakeJointResult(arm=self, state=lastState)
            res, lastState = NewtonSolver(func, lastState.bottomJointBendingAngle if lastState else 0.0)
        
        return SnakeJointResult(arm=self, state=lastState)
            
            