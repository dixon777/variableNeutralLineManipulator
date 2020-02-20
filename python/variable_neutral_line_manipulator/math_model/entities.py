from typing import List
import math

import numpy as np

from ..common import *

# Knob length does not alter the force effect on the free body

class ErrorCollection():
    def __init__(self, errors:dict):
        self.errors = errors
        
    def __str__(self):
        return "\n".join([f"{k}: {e}" for k,e in self.errors.items()])

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
    def __init__(self, orientationBF: float, horizontalDistFromAxis: float):
        self.orientationBF = orientationBF
        self.horizontalDistFromAxis = horizontalDistFromAxis
        
    def __str__(self):
        return (f"{self.__class__.__name__}:"
                f"orientation base frame = {self.orientationBF}\n"
               f"horizontalDistFromAxis = {self.horizontalDistFromAxis}\n")


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
    
    def __str__(self):
        tmStr = "\n  ".join([str(tm) for tm in self.knobTendonModels])
        return (f"{self.__class__.__name__}:"
                f"length = {self.length}\n"
                f"orientation base frame = {self.orientationBF}\n"
               f"bottomCurveRadius = {self.bottomCurveRadius}\n"
               f"topCurveRadius = {self.topCurveRadius}\n"
               
               f"topOrientationRF = {self.topOrientationRF}\n"
               f"fricCoefRingTendon = {self.fricCoefRingTendon}\n"
               f"knobTensionModels:\n" + tmStr)

class SegmentModel():
    """
        Define each manipulator segments' parameters. Each segment has a set of tendon knobs attached to its end ring
        For N joints, the segment consists of: j1, r1, j2, r2, ..., j(n-1), r(n-1), j(n), r(n), where j and r are the annotations
        for joint and ring
        Should be passed into class Arm for composition of full arm
    """
    def __init__(self, is1DoF:bool=True,
                 numJoints:float=1,
                 ringLength:float=1.0,
                 orientationBF:float=0.0,
                 curveRadius: float=1.0,
                 tendonHorizontalDistFromAxis: float=0.5):
        
        d = {
            "is1DoF": is1DoF,
            "numJoints": numJoints,
            "ringLength": ringLength,
            "orientationBF": orientationBF,
            "curveRadius": curveRadius,
            "tendonHorizontalDistFromAxis": tendonHorizontalDistFromAxis
        }
        for k,v in d.items():
            self.__dict__[k] = v
            
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
                              ) for tendonOrientation in tendonOrientationsBF]
        
    def __repr__(self):
        return (f"{self.__class__.__name__}:"
                f"is 1 DoF = {self.is1DoF}\n"
               f"num joints = {self.numJoints}\n"
               f"ring length = {self.ringLength}\n"
               f"orientation base frame = {self.orientationBF}\n"
               f"Curve radius = {self.curveRadius}\n"
               f"Tendon horizontal dist from axis = {self.tendonHorizontalDistFromAxis}\n")


@Logger.hierarchy
def generateRingModels(segments: List[SegmentModel], fricCoefRingTendon=0.0) -> List[RingModel]:
    Logger.D(f"generateRingModels()")
    for i, s in enumerate(segments): 
        Logger.D(f"Model {i}: {s}")
        
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


class Validator():
    def __init__(self, streamFuncs):
        self.streamFuncs = streamFuncs
        self.errs = {}
        
    @staticmethod
    def streamCallUntilException(initVal, funcs):
        val = initVal
        for f in funcs:
            temp = f(val)
            if isinstance(temp, Exception):
                return val, temp
            val = temp
        return val, None
    
    def validate(self, key, val):
        funcs = self.streamFuncs.get(key)
        if funcs is None: # if the key is not defined
            raise KeyError("Not found")
        res, err = self.streamCallUntilException(val, funcs)
        self.errs[key] = err
        return res, err
    
    def isValid(self):
        for e in self.errs.values():
            if isinstance(e, Exception):
                return False
        return True
    
    @property
    def keys(self):
        return self.streamFuncs.keys()
    
    
    @property
    def errors(self):
        return ErrorCollection(self.errs)


class SegmentModelWithValidator(SegmentModel):
    def __init__(self, is1DoF=True, numJoints=1, ringLength=1.0, orientationBF=0.0, curveRadius=1.0, tendonHorizontalDistFromAxis=0.5):
        super().__init__(is1DoF=is1DoF, numJoints=numJoints, ringLength=ringLength, orientationBF=orientationBF, curveRadius=curveRadius, tendonHorizontalDistFromAxis=tendonHorizontalDistFromAxis)
        # For comparison between curveRadius and tendonHorizontalDistFromAxis
        def _compare(tendonHorizontalDistFromAxis,curveRadius):
            return tendonHorizontalDistFromAxis is None or curveRadius is None or tendonHorizontalDistFromAxis < curveRadius
        
        self.validator = Validator({
                "is1DoF": [lambda v: bool(v)],
                "numJoints": [
                    lambda v: int(v) if v is not None else AttributeError("Num joints must not be empty"),
                    lambda v: v if v >= 1 else AttributeError("Num joints must be greater or equal to 1")
                    ],
                "ringLength":[
                    lambda v: float(v) if v is not None else AttributeError("Ring Length must not be empty"),
                    lambda v: v if v > 0.0 else AttributeError("Ring length must be greater than 0")
                    ],
                "orientationBF":[
                    lambda v: float(v) if v is not None else AttributeError("Orientation must not be empty"),
                    ],
                "curveRadius":[
                    lambda v: float(v) if v is not None else AttributeError("Curve radius must not be empty"),
                    lambda v: v if v > 0.0 else AttributeError("Curve radius must be greater than 0"),
                    # lambda v: v if _compare(self.__dict__.get("tendonHorizontalDistFromAxis"), v) else AttributeError("Curve radius must be greater than tendon horizontal dist from axis")
                ],
                "tendonHorizontalDistFromAxis":[
                    lambda v: float(v) if v is not None else AttributeError("Tendon horizontal dist from axis must not be empty"),
                    lambda v: v if v > 0.0 else AttributeError("Tendon horizontal dist from axis must be greater than 0"),
                    lambda v: v if _compare(v, self.__dict__.get("curveRadius")) else AttributeError("Curve radius must be greater than tendon horizontal dist from axis")
                ],
            })
        
    def validate(self):
        for k in self.validator.keys:
            self.validator.validate(k, self.__dict__[k])
        return self.validator.errors
    
    def isValid(self, revalidate=True):
        if revalidate:
            self.validate()
        return self.validator.isValid()
        