from uuid import uuid4 
from copy import deepcopy

from ..math_model.entities import *
from ..math_model.states import *
from ..math_model.solvers import *

from ..common import Singleton, Logger

class Repo(metaclass=Singleton):
    def __init__(self):
        super().__init__()
        self._segmentModels = {} # Segment
        self._ringModels = []
        self._knobTensionLists = []
        
    def addSegment(self):
        Logger.D(f"Add segment")
        key = uuid4()
        self._segmentModels[key] = SegmentModel()
        return (key, self._segmentModels[key])
    
    def removeSegment(self, key):
        Logger.D(f"Remove segment: {key}")
        del self._segmentModels[key]
        return key
        
    def updateSegment(self, keySegmentPair):
        Logger.D(f"Update segment: {keySegmentPair}")
        k, v = keySegmentPair
        self._segmentModels[k] = v
        return self._segmentModels[k].validate()
        
    def generateSegments(self, _):
        Logger.D("Generate segments")
        for v in self._segmentModels.values():
            if not v.isValid():
                return  Exception("Some segments' param are not valid")
            
        self._knobTensionLists.clear()
        self._ringModels = generateRingModels(list(self._segmentModels.values()))
        knobTendonModelCompositeList = []
        for r in self._ringModels:
            if r.knobTendonModels:
                knobTendonModelCompositeList.append((r, r.knobTendonModels))
                self._knobTensionLists.append([0.0,]*len(r.knobTendonModels))
        return knobTendonModelCompositeList
    
    def updateTensions(self, indicesValuePair):
        indices, value = indicesValuePair
        assert(isinstance(indices, tuple))
        assert(isinstance(value, float))
        Logger.D(f"Update tensions")
        self._knobTensionLists[indices[0]][indices[1]] = value
    
    def computeTensions(self):
        Logger.D("Compute tensions")
        s = computeFromEndTensions(self._ringModels, self._knobTensionLists)
        return s