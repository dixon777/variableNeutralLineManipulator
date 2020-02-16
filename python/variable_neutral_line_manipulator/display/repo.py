from uuid import uuid4 
from copy import deepcopy

from ..math_model.entities import *
from ..math_model.states import *
from ..math_model.solvers import *

from .helper import Singleton

class Repo(metaclass=Singleton):
    def __init__(self):
        super().__init__()
        self._segmentModels = {} # Segment
        self._ringModels = []
        self._knobTensionLists = []
        
    def addSegment(self):
        print(f"Add segment")
        key = uuid4()
        self._segmentModels[key] = SegmentModel(True, 1, 1, 0.0, 1.0, 0.1, 1.0)
        return (key, self._segmentModels[key])
    
    def removeSegment(self, key):
        print(f"Remove segment: {key}")
        del self._segmentModels[key]
        return key
        
    def updateSegment(self, keySegmentPair):
        print(f"Update segment: {keySegmentPair}")
        k, v = keySegmentPair
        self._segmentModels[k] = v
        
    def generateSegments(self, _):
        print("Generate segments")
        self._knobTensionLists.clear()
        self._ringModels = generateRingModels(list(self._segmentModels.values()))
        knobTendonModelCompositeList = []
        for r in self._ringModels:
            if r.knobTendonModels:
                knobTendonModelCompositeList.append((r, r.knobTendonModels))
                self._knobTensionLists.append([0,]*len(r.knobTendonModels))
        return knobTendonModelCompositeList
    
    def updateTensions(self, indicesValuePair):
        indices, value = indicesValuePair
        assert(isinstance(indices, tuple))
        assert(isinstance(value, float))
        print(f"Update tensions")
        self._knobTensionLists[indices[0]][indices[1]] = value
    
    def computeTensions(self):
        print("Compute tensions")
        s = computeFromEndTensions(self._ringModels, self._knobTensionLists)
        return s