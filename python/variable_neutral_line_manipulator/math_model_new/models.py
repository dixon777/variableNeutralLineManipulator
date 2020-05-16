from typing import List
from math import pi
import numpy as np

from .calculation import *
from .vec import *


class TransitionMathConfig:
    """
        Geometry definition of transition (before initial segment, after last segment and between consecutive segments)
    """
    def __init__(self, diskLength):
        self.diskLength = diskLength

class SegmentMathConfig:
    """
        Geometry definition of segment
    """
    def __init__(self, is2DoF, nJoints, diskLength, orientationBF, curveRadius, tendonDistFromAxis):
        self.nJoints = nJoints
        self.is2DoF = is2DoF
        self.diskLength = diskLength
        self.orientationBF = orientationBF
        self.curveRadius = curveRadius
        self.tendonDistFromAxis = tendonDistFromAxis
        
class TendonMathModel:
    """
        GetensionInDiskon
    """
    def __init__(self, orientationBF, distFromAxis, nJoints):
        self.orientationBF = orientationBF
        self.distFromAxis = distFromAxis
        self.nJoints = nJoints
        
    def __repr__(self):
        return f"<TendonMathModel>\no = {self.orientationBF}\distFromAxis = {self.distFromAxis}\nnJoints = {self.nJoints}\n"

        return self.tendon_generic_model.distFromAxis
    
class DiskMathModel:
    """
        Geometry of disk 
    """
    def __init__(self, outerDiameter, length, bottomOrientationBF=None, bottomCurveRadius=None, topOrientationBF=None, topCurveRadius=None):
        self.outerDiameter = outerDiameter
        self.length = length
        self.bottomOrientationBF = bottomOrientationBF
        self.bottomCurveRadius = bottomCurveRadius
        self.topOrientationBF = topOrientationBF
        self.topCurveRadius = topCurveRadius
    
    @staticmethod
    def Base(outerDiameter, length, topOrientationBF, topCurveRadius):
        return DiskMathModel(outerDiameter, length, topOrientationBF=topOrientationBF, topCurveRadius=topCurveRadius)
        
    @staticmethod
    def End(outerDiameter, length, bottomOrientationBF, bottomCurveRadius):
        return DiskMathModel(outerDiameter, length, bottomOrientationBF, bottomCurveRadius)
    
    def __repr__(self):
        return f"<DiskMathModel>\noD = {self.outerDiameter}\nlength = {self.length}\nbo = {self.bottomOrientationBF}\nbcr = {self.bottomCurveRadius}\nto={self.topOrientationBF}\ntcr={self.topCurveRadius}\n"
    
class ManipulatorMathModel:
    def __init__(self, segmentConfigs, transitionConfigs, outerDiameter):
        super().__init__()
        self.segmentConfigs = segmentConfigs
        self.transitionConfigs = transitionConfigs
        self.outerDiameter = outerDiameter
        self.disks = []
        self.tendons = []
        
    def get_num_disks(self):
        self._ensure_generation()
        return len(self.disks)
    
    def get_disks(self):
        self._ensure_generation()
        return [d for d in self.disks]
    
    def get_tendons(self):
        self._ensure_generation()
        return [t for t in self.tendons]
    
    def get_disk(self, index):
        self._ensure_generation()
        return self.disks[index]
    
    def get_tendons_at_disk(self, index):
        self._ensure_generation()
        return [t for t in self.tendons if t.nJoints >= index]
    
    def get_knobbed_tendons_at_disk(self, index):
        self._ensure_generation()
        return [t for t in self.tendons if t.nJoints == index]
    
    def get_unkobbed_tendons_at_disk(self, index):
        self._ensure_generation()
        return [t for t in self.tendons if t.nJoints > index]
    
    @property
    def disk_knobbed_tendons_reversed_iterator(self):
        self._ensure_generation()
        for i, d in enumerate(reversed(self.disks[1:])):
            yield d, self.get_knobbed_tendons_at_disk(len(self.disks)-i-1)
        
    def _ensure_generation(self):
        if (not self.disks or not self.tendons) and self.segmentConfigs:
            self.generate_models()
        
    
    def generate_models(self) -> List[DiskMathModel]:
        scs = self.segmentConfigs
        tc = self.transitionConfigs
        
        # Disk
        self.disks = [DiskMathModel.Base(self.outerDiameter, tc[0].diskLength, scs[0].orientationBF, scs[0].curveRadius),]
        for i,sc in enumerate(scs):
            orientationFlag = False
            for _ in range(sc.nJoints-1):
                self.disks.append(DiskMathModel(self.outerDiameter, sc.diskLength, sc.orientationBF + (pi/2 if sc.is2DoF and orientationFlag else 0), sc.curveRadius, sc.orientationBF + (pi/2 if sc.is2DoF and not orientationFlag else 0), sc.curveRadius))
                orientationFlag = not orientationFlag
                
            if i >= len(scs)-1:
                self.disks.append(DiskMathModel.End(self.outerDiameter, tc[i+1].diskLength, sc.orientationBF + (pi/2 if sc.is2DoF and orientationFlag else 0), sc.curveRadius))
                break
            
            self.disks.append(DiskMathModel(self.outerDiameter, tc[i+1].diskLength, sc.orientationBF + (pi/2 if sc.is2DoF and orientationFlag else 0), sc.curveRadius, scs[i+1].orientationBF, scs[i+1].curveRadius))
        
        # Tendon
        self.tendons = []
        nJointsThrough = 0
        for sc in scs:
            nJointsThrough += sc.nJoints
            if sc.is2DoF:
                self.tendons += [TendonMathModel(o, sc.tendonDistFromAxis, nJointsThrough) for o in (np.array((-pi/2, 0, pi/2, pi)) + sc.orientationBF)]
            else:
                self.tendons += [TendonMathModel(o, sc.tendonDistFromAxis, nJointsThrough) for o in (np.array((-pi/2, pi/2)) + sc.orientationBF)]
  
    
        

    