from typing import List
from math import pi, sqrt
import numpy as np

from ..common import ErrorDict
from .calculation import *
from .vec import *

class SegmentMathConfig:
    """
        Math definition of segment
    """
    def __init__(self, is_2_DoF, n_joints, disk_length, orientationBF, curve_radius, tendon_dist_from_axis, end_disk_length):
        self.n_joints = n_joints
        self.is_2_DoF = is_2_DoF
        self.disk_length = disk_length
        self.orientationBF = orientationBF
        self.curve_radius = curve_radius
        self.tendon_dist_from_axis = tendon_dist_from_axis
        self.end_disk_length = end_disk_length
            
class TendonMathModel:
    """
        Geometry definition of transitions between segments
    """
    def __init__(self, orientationBF, dist_from_axis, n_joints):
        self.orientationBF = orientationBF
        self.dist_from_axis = dist_from_axis
        self.n_joints = n_joints
        
    def __repr__(self):
        return f"<TendonMathModel> [orientationBF={self.orientationBF}, distance from axis={self.dist_from_axis}, num joints={self.n_joints}\n"
    
class DiskMathModel:
    """
        Geometry of disk 
    """
    def __init__(self, outer_diameter, length, bottom_orientationBF=0.0, bottom_curve_radius=None, top_orientationBF=None, top_curve_radius=None):
        self.outer_diameter = outer_diameter
        self.length = length
        self.bottom_orientationBF = bottom_orientationBF
        self.bottom_curve_radius = bottom_curve_radius
        self.top_orientationBF = top_orientationBF
        self.top_curve_radius = top_curve_radius
    
    @staticmethod
    def Base(outer_diameter, length, top_orientationBF, top_curve_radius):
        return DiskMathModel(outer_diameter, length, top_orientationBF=top_orientationBF, top_curve_radius=top_curve_radius)
        
    @staticmethod
    def End(outer_diameter, length, bottom_orientationBF, bottom_curve_radius):
        return DiskMathModel(outer_diameter, length, bottom_orientationBF, bottom_curve_radius, top_orientationBF=bottom_orientationBF)
    
    def __repr__(self):
        return f"<DiskMathModel> [outer diameter={self.outer_diameter}, length={self.length}, bottom orientation={self.bottom_orientationBF}, bottom curvature radius={self.bottom_curve_radius}, top orientation={self.top_orientationBF}, top curvature radius={self.top_curve_radius}]"
    
class ManipulatorMathModel:
    def __init__(self, segment_configs:List[SegmentMathConfig]=[], base_disk_length:float=0., outer_diameter:float=0.):
        super().__init__()
        self._segment_configs = segment_configs
        self._base_disk_length = base_disk_length
        self._outer_diameter = outer_diameter
        self._disks:List[DiskMathModel] = []
        self._tendons:List[TendonMathModel] = []
        self._error_dict = ErrorDict()
        
        self.ensure_generation()
    
    @property
    def segment_configs(self):
        return self._segment_configs.copy()
    
    @property
    def outer_diameter(self):
        return self._outer_diameter
    
    @property
    def base_disk_length(self):
        return self._base_disk_length
    
    @property
    def disks(self):
        if not self.ensure_generation():
            return []
        return self._disks.copy()
    
    @property
    def tendons(self):
        if not self.ensure_generation():
            return []
        return self._tendons.copy()
    
    def get_disk(self, index):
        if not self.ensure_generation():
            return None
        return self._disks[index]
    
    def get_tendons_at_disk(self, index):
        if not self.ensure_generation():
            return []
        return [t for t in self._tendons if t.n_joints >= index]
    
    def get_knobbed_tendons_at_disk(self, index):
        if not self.ensure_generation():
            return []
        return [t for t in self._tendons if t.n_joints == index]
    
    def get_unkobbed_tendons_at_disk(self, index):
        if not self.ensure_generation():
            return []
        return [t for t in self._tendons if t.n_joints > index]
    
    @property
    def error_dict(self):
        return self._error_dict.dict_copy()
    
    @property
    def disk_knobbed_tendons_iterator(self):
        if not self.ensure_generation():
            return None, None
        for i, d in enumerate(self._disks[1:]):
            yield d, self.get_knobbed_tendons_at_disk(i+1)
    
    @property
    def disk_knobbed_tendons_reversed_iterator(self):
        if not self.ensure_generation():
            return None, None
        for i, d in enumerate(reversed(self._disks[1:])):
            yield d, self.get_knobbed_tendons_at_disk(len(self._disks)-i-1)
            
    def update(self, outer_diameter=None, base_disk_length=None, segment_configs=None):
        if ((outer_diameter is None or 
           self._outer_diameter == outer_diameter) and 
            (base_disk_length is None or
             self._base_disk_length == base_disk_length) and 
            segment_configs is None):
            return

        self._disks.clear()
        self._tendons.clear()
        self._error_dict.clear()
        
        if outer_diameter is not None:
            self._outer_diameter = outer_diameter
        if base_disk_length is not None:
            self._base_disk_length = base_disk_length
        if segment_configs is not None:
            self._segment_configs = segment_configs
                
    def is_config_valid(self) -> bool:
        self._error_dict.clear()
        if len(self._segment_configs) < 1:
            return False
        for s in self._segment_configs:
            if s.curve_radius <= s.tendon_dist_from_axis:
                self._error_dict.add(s, "Physical compatibility: Curvature radius must be larger than tendon distance from axis")
            
            if s.curve_radius < self.outer_diameter/2:
                self._error_dict.add(s, "Physical compatibility: Curvature radius must be larger than or equal to outer radius")

            elif s.curve_radius - sqrt(s.curve_radius**2 - (self._outer_diameter/2)**2) > s.disk_length/2:
                self._error_dict.add(s, "Physical compatibility: The disk length is too small and not achivable with the configured curvature radius and outer diameter")

        return not self._error_dict.has_errors()
        
    
    def generate_models(self) -> bool:
        if not self.is_config_valid():
            return None
        
        scs = self._segment_configs
        
        # Disk
        self._disks = [DiskMathModel.Base(self._outer_diameter, self._base_disk_length, scs[0].orientationBF, scs[0].curve_radius),]
        for i,sc in enumerate(scs):
            orientationFlag = False
            for _ in range(sc.n_joints-1):
                self._disks.append(DiskMathModel(self._outer_diameter, sc.disk_length, sc.orientationBF + (pi/2 if sc.is_2_DoF and orientationFlag else 0), sc.curve_radius, sc.orientationBF + (pi/2 if sc.is_2_DoF and not orientationFlag else 0), sc.curve_radius))
                orientationFlag = not orientationFlag
                
            if i >= len(scs)-1:
                self._disks.append(DiskMathModel.End(self._outer_diameter, sc.end_disk_length, sc.orientationBF + (pi/2 if sc.is_2_DoF and orientationFlag else 0), sc.curve_radius))
                break
            
            self._disks.append(DiskMathModel(self._outer_diameter, sc.end_disk_length, sc.orientationBF + (pi/2 if sc.is_2_DoF and orientationFlag else 0), sc.curve_radius, scs[i+1].orientationBF, scs[i+1].curve_radius))
        
        # Tendon
        self._tendons = []
        n_jointsThrough = 0
        for sc in scs:
            n_jointsThrough += sc.n_joints
            if sc.is_2_DoF:
                self._tendons += [TendonMathModel(o, sc.tendon_dist_from_axis, n_jointsThrough) for o in (np.array((-pi/2, 0, pi/2, pi)) + sc.orientationBF)]
            else:
                self._tendons += [TendonMathModel(o, sc.tendon_dist_from_axis, n_jointsThrough) for o in (np.array((-pi/2, pi/2)) + sc.orientationBF)]
        return True
    
    def ensure_generation(self):
        if (not self._disks or not self._tendons) and self._segment_configs:
            return self.generate_models()
        return True

    