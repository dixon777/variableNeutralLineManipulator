from .common import SegmentMathConfig

"""
    Provides mutually agreed data structures between UI and repository
"""

class TensionInputDisplayModel:
    def __init__(self, orientation, tension):
        self.orientation = orientation
        self.tension = tension

class TensionInputListDisplayModel:
    def __init__(self):
        self.values = []
        
    def updateTension(self, i, j, tension):
        self.values[i][j].tension = tension
        
    def to_pure_values(self):
        return [[t.tension for t in tl] for tl in self.values]
    
class SegmentConfigDisplayModel(SegmentMathConfig):
    def __init__(self, key, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.key = key
        
        
class ManipulatorConfigDisplayModel:
    def __init__(self, 
                 base_disk_length=0, 
                 outer_diameter=0):
        super().__init__()
        self.base_disk_length = base_disk_length
        self.outer_diameter = outer_diameter
        self.segment_models = {}
        