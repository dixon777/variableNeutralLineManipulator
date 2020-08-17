from ..math_model.models import *
import os





class ManipulatorGeometryModel(ManipulatorMathModel):
    def __init__(self, segment_configs, base_disk_length=0.0, outer_diameter=0.0, centre_hole_diameter=0.0, tendon_guide_diameter=0.0):
        super().__init__(segment_configs=segment_configs, base_disk_length=base_disk_length, outer_diameter=outer_diameter)
        self.centre_hole_diameter = centre_hole_diameter
        self.tendon_guide_diameter = tendon_guide_diameter
        
    def generate_disk_geometries(self):
        if not self.ensure_generation():
            return None
        
        disk_geometries = []
        for i, d in enumerate(self.disks):
            tendon_geometries = [TendonGuideGeometry.from_math_model(t, self.tendon_guide_diameter) for t in self.tendons if t.n_joints >= i]
            disk_geometries.append(DiskGeometry.from_math_model(
                d, 
                self.centre_hole_diameter,
                tendon_geometries
            ))
            
        return disk_geometries
    
    
    def generate_and_export_CAD(self, dir_path, export_type="step",tolerance=0.001):
        from ..cad.cadquery_disk_generator import generate_disk_CAD, export_CAD
        disk_geometries = self.generate_disk_geometries()
        import os
        
        os.makedirs(dir_path, exist_ok=True)
        
        for i, dg in enumerate(disk_geometries):
            cad_model, errors = generate_disk_CAD(dg)
            if len(errors) > 0:
                return
            export_CAD(cad_model, os.path.join(dir_path, f"{i}"), export_type, tolerance=tolerance)           
        

class TendonGuideGeometry():
    attr_keys = ["orientationBF", "dist_from_axis", "diameter"]
    
    def __init__(self,
                 orientationBF,
                 dist_from_axis,
                 diameter
                 ):
        self.orientationBF = orientationBF
        self.dist_from_axis = dist_from_axis
        self.diameter = diameter
        
    @staticmethod
    def from_math_model(model, diameter):
        return TendonGuideGeometry(
            model.orientationBF,
            model.dist_from_axis,
            diameter
        )
    
    def orientationDF(self, disk_orientationBF):
        return self.orientationBF - disk_orientationBF
    
    
    def __repr__(self):
        res = "<TendonGuideGeometry> "
        for k in self.attr_keys:
            res += f"{k} = {self.__dict__[k]}, "
        
        return res
    
    
class DiskGeometry():
    attr_keys = ["length", "outer_diameter", "bottom_orientationBF", "bottom_curve_radius", "top_orientationBF", "top_curve_radius", "centre_hole_diameter"]
    
    def __init__(self, 
                 length, 
                 outer_diameter,
                 bottom_orientationBF=0,
                 bottom_curve_radius=None, 
                 top_orientationBF=0, 
                 top_curve_radius=None,
                 centre_hole_diameter=None,
                 tendon_guide_geometries=[], 
                 ):
        self.length = length
        self.outer_diameter = outer_diameter
        self.bottom_orientationBF = bottom_orientationBF
        self.top_orientationBF = top_orientationBF
        self.bottom_curve_radius = bottom_curve_radius
        self.top_curve_radius = top_curve_radius
        self.centre_hole_diameter = centre_hole_diameter
        self.tendon_guide_geometries = tendon_guide_geometries
        
    @staticmethod
    def from_math_model(model, centre_hole_diameter, tendon_guide_geometries):
        return DiskGeometry(
            model.length,
            model.outer_diameter,
            model.bottom_orientationBF,
            model.bottom_curve_radius,
            model.top_orientationBF,
            model.top_curve_radius,
            centre_hole_diameter,
            tendon_guide_geometries,
        )
            
    @property
    def top_curve_orientationDF(self):
        return self.top_orientationBF - self.bottom_orientationBF
    
    
    def __repr__(self):
        res = "<DiskGeometry>:\n"
        for k in self.attr_keys:
            res += f"    - {k} = {self.__dict__[k]}\n"
        
        res += "    - tendon guide geometires:\n"
        for i, t in enumerate(self.tendonGuideGeometries):
            res += f"       {i}. {t}\n"
        return res
        
        

            
if __name__ == "__main__":
    m = ManipulatorGeometryModel([
        SegmentMathConfig(
            2, 2, 5, 0, 3, 2, 5
        )
    ])
    print(m.generate_disk_geometries())