from math import sqrt, sin, cos, degrees, pi
import numpy as np

from .entities import TendonGuideGeometry
"""
Model construction
"""
def validate_disk_param(outer_diameter, 
                        length,
                        bottom_curve_radius=None, 
                        top_curvature_radius=None, 
                        top_curvature_orientationDF=0.0, 
                        centre_hole_diameter=None, 
                        tendon_guides_geometries:list=[], 
                        **kwargs):
        errors = []
        
        # Disk param
        if outer_diameter is None or outer_diameter <= 0.0:
                errors.append("Outer diameter must be greater than 0")
        
        if length is None or length <= 0.0:
                errors.append("Length must be must be greater than 0")

        if bottom_curve_radius is not None and bottom_curve_radius <=outer_diameter/2:
                errors.append("Bottom curvature radius must be greater than half of outer diameter")
        
        if top_curvature_radius is not None and top_curvature_radius <=outer_diameter/2:
                errors.append("Top curvature radius must be greater than half of outer diameter")
        
        if top_curvature_radius is not None and top_curvature_orientationDF is None:
                errors.append("Top curvature relative orientation must not be None")                
        
        if centre_hole_diameter is not None and centre_hole_diameter >= outer_diameter:
                errors.append("Center hole diameter must be less than outer diameter")
                
        
        # Tendon guide param
        for i, tg1 in enumerate(tendon_guides_geometries):
                if tg1.diameter + tg1.dist_from_axis*2 >= outer_diameter:
                        warnings.append(f"{i}-th tendon guide's surface intersect the outer cylindrical surface")
                        
                if centre_hole_diameter and tg1.dist_from_axis*2 - tg1.diameter <= centre_hole_diameter:
                        warnings.append(f"{i}-th tendon guide's surface intersect the centre hole cylindrical surface")
                
                for j, tg2 in enumerate(tendon_guides_geometries[i+1:]):
                        diff = np.linalg.norm(
                                tg1.dist_from_axis * np.array((cos(tg1.orientationBF), sin(tg1.orientationBF)))
                                - 
                                tg2.dist_from_axis * np.array((cos(tg2.orientationBF), sin(tg2.orientationBF)))
                        )
                        if diff < tg1.diameter/2 + tg2.diameter/2:
                                errors.append(f"{i}-th and {i+j}-th tendon guides overlap")
                                
        
        return errors