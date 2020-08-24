from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.solver import *
from variable_neutral_line_manipulator.simulation.adam_command import *
import time
import numpy 
import math

def test_geometry():
    import pprint
    segments = [
        Segment2DoFGeometryModel(
            True,
            6,
            5,
            0,
            3,
            1.5,
            end_disk_length=5,
            disk_outer_diameter=5,
            centre_hole_diameter=1,
            tendon_guide_diameter=0.5
        ), ]
    disk_geometry_indices = generate_disk_geometry_indices(segments, 5.0)
    cad_indices = generate_CAD_indices(disk_geometry_indices)
    export_CAD_indices_for_simulation(disk_geometry_indices, cad_indices, "./.temp")
    

def test_math_model():
    segments = [
        SegmentMathModel(
            is_2_DoF=True,
            n_joints=2,
            disk_length=5,
            orientationMF=0,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=5,
        )
    ]
    
    manipulator_model = ManipulatorMathModel(segments)
    disk_model_states = IterativeSolver().solve(manipulator_model, [[4,3,2,1]])
    print(disk_model_states)
    
if __name__ == "__main__":
    test_math_model()