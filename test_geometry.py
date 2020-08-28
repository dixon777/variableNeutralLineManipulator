from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.solver import *

def test_geometry():
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
        ), 
    ]
    manipulatorModel = ManipulatorGeometryModel(segments)
    disk_geometry_indices = manipulatorModel.generate_indices_disk_model_pairs(5.0)
    cad_indices = generate_CAD_indices(disk_geometry_indices)
    export_CAD_indices_for_simulation(
        disk_geometry_indices, cad_indices, "./.temp")


if __name__ == "__main__":
    test_geometry()