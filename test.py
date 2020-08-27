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
        ), 
    ]
    manipulatorModel = ManipulatorGeometryModel(segments)
    disk_geometry_indices = manipulatorModel.generate_indices_disk_model_pairs(5.0)
    cad_indices = generate_CAD_indices(disk_geometry_indices)
    export_CAD_indices_for_simulation(
        disk_geometry_indices, cad_indices, "./.temp")


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
    disk_model_states = IterativeSolver().solve(
        manipulator_model, [[4, 3, 2, 1]])
    print(disk_model_states)


def test_simulation_socket():
    s = AdamViewSocket()
    print(s.get_default_settings())


def test_manipulator_creator():
    segments = [
            Segment2DoFGeometryModel(
                True, 3,
                5, 0, 3, 2, None, 5, 1, 0.1
            )
        ]
    manipulatorModel = ManipulatorGeometryModel(segments)
    s = ManipulatorCreator(manipulatorModel)
    s.reset_model()
    s.generate_model()
    s.set_forces([2.0,1.5,1,0.5])
    s.run(700, 0.1)

if __name__ == "__main__":
    from math import pi
    # test_geometry()
    # test_simulation_socket()
    test_manipulator_creator()
    # a = DiskGeometryBase(0,0,2*pi,0)
    # b = DiskGeometryBase(0,0,0,0)
    # print(a == b)