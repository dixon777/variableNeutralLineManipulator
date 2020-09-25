from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.solver import *
from variable_neutral_line_manipulator.common.result import *


if __name__ == "__main__":
    from math import degrees
    segments = [
        SegmentModel(
            n_joints=2,
            disk_length=5,
            base_orientationMF=0,
            distal_orientationDF=0,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=None,
        ),
        SegmentModel(
            n_joints=2,
            disk_length=5,
            base_orientationMF=pi/2,
            distal_orientationDF=0,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=None,
        ),
        SegmentModel(
            n_joints=2,
            disk_length=5,
            base_orientationMF=0,
            distal_orientationDF=0,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=None,
        ),
    ]

    manipulator_model = ManipulatorModel(segments)
    math_disk_states = DirectSolver().solve(
        manipulator_model, [1.5,1.5,1,0.8]
    )
    model_table = generate_manipulator_model_table(manipulator_model)
    
    result_table = generate_disk_states_compare_table(
        math_disk_states,
        math_disk_states
    )
    
    combine_tables(model_table,result_table)
