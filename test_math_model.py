from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.solver import *


if __name__ == "__main__":
    segments = [
        SegmentMathModel(
                is_2_DoF=True, 
                n_joints=2,
                disk_length=5, 
                orientationMF=0, 
                curve_radius=3, 
                tendon_dist_from_axis=2, 
                end_disk_length=None
            )
    ]

    manipulator_model = ManipulatorMathModel(segments)
    disk_model_states = DirectSolver().solve(
        manipulator_model, [[2,1.5,1,0.5]]
    )
    print(disk_model_states)