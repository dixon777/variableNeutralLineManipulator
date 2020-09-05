from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.solver import *


if __name__ == "__main__":
    from math import degrees
    segments = [
        SegmentModel(
            n_joints=10,
            disk_length=5,
            base_orientationMF=0,
            distal_orientationDF=pi/2,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=None,
        )
    ]

    manipulator_model = ManipulatorModel(segments)
    res = DirectSolver().solve(
        manipulator_model, [[2, 1.5, 1, 0.5]]
    )
    print(res)
