from variable_neutral_line_manipulator.simulation.entities import *
from variable_neutral_line_manipulator.simulation.manipulator_model import *
from variable_neutral_line_manipulator.common import Timer
from variable_neutral_line_manipulator.math_model.solver import DirectSolver
from variable_neutral_line_manipulator.common.result import generate_disk_states_compare_table
from math import degrees


def write_to_csv(path, table):
    import csv
    with open(path, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerows(table)


def process():
    input_forces = [2, 1.5, 1, 0.5]
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

    # Math model
    math_disk_states = DirectSolver().solve(
        manipulator_model, input_forces
    )

    # Simulation
    s = SimManipulatorAdamModel(manipulator_model)
    
    # # Uncomment the following lines to run the simulation, otherwise it will extract the current simulation result on Adams View
    s.clear_model()
    s.generate_model()
    s.run_sim(input_forces,
                total_iterations=1,
                duration=0.1,
                step_size=0.00001,
                max_iterations_search_eqilibrium=500)
    
    sim_disk_states = s.extract_final_state()

    # Write results to result.csv
    result_table = generate_disk_states_compare_table(
        math_disk_states,
        sim_disk_states
    )

    write_to_csv("result.csv", result_table)


if __name__ == "__main__":
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)
    process()
