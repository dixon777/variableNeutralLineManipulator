from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.common.common import Timer, Logger
from variable_neutral_line_manipulator.common.result import generate_disk_states_compare_table, generate_manipulator_model_table, combine_tables

from variable_neutral_line_manipulator.simulation.sim_model import SimManipulatorAdamModel
from variable_neutral_line_manipulator.math_model.solver import DirectSolver
from math import degrees


def write_to_csv(path, table):
    import csv
    with open(path, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerows(table)


def process():
    input_forces = np.array([2,3,2,2], dtype=float)
    segments = [
        SegmentModel(
            n_joints=2,
            disk_length=5,
            base_orientationMF=0,
            distal_orientationDF=pi/2,
            curve_radius=3,
            tendon_dist_from_axis=2,
            end_disk_length=None,
        ),
        #  SegmentModel(
        #     n_joints=1,
        #     disk_length=5,
        #     base_orientationMF=pi/4,
        #     distal_orientationDF=pi/2,
        #     curve_radius=3,
        #     tendon_dist_from_axis=2,
        #     end_disk_length=None,
        # ),
    ]
    manipulator_model = ManipulatorModel(segments)
    
    

    # Simulation
    s = SimManipulatorAdamModel(manipulator_model)
    
    # # Uncomment the following lines to run the simulation, otherwise it will extract the current simulation result on Adams View
    s.clear_model()
    s.generate_model()
    s.run_sim(input_forces,
                num_steps=50,
                max_iterations_search_eqilibrium=1500,
                solver_translational_limit=3,
                solver_rotational_limit=pi/10,
                solver_stability=6e-5) # 6e-5
    
    # sim_manipulator_state = s.extract_final_state()
    # sim_disk_states = sim_manipulator_state.disk_states
    
    # # Math model
    # math_manipulator_state = DirectSolver().solve(
    #     manipulator_model, input_forces
    # )
    # math_disk_states = math_manipulator_state.disk_states

    # # Write results to result.csv
    # model_table = generate_manipulator_model_table(manipulator_model, input_forces)
    
    # result_table = generate_disk_states_compare_table(
    #     math_disk_states,
    #     sim_disk_states
    # )
        
    # write_to_csv("result.csv", combine_tables(model_table, result_table))


if __name__ == "__main__":
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)
    process()
