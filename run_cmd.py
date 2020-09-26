
from variable_neutral_line_manipulator.util import Timer, Logger, combine_tables
from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.common.result import generate_disk_states_compare_table, generate_manipulator_model_table

from variable_neutral_line_manipulator.simulation.sim_model import SimManipulatorAdamModel
from variable_neutral_line_manipulator.math_model.solver import DirectSolver
from math import degrees


def write_to_csv(path, table):
    import csv
    with open(path, "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerows(table)

    
def create_manipulator_model():
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
    return ManipulatorModel(segments)
    
    
def eval_from_sim(manipulator_model, input_forces:List[float]):
    # Simulation
    s = SimManipulatorAdamModel(manipulator_model)
    
    # # Uncomment the following lines to run the simulation, otherwise it will extract the current simulation result on Adams View
    # s.clear_model()
    # s.generate_model()
    # s.run_sim(input_forces,
    #             num_steps=50,
    #             max_iterations_search_eqilibrium=1500,
    #             solver_translational_limit=3,
    #             solver_rotational_limit=pi/10,
    #             solver_stability=6e-5) # 6e-5
    
    return s.extract_final_state()
 
def eval_from_math_model(manipulator_model: ManipulatorModel, input_forces:List[float]):   
    # Math model
    return DirectSolver().solve(
        manipulator_model, input_forces
    )

    
    
def write_results(manipulator_model, input_forces, math_manipulator_state, sim_manipulator_state):
    """ Write results to result.csv """
    model_table = generate_manipulator_model_table(manipulator_model, input_forces)
    
    result_table = generate_disk_states_compare_table(
        math_manipulator_state,
        sim_manipulator_state
    )
        
    write_to_csv("result.csv", combine_tables(model_table, result_table))


if __name__ == "__main__":
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)
    
    input_forces = np.array([3,1.5,1,2.5], dtype=float)
    model = create_manipulator_model()
    sim_state = eval_from_sim(model, input_forces)
    math_state = eval_from_math_model(model, input_forces)
    write_results(model, input_forces, math_state, sim_state)
