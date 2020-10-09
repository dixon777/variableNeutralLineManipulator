
from variable_neutral_line_manipulator.util import Logger, combine_tables
from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.common.result import generate_disk_states_compare_table, generate_manipulator_model_table, write_to_csv

from variable_neutral_line_manipulator.simulation.sim_model import SimManipulatorAdamModel
from variable_neutral_line_manipulator.math_model.solver import DirectSolver
from math import degrees

    
def eval_from_sim(manipulator_model, input_tensions:List[float]):
    """ Acquire the result from Adams View simulation """
    # Simulation
    s = SimManipulatorAdamModel(manipulator_model)
    
    # Uncomment the following lines to run the simulation, otherwise it will extract the current simulation result on Adams View
    s.clear_model()
    s.generate_model()
    try:
        s.run_sim(input_tensions,
                num_steps=100,
                max_iterations_search_eqilibrium=2000,
                solver_translational_limit=5,
                solver_rotational_limit=pi/10,
                solver_stability=1e-5) # 6e-5s
    except RuntimeError as e:
        print(e)
        return None
    
    return s.extract_final_state()
 
def eval_from_math_model(manipulator_model: ManipulatorModel, input_tensions:List[float]):   
    """ Evaluate the result from math model """
    return DirectSolver().solve(
        manipulator_model, input_tensions
    )

    
    
def write_results(manipulator_model, 
                  input_tensions, 
                  math_manipulator_state, 
                  sim_manipulator_state,
                  output_path="result.csv"):
    """ Write results in CSV format"""
    model_table = generate_manipulator_model_table(manipulator_model, input_tensions)
    
    result_table = generate_disk_states_compare_table(
        math_manipulator_state,
        sim_manipulator_state
    )
        
    write_to_csv(output_path, combine_tables(model_table, result_table))

def main():
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)
    
    # Define manipulator model
    segments = [
        SegmentModel(
            n_joints=10,
            disk_length=12,
            base_orientationMF=0,
            distal_orientationDF=pi/2,
            curve_radius=6,
            tendon_dist_from_axis=3.5,
            end_disk_length=12,
        ),
        # SegmentModel(
        #     n_joints=2,
        #     disk_length=12,
        #     base_orientationMF=pi/4,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     tendon_dist_from_axis=3.5,
        #     end_disk_length=12,
        # ),
    ]
    model = ManipulatorModel(segments)
    
    # Define input tensions
    input_tensions = np.array([2,1.5,1,.5], dtype=float)
    # input_tensions = np.array([300,300,300,300,200,200,200,200], dtype=float)
    
    # Acquire results from simulation
    sim_state = eval_from_sim(model, input_tensions)
    
    if sim_state is None:
        print("Terminate program")
        return
    
    # Compute results from math model
    math_state = eval_from_math_model(model, input_tensions)
    
    print(math_state.get_TF(6,pos="top"))
    print(math_state.get_TF(model.num_joints,pos="top"))
    
    # Write results
    write_results(model, input_tensions, math_state, sim_state)

if __name__ == "__main__":
    main()
