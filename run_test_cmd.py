
from variable_neutral_line_manipulator.util import Logger, combine_tables, write_to_csv
from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.common.result import *

from variable_neutral_line_manipulator.simulation.sim_model import SimManipulatorAdamModel
from variable_neutral_line_manipulator.math_model.solver import DirectSolver, IterativeSolver
from math import degrees


def eval_from_sim(manipulator_model, input_tensions: List[float]):
    """ Acquire the result from Adams View simulation """
    # Simulation
    s = SimManipulatorAdamModel(manipulator_model)

    # Uncomment the following lines to run the simulation, otherwise it will extract the current silllation result on Adams View
    s.clear_model()
    s.generate_model(
        initial_disk_overlap_length=0.05, # 0.04 0.16
        marker_offset_from_curve=0.06,     # 0.03 0.16
        contact_config=SimManipulatorAdamModel.ContactConfig(
            stiffness=3e6,
            force_exponent=3.5,
            damping=3e4,
        )
    )
    try:
        s.run_sim(input_tensions,
                  num_steps=3000,
                  max_iterations_search_eqilibrium=10000,
                  num_joint_angle_validation=0,
                  solver_translational_limit=3,
                  solver_rotational_limit=pi/10,
                  solver_error_threshold=1e-5,
                  solver_imbalance=1e-5,
                  solver_stability=1e-5)  # 6e-5s
    except RuntimeError as e:
        print(e)
        return None

    return s.extract_final_state()


def eval_from_math_model(manipulator_model: ManipulatorModel, input_tensions: List[float]):
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
    model_table = generate_manipulator_model_table(
        manipulator_model,
    )
    
    input_table = generate_input_tensions_table(
        manipulator_model,
        input_tensions
    )

    joint_angle_table = generate_final_state_joint_angle_table(
        math_manipulator_state,
        sim_manipulator_state
    )

    reaction_table = generate_final_state_reaction(
        math_manipulator_state,
        sim_manipulator_state
    )

    tf_table = generate_final_state_TF_table(
        math_manipulator_state,
        sim_manipulator_state
    )

    write_to_csv(output_path, combine_tables(
        model_table, input_table, joint_angle_table, reaction_table, tf_table))


def main():
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)

    # Define manipulator model
    segments = [
        # SegmentModel(
        #     n_joints=3,
        #     disk_length=12,
        #     base_orientationMF=0,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     tendon_dist_from_axis=3.5,
        #     end_disk_length=None,
        # ),
        SegmentModel(
            n_joints=9,
            disk_length=12,
            base_orientationMF=0,
            distal_orientationDF=pi/2,
            curve_radius=6,
            tendon_dist_from_axis=3.5,
            end_disk_length=None,
        ),
        
    ]
    model = ManipulatorModel(segments)

    # Define input tensions
    input_tensions = np.array([1.5,1.5,1,1], dtype=float)*9.81

    # Acquire results from simulation
    sim_state = eval_from_sim(model, input_tensions)
    
    # sim_state = SimManipulatorAdamModel(model).extract_final_state()

    if sim_state is None:
        print("Terminate program")
        return

    # Compute results from math model
    math_state = eval_from_math_model(model, input_tensions)

    # Write results
    write_results(model, input_tensions, math_state, sim_state)


if __name__ == "__main__":
    main()
