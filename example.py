
from variable_neutral_line_manipulator.util import Logger, combine_tables, write_to_csv
from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.common.external_load import *
from variable_neutral_line_manipulator.common.result import *

from variable_neutral_line_manipulator.simulation.sim_model import SimManipulatorAdamModel
from variable_neutral_line_manipulator.math_model.solver import DirectSolver, IterativeSolver
from math import degrees


def eval_from_sim(manipulator_model, input_tensions: List[float], external_loads: List[ExternalLoad]):
    """ Acquire the result from Adams View simulation """
    s = SimManipulatorAdamModel(manipulator_model)

    try:
        s.run_sim(input_tensions,
                  initial_disk_overlap_length=0.05,
                  marker_offset_from_contact=0.1,
                  contact_config=SimManipulatorAdamModel.ContactConfig(
                      stiffness=6e6,
                      force_exponent=3.5,
                      damping=6e4,
                  ),
                  num_steps=1000,
                  max_iterations_search_eqilibrium=10000,
                  num_joint_angle_validation=0,
                  solver_translational_limit=1,
                  solver_rotational_limit=pi/10,
                  solver_error_threshold=1e-4,
                  solver_imbalance=1e-4,
                  solver_stability=4e-5,
                  external_loads=external_loads,
                  use_cpp_solver=True)
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
                  external_loads,
                  math_manipulator_state,
                  sim_manipulator_state,
                  output_path):
    """ Write results in CSV format"""
    model_table = generate_manipulator_model_table(
        manipulator_model,
    )

    input_tensions_table = generate_input_tensions_table(
        manipulator_model,
        input_tensions
    )

    external_loads_table = generate_external_load_table(
        external_loads=external_loads
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
        model_table,
        input_tensions_table + [[]] + external_loads_table, # Add an empty row in between
        joint_angle_table,
        reaction_table,
        tf_table))


def main():
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)

    # Define manipulator model
    segments = [
        TwoDOFParallelSegmentModel(
            n_joints=9,
            disk_length=12,
            base_orientationMF=0,
            distal_orientationDF=0,
            curve_radius=6,
            tendon_dist_from_axis=3.5,
            end_disk_length=None,
        ),
        # TwoDOFParallelSegmentModel(
        #     n_joints=4,
        #     disk_length=12,
        #     base_orientationMF=0,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     tendon_dist_from_axis=3.5,
        #     end_disk_length=None,
        # ),
        # TwoDOFParallelSegmentModel(
        #     n_joints=4,
        #     disk_length=12,
        #     base_orientationMF=pi/4,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     tendon_dist_from_axis=3.5,
        #     end_disk_length=None,
        # ),
        # TwoDoFSegmentModel(
        #     n_joints=4,
        #     disk_length=12,
        #     base_orientationMF=pi/4,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     end_disk_length=None,
        #     tendon_models=[
        #         TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=0,
        #         ),
        #         TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=2*pi/3,
        #         ),TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=4*pi/3,
        #         )
        #     ]
            
        # ),
        # TwoDoFSegmentModel(
        #     n_joints=4,
        #     disk_length=12,
        #     base_orientationMF=pi/4,
        #     distal_orientationDF=pi/2,
        #     curve_radius=6,
        #     end_disk_length=None,
        #     tendon_models=[
        #         TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=pi/3,
        #         ),
        #         TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=3*pi/3,
        #         ),TendonModel(
        #             dist_from_axis=3.5,
        #             orientation=5*pi/3,
        #         )
        #     ]
        # )
    ]
    model = ManipulatorModel(segments)

    # Define input tensions
    input_tensions = np.array([2,1], dtype=float)
    # input_tensions = np.array([2.5,2,1.5,1,1,2.5,2,1.5], dtype=float)

    # Define external loads (if there is any)
    external_loads = []
    # external_loads = [
    #     ExternalLoad(
    #         1, np.array((0.5, 0, 0)), np.array((0.01, 0.02, 0.01)), np.array((7, 0, 18.0)), is_attached_to_disk=False
    #     )
    # ]

    # Acquire results from simulation
    sim_state = eval_from_sim(model, input_tensions, external_loads)

    # Terminate the program if simulation result fails
    if sim_state is None:
        print("Terminate program")
        return

    # Compute results from math model (Currently external loads are not supported for math model)
    math_state = eval_from_math_model(model, input_tensions)

    # Write results
    write_results(model, input_tensions, external_loads, math_state, sim_state, "results.csv")


if __name__ == "__main__":
    main()
