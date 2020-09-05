from variable_neutral_line_manipulator.simulation.entities import *
from variable_neutral_line_manipulator.simulation.adam_command import *
    
def process():
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
    manipulatorModel = ManipulatorModel(segments)
    s = SimManipulatorAdamExecuter(manipulatorModel)
    s.reset_model()
    s.generate_model()
    
    res = s.run_sim([2,1.5,1,0.5], total_iterations=1,
                    duration=0.001,
                    step_size=0.0000005, 
                    max_iterations_search_eqilibrium=300)
    res = s.extract_state()
    print(res)

if __name__ == "__main__":
    process()