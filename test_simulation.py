from variable_neutral_line_manipulator.simulation.entities import *
from variable_neutral_line_manipulator.simulation.adam_command import *
    
def postprocess():
    segments = [
            SegmentModel(
                n_joints=18,
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
    s.run_sim([2,1.5,1,0.5], total_steps=40, iteration_per_step=200, max_iterations_search_eqilibrium=50)
    

if __name__ == "__main__":
    postprocess()