from variable_neutral_line_manipulator.simulation.entities import *
from variable_neutral_line_manipulator.simulation.manipulator_model import *
from variable_neutral_line_manipulator.common import Timer
from math import degrees
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
    s = SimManipulatorAdamModel(manipulatorModel)
    s.clear_model()
    s.generate_model()
    
    s.run_sim([2,1.5,1,0.5], 
                    total_iterations=1,
                    duration=0.1,
                    step_size=0.00001, 
                    max_iterations_search_eqilibrium=500)
    disk_states = s.extract_final_state()
    print(disk_states)

if __name__ == "__main__":
    import logging
    Logger.switchLogger("manipulator")
    Logger.setLevel(logging.ERROR)
    process()