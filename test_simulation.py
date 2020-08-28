from variable_neutral_line_manipulator.simulation.entities import *
from variable_neutral_line_manipulator.simulation.adam_command import *

def test_simulation_socket():
    s = AdamViewSocket()
    print(s.get_default_settings())


def test_manipulator_creator():
    segments = [
            SimSegment2DoFModel(
                is_2_DoF=True, 
                n_joints=1,
                disk_length=5, 
                orientationMF=0, 
                curve_radius=3, 
                tendon_dist_from_axis=2, 
                end_disk_length=None, 
                disk_outer_diameter=5,
            )
        ]
    manipulatorModel = SimManipulatorModel(segments)
    s = SimManipulatorAdamExecuter(manipulatorModel)
    s.reset_model()
    s.generate_model()
    s.set_forces([2.0,1.5,1,0.5])
    # s.run(3000, 1)
    
    
if __name__ == "__main__":
    # test_simulation_socket()
    test_manipulator_creator()