from variable_neutral_line_manipulator.common.entities import *
from variable_neutral_line_manipulator.simulation.adam_command import *

def test_simulation_socket():
    s = AdamViewSocket()
    print(s.get_default_settings())


def test_manipulator_creator():
    segments = [
            Segment2DoFGeometryModel(
                True, 2,
                5, 0, 3, 2, None, 5, 1, 0.1
            )
        ]
    manipulatorModel = ManipulatorGeometryModel(segments)
    s = ManipulatorCreator(manipulatorModel)
    s.reset_model()
    s.generate_model()
    s.set_forces([2.0,1.5,1,0.5])
    # s.run(3000, 1)
    
    
if __name__ == "__main__":
    # test_simulation_socket()
    test_manipulator_creator()