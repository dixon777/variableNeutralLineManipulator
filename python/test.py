from math import pi
from variable_neutral_line_manipulator.math_model import *

def test_force_model():
    se = StateComputer() 
    
    
    s1 = SegmentMathConfig(True, 3, 2, 0, 2, 1, 2)
    s2 = SegmentMathConfig(False, 2, 1, 0, 1, 0.5, 1)
    
    err = None
    m = ManipulatorMathModel([s1,s2], 2, 1.5)
    if m.error_dict:
        for k, s in m.error_dict.items():
            print(s)
        return
    
    se.eval(m, [[4,3,3,2], [2,1]], SolverType.DIRECT)
        
    
    for s in se.states:
        print(s.bottom_joint_angle*180/pi)

if __name__ == "__main__":
    test_force_model()