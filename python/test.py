from math import pi
from variable_neutral_line_manipulator.math_model_new import *

def test_force_model():
    se = StateComputer() 
    
    
    start = TransitionMathConfig(2)
    s1 = SegmentMathConfig(True, 3, 2, 0, 2, 1)
    mid = TransitionMathConfig(2)
    s2 = SegmentMathConfig(False, 2, 1, 0, 1, 0.5)
    end = TransitionMathConfig(1)
    
    # m = ManipulatorMathModel([s1], [start, mid], 5)
    # m.generate_models()
    # se.eval(m, [[4,1,3,1],], StateEvaluator.NumericalSolverType.BINARY)
    
    err = None
    m = ManipulatorMathModel([s1,s2], [start, mid, end], 2.1)
    if m.error_dict:
        for k, s in m.error_dict.items():
            print(s)
        return
    
    se.eval(m, [[4,3,3,2], [2,1]], SolverType.DIRECT)
        
    
    for s in se.states:
        print(s.bottom_joint_angle*180/pi)

if __name__ == "__main__":
    test_force_model()