from math import pi
from variable_neutral_line_manipulator.math_model import *

def test_force_model():
    s1 = SegmentMathConfig(True, 3, 2, 0, 2, 1, 2)
    s2 = SegmentMathConfig(False, 2, 1, 0, 1, 0.5, 1)
    
    err = None
    m = ManipulatorMathModel([s1,s2], 2, 1.5)
    if m.error_dict:
        for k, s in m.error_dict.items():
            print(s)
        return
    
    manipulator_state = eval_manipulator_state(m, [[4,3,3,2], [2,1]], SolverType.DIRECT)
        
    
    for s in manipulator_state.disk_states:
        print(s.bottom_joint_angle*180/pi)
    
    for tf in manipulator_state.TFs_DF:
        print(tf)
        
def testCB():
    """ 
        To ensure the variables, which are going to be used in a closure, guaranteed to remain constant until the closure is called,
        an extra outer closure wrapping the closure is created and called during initialisation, as following.
    """
    y = 8
    def outer():
        x = y
        def __inner():
            return x
        return __inner
    
    for i in range(10):
        cb = outer()
        y = i
        print(cb())

if __name__ == "__main__":
    test_force_model()