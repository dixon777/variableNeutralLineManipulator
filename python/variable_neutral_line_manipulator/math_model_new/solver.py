from math import cos, sin, asin, atan, atan2, sqrt

from .models import *
from .calculation import *
from .vec import *

class SolverType:
    DIRECT="direct"
    BINARY="binary"
    NEWTON="Newton"
    
class TendonModelState():
    def __init__(self, tendon_model: TendonMathModel, tension_in_disk: float, is_knob: bool):
        self.model = tendon_model
        self.tension_in_disk = tension_in_disk
        self.is_knob = is_knob  # Currently useless for computation, but useful for visualisation
        
    def to_proximal_disk_frame(self, joint_angle):
        return TendonModelState(self.model, self.tension_in_disk, False)
    
    def __repr__(self):
        return f"tension: {self.tension_in_disk}, is_knob: {self.is_knob}"

class DiskModelState():
    def __init__(self,
                 disk: DiskMathModel,
                 tendon_states: List[TendonModelState],
                 bottom_contact_reactionRF: np.array,
                 bottom_joint_angle: float):
        self.disk = disk
        self.tendon_states:List[TendonModelState] = tendon_states
        self.bottom_contact_reactionRF = bottom_contact_reactionRF
        self.bottom_joint_angle = bottom_joint_angle
        
    @property
    def marked_unknobbed_tendon_states(self):
        return [TendonModelState(t.model, t.tension_in_disk, False) for t in self.tendon_states]
        
        
    def eval_proximal_disk_top_components(self, proximal_disk:DiskMathModel):
        top_orientationRF = self.disk.bottomOrientationBF - proximal_disk.bottomOrientationBF
        
        # contact
        c = np.concatenate((distalToProximalFrame(self.bottom_contact_reactionRF.force, self.bottom_joint_angle, top_orientationRF),
        distalToProximalFrame(self.bottom_contact_reactionRF.total_moment, self.bottom_joint_angle, top_orientationRF)))
        
        
        # tendon
        for t in self.tendon_states:
            c += ForceMomentVec(force=evalTopGuideForce(t.tension_in_disk, self.bottom_joint_angle, top_orientationRF),
                                disp=evalTopGuideEndDisp(
                                    proximal_disk.length, self.disk.bottomCurveRadius, t.model.distFromAxis, 
                                    t.model.orientationBF - proximal_disk.bottomOrientationBF, top_orientationRF
                                )).flat_total
        return c

def eval_bottom_tendon_components(disk:DiskMathModel, tendonStates, bottom_joint_angle):
        c = np.zeros(6)
        for t in tendonStates:
            c += ForceMomentVec(force=evalBottomGuideForce(t.tension_in_disk, bottom_joint_angle), 
                                disp=evalBottomGuideEndDisp(disk.length, disk.bottomCurveRadius, t.model.distFromAxis, t.model.orientationBF - disk.bottomOrientationBF)).flat_total
        return c

def solve_numerically(disk:DiskMathModel, knob_tendon_states:List[TendonModelState], distal_disk_state:DiskModelState, solver_type:SolverType):
    top_vec = distal_disk_state.eval_proximal_disk_top_components(disk) if distal_disk_state else np.zeros(6) 
    
    tension_states = [t for t in knob_tendon_states]
    if distal_disk_state:
        tension_states += distal_disk_state.marked_unknobbed_tendon_states
    
    def _equilibrium(bottom_joint_angle):
        sum_comp = top_vec + eval_bottom_tendon_components(disk, tension_states, bottom_joint_angle)
        bottom_contact_comp = ForceMomentVec.from_force_disp_total_moment(force=-sum_comp[:3], 
                                                                            disp=evalBottomContactDisp(disk.length, disk.bottomCurveRadius, bottom_joint_angle),
                                                                            total_moment=- sum_comp[3:])
        return bottom_contact_comp.pure_moment[0], bottom_contact_comp
    
    if solver_type == SolverType.BINARY:
        bottom_joint_angle, bottom_contact_vec = equation_binary_search(_equilibrium, lower=-pi/2, upper=pi/2, init=distal_disk_state.bottom_joint_angle if distal_disk_state else 0)
    else:
        raise NotImplementedError("Other solver has not been implemented")
    
    if bottom_joint_angle is None:
        return None
    return DiskModelState(disk, tension_states, bottom_contact_vec, bottom_joint_angle)

def equation_binary_search(func, lower, upper, init=None, threshold=0.0000000001):
    if func(lower)[0] * func(upper)[0] > 0:
        return None
    x = (lower+upper)/2 if init is None else init
    y,res = func(x)
    while abs(y) > threshold:
        if y < 0:
            lower = x
        else:
            upper = x
        x = (lower + upper)/2 
        y,res = func(x)
    return x, res

def atanC(y, x):
    tmp = atan(y/ x)
    while tmp < -pi/2:
        tmp += pi
    return tmp

def solve_direct(disk:DiskMathModel, knob_tendon_states:List[TendonModelState], distal_disk_state:DiskModelState):
    tendon_states = [t for t in knob_tendon_states]
    if distal_disk_state:
        tendon_states += distal_disk_state.marked_unknobbed_tendon_states
        
    sum_tensions = 0
    sum_bottom_guide_x_moment_sin = 0
    sum_bottom_guide_x_moment_cos = 0
    bottom_guide_disps = []
    for ts in tendon_states:
        bottom_guide_disps.append(evalBottomGuideEndDisp(disk.length, disk.bottomCurveRadius, ts.model.distFromAxis, ts.model.orientationBF - disk.bottomOrientationBF))
        sum_tensions += ts.tension_in_disk
        sum_bottom_guide_x_moment_sin += bottom_guide_disps[-1][2]*ts.tension_in_disk
        sum_bottom_guide_x_moment_cos += bottom_guide_disps[-1][1]*ts.tension_in_disk
        
    top_vec = distal_disk_state.eval_proximal_disk_top_components(disk) if distal_disk_state else np.zeros(6)
    
    # Refers to the note
    P = disk.bottomCurveRadius*top_vec[2] + (disk.length/2-disk.bottomCurveRadius)*sum_tensions + sum_bottom_guide_x_moment_sin
    Q = -disk.bottomCurveRadius*top_vec[1] - sum_bottom_guide_x_moment_cos
    R = top_vec[1]*(disk.bottomCurveRadius - disk.length/2) + top_vec[3]
    
    # P*sin(theta) + Q*cos(theta) = +-sqrt(P**2+Q**2)*sin(theta+atan(Q/P))
    # There are 2 possible formulae, mathematically speaking, but only 1 of them complies with the constraint
    # Validated by plotting both solutions to see which matches the original formula
    half_joint_angle = asin(R/sqrt(P**2 + Q**2)) - atan(Q/P)  # OR asin(R/sqrt(P**2 + Q**2)) - atan2(Q,P) 
    
    # To check whether the result complies with the original formula
    # Not sure whether this condition will be true in any case, but at least it is
    # confirmed that the solution must be evaluated from either 2 of these formulae
    if abs(P*sin(half_joint_angle) + Q*cos(half_joint_angle) + R) > 1**-10:
        print(f"ERROR: {P*sin(half_joint_angle) + Q*cos(half_joint_angle) + R}")
        half_joint_angle = asin(-R/sqrt(P**2 + Q**2)) - atan(Q/P)
    
    bottom_joint_angle = 2*half_joint_angle
    
    # Store the state related data for next proximal disk bottom joint angle evaluation
    # Force displacement has been evaluated
    bottom_guide_vecRF = 0
    for ts,d in zip(tendon_states, bottom_guide_disps):
        bottom_guide_vecRF += ForceMomentVec(force=evalBottomGuideForce(ts.tension_in_disk, bottom_joint_angle), disp=d).flat_total
    # bottom_guide_vecRF = eval_bottom_tendon_components(disk, tendon_states, bottom_joint_angle)
    
    bottom_contact_reactionRF = -top_vec - bottom_guide_vecRF
    bottom_contact_reactionRF = ForceMomentVec.from_force_disp_total_moment(bottom_contact_reactionRF[:3], 
                                                                            evalBottomContactDisp(disk.length, disk.bottomCurveRadius, bottom_joint_angle),
                                                                            bottom_contact_reactionRF[3:])
        
    return DiskModelState(disk, tendon_states, bottom_contact_reactionRF, bottom_joint_angle)

class StateComputer():
    def __init__(self):
        self.states = []
        self.tensionInputs = []
        
    def eval(self, manipulator_model:ManipulatorMathModel, tensionInputs:List, solver_type:SolverType):            
        self.tensionInputs = [ti for ti in tensionInputs]
        distal_disk_state = None
        
        for disk, knob_tension_models in manipulator_model.disk_knobbed_tendons_reversed_iterator:
            knob_tendon_states = [] 
            if knob_tension_models:
                knob_tendon_states = [TendonModelState(t, tensionInputs[-1][i], True) for i, t in enumerate(knob_tension_models)] 
                tensionInputs = tensionInputs[:-1]
                
            if solver_type == SolverType.DIRECT:
                distal_disk_state = solve_direct(disk, knob_tendon_states, distal_disk_state)
            elif solver_type == SolverType.BINARY or solver_type == SolverType.NEWTON:
                distal_disk_state = solve_numerically(disk, knob_tendon_states, distal_disk_state, solver_type)
                
            if not distal_disk_state:
                return False
            self.states.insert(0, distal_disk_state)
            
        return True