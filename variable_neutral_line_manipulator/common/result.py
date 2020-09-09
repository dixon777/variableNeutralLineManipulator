from math import degrees
from typing import List
from .entities import ManipulatorModel, DiskState

def generate_manipulator_model_table(manipulator_model:ManipulatorModel):
    res = [
        ["Segments:"],
        ["Num joints", f"{manipulator_model.num_joints}"]
    ]
    
    for i, s in enumerate(manipulator_model.segments):
        res.append([f"{i}"])
        for k, v in dict(s).entries():
            res.append([k, f"{v}"])
        res.append([""])

def generate_disk_states_compare_table(math_disk_states: List[DiskState], 
                                       sim_disk_states: List[DiskState]):
    res = [
        ["Final state:"],
    ]
    
    if len(math_disk_states) != len(sim_disk_states):
        print("Warning, disk states lengths are not consistent")
    
    # Joint angles
    res.append(["Joint angles:"])
    res.append(["Joint (deg)", "Math", "Sim", "Diff"])
    for i, (ms, ss) in enumerate(zip(math_disk_states, sim_disk_states)):
        m_joint_angle = degrees(ms.bottom_joint_angle if ms.bottom_joint_angle is not None else 0)
        s_joint_angle =  degrees(ss.bottom_joint_angle if ss.bottom_joint_angle is not None else 0)
        res.append([f"{i}",f"{m_joint_angle}", f"{s_joint_angle}", f"{abs(m_joint_angle - s_joint_angle)}"])
        
    
    res.append([])
    
    # Contact reaction
    res.append(["Contact reaction component:"])
    for i, (ms, ss) in enumerate(zip(math_disk_states, sim_disk_states)):
        res.append([f"{i}-th joint:"])
        if ms.has_bottom_contact and ss.has_bottom_contact:
            res.append([f"Bottom:"])
            res.append(["Component (N or Nmm)", "Math", "Sim", "Diff"])
            for force_comp_str, m_val, s_val in zip(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"], 
                                        ms.bottom_contact_force_and_pure_momentDF,
                                        ss.bottom_contact_force_and_pure_momentDF):
                
                res.append([force_comp_str, f"{m_val}", f"{s_val}", f"{abs(m_val - s_val)}"])
            
        if ms.has_top_contact and ss.has_top_contact:
            res.append([f"Top:"])
            res.append(["Component (N or Nmm)", "Math", "Sim", "Diff"])
            for force_comp_str, m_val, s_val in zip(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"], 
                                        ms.top_contact_force_and_pure_momentDF,
                                        ss.top_contact_force_and_pure_momentDF):
            
                res.append([force_comp_str, f"{m_val}", f"{s_val}", f"{abs(m_val - s_val)}"])
    
    return res

