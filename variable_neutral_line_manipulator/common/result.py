from math import degrees
from typing import List
from .entities import ManipulatorModel, DiskState
import numpy as np

def generate_manipulator_model_table(manipulator_model:ManipulatorModel, input_tensions: List[float]):
    res = [
        ["Manipulator model:"],
        ["Num joints", f"{manipulator_model.num_joints}"]
    ]
    # Segments
    res += [
        ["Segments:"],
    ]
    for i, s in enumerate(manipulator_model.segments):
        res.append([f"Segment {i}:"])
        for k, v in dict(s).items():
            if k == "__class_name__":
                continue
            elif k == "base_orientationMF" or k == "distal_orientationDF":
                res.append([k, f"{degrees(v):.2f}"])
            else:
                res.append([k, f"{v:.2f}" if isinstance(v,float) else f"{v}"])
        res.append([""])
        
    res += [
        ["Tendon models:"],
        ["Index", "Dist from axis", "Orientation (deg)", "Tension"],
    ]
    
    # Input tensions
    for i, (tm, tension) in enumerate(zip(manipulator_model.tendon_models,input_tensions)):
        res.append(
            [i, f"{tm.dist_from_axis:.2f}", f"{degrees(tm.orientation):.2f}", f"{tension:.2f}"]
        )
    return res

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



def ensure_equal_num_cells_per_row(rows):
    max_row_len = max(len(r) for r in rows)
    for r in rows:
        r += [""]*(max_row_len - len(r))
    return rows

def ensure_equal_num_rows(tables):
    max_rows = max(len(t) for t in tables)
    for i in range(len(tables)):
        tables[i] += [[""]*len(tables[i][0]) for _ in range(max_rows - len(tables[i]))]
    return tables

def combine_tables(*tables):
    tables = [ensure_equal_num_cells_per_row(t) for t in tables]
    tables = ensure_equal_num_rows(tables)
    tables = [np.array(t) for t in tables]
    combined_table = tables[0]
    
    for t in tables[1:]:
        combined_table = np.hstack((combined_table, np.array([""]*len(t)).reshape((-1,1)), t))
    return combined_table