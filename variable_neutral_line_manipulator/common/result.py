from math import degrees
from typing import List, Iterable
from .entities import ManipulatorModel, DiskState, ManipulatorState
from .external_load import *
from ..util.table_operation import *
import numpy as np


def generate_manipulator_model_table(manipulator_model: ManipulatorModel):
    res = [
        ["Manipulator model:"],
        ["Num joints", f"{manipulator_model.num_joints}"],
        []
    ]
    # Segments
    res += [
        ["Segments:"],
    ]
    for i, s in enumerate(manipulator_model.segments):
        res.append([f"Segment {i+1}:"])
        for k, v in s.get_data_pairs():
            if k == "base_orientationMF" or k == "distal_orientationDF":
                res.append([k, f"{degrees(v):.2f}"])
            else:
                res.append([k, f"{v:.2f}" if isinstance(v, float) else f"{v}"])
        res.append([""])
            
    return res


def generate_applied_tensions_table(manipulator_model: ManipulatorModel, applied_tensions: List[float]):
    res = [
        ["Applied tensions:"],
        ["Index", "Dist from axis", "Orientation (deg)", "Tension"],
    ]
    for i, (tm, tension) in enumerate(zip(manipulator_model.tendon_models, applied_tensions)):
        res.append(
            [i, f"{tm.dist_from_axis:.2f}",
                f"{degrees(tm.orientation):.2f}", f"{tension:.2f}"]
        )
    return res

def generate_external_load_table(external_loads: List[ExternalLoad]):
    res = [
        ["External loads:"],
        ["Index"]
    ]
    for i, el in enumerate(external_loads):
        data_table = []
        for k, v in el.get_data_pairs():
            if isinstance(v, Iterable):
                data_table.append([ k, *v])
            elif isinstance(v, bool):
                data_table.append([ k, 1 if v else 0])
            else:
                data_table.append([ k,v])
        res += combine_tables(
            [[f"{i+1}"]], data_table, space_between=0
        ).tolist()
        res.append([])
    
    return res


def generate_final_state_TF_table(math_manipulator_state: ManipulatorState,
                                  sim_manipulator_state: ManipulatorState):

    # math_tf = math_manipulator_state.get_TF(
    #     math_manipulator_state.manipulator_model.num_joints, 'top')
    # sim_tf = sim_manipulator_state.get_TF(
    #     sim_manipulator_state.manipulator_model.num_joints, 'top')
    
    math_disp = math_manipulator_state.tip_disp
    sim_disp = sim_manipulator_state.tip_disp
    # res = [
    #     ["Disp (End effector):"],
    #     ["Math:"],
    #     *[list(r) for r in math_tf],
    #     [],
    #     ["Sim:"],
    #     *[list(r) for r in sim_tf],
    #     [],
    #     ["Diff:"],
    #     *[list(r) for r in abs(math_tf-sim_tf)],
    # ]
    res = [
        ["Disp (End effector):"],
        ["Math:"],
        [r for r in math_disp],
        [],
        ["Sim:"],
        [r for r in sim_disp],
        [],
        ["Diff:"],
        [r for r in  abs(math_disp-sim_disp)],
    ]
    return res

def generate_final_state_joint_angle_table(math_manipulator_state: ManipulatorState,
                                  sim_manipulator_state: ManipulatorState):
    math_disk_states = math_manipulator_state.disk_states
    sim_disk_states = sim_manipulator_state.disk_states
    
    res = [
        ["Joint angles (deg):"],
        ["Joint", "Math", "Sim", "Diff"]
    ]
    for i, (ms, ss) in enumerate(zip(math_disk_states[1:], sim_disk_states[1:])):
        m_joint_angle = degrees(
            ms.bottom_joint_angle if ms.bottom_joint_angle is not None else 0)
        s_joint_angle = degrees(
            ss.bottom_joint_angle if ss.bottom_joint_angle is not None else 0)
        res.append([f"{i+1}", f"{m_joint_angle}", f"{s_joint_angle}",
                    f"{abs(m_joint_angle - s_joint_angle)}"])

    return res

def generate_final_state_reaction(math_manipulator_state: ManipulatorState,
                                       sim_manipulator_state: ManipulatorState):
    math_disk_states = math_manipulator_state.disk_states
    sim_disk_states = sim_manipulator_state.disk_states
    if len(math_disk_states) != len(sim_disk_states):
        print("Warning, disk states lengths are not consistent")
        
    res = [
        ["Contact reaction"]
    ]

    # Contact reaction
    for i, (ms, ss) in enumerate(zip(math_disk_states, sim_disk_states)):
        res += [
            [],
            [f"{i}-th joint:" if i > 0 else "Base"],
        ]
        if ms.has_bottom_contact and ss.has_bottom_contact:
            res.append([f"Bottom:"])
            res.append(["Component (N or Nmm)", "Math", "Sim", "Diff"])
            for force_comp_str, m_val, s_val in zip(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
                                                    ms.bottom_contact_force_and_pure_momentDF,
                                                    ss.bottom_contact_force_and_pure_momentDF):

                res.append(
                    [force_comp_str, f"{m_val}", f"{s_val}", f"{abs(m_val - s_val)}"])

        if ms.has_top_contact and ss.has_top_contact:
            res.append([f"Top:"])
            res.append(["Component (N or Nmm)", "Math", "Sim", "Diff"])
            for force_comp_str, m_val, s_val in zip(["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
                                                    ms.top_contact_force_and_pure_momentDF,
                                                    ss.top_contact_force_and_pure_momentDF):

                res.append(
                    [force_comp_str, f"{m_val}", f"{s_val}", f"{abs(m_val - s_val)}"])

    return res


