from typing import List, Dict, Iterable
from math import sin, cos, sqrt, pi, atan, atan2, isnan, asin
from abc import ABC, abstractmethod

import numpy as np

from .entities import *


def _eval_top_components(current_disk_model: DiskMathModel, distal_disk_state: DiskMathState):
    if distal_disk_state is None:
        return (np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))

    disk_geometry = current_disk_model.disk_geometry
    top_tendon_states = distal_disk_state.tendon_states
    top_contact_joint_angle = distal_disk_state.bottom_joint_angle

    # Top vectors
    tendon_guide_top_end_force_disp_pairs = [
        (eval_tendon_guide_top_force(
            tm.tension_in_disk, top_contact_joint_angle, disk_geometry.top_orientationDF),
            eval_tendon_guide_top_end_disp(
            disk_geometry.length,
            disk_geometry.top_curve_radius,
            tm.model.dist_from_axis,
            tm.model.orientationMF - current_disk_model.bottom_orientationMF,
            disk_geometry.top_orientationDF,
        ))
        for tm in top_tendon_states]

    # Top total force
    top_contact_forceDF = distal_to_proximal_frame(
        -distal_disk_state.bottom_contact_forceDF, top_contact_joint_angle, disk_geometry.top_orientationDF)

    top_total_forceDF = (top_contact_forceDF + sum(f for (f, _)
                                                   in tendon_guide_top_end_force_disp_pairs))

    # Top total moment
    # Pure moment at contact
    top_contact_pure_momentDF = distal_to_proximal_frame(
        -distal_disk_state.bottom_contact_pure_momentDF, top_contact_joint_angle, disk_geometry.top_orientationDF)

    # Moment by contact force
    top_contact_total_momentDF = top_contact_pure_momentDF + np.cross(eval_top_contact_disp(disk_geometry.length,
                                                                                            disk_geometry.top_curve_radius,
                                                                                            top_contact_joint_angle,
                                                                                            disk_geometry.top_orientationDF
                                                                                            ), top_contact_forceDF)
    # Moment by tendon tensions
    top_total_momentDF = (top_contact_total_momentDF + sum(np.cross(r, f)
                                                           for (f, r) in tendon_guide_top_end_force_disp_pairs))

    return top_contact_forceDF, top_contact_pure_momentDF, top_total_forceDF, top_total_momentDF


def _eval_bottom_tendon_guide_components(current_disk_model: DiskMathModel, distal_disk_state: DiskMathState, input_forces: List[float]):
    disk_geometry = current_disk_model.disk_geometry

    bottom_knobbed_tendon_states = [TendonMathState(model, tension) for model, tension in zip(
        current_disk_model.knobbed_tendon_models, input_forces)] if input_forces is not None and len(input_forces) > 0 else []
    bottom_continous_tendon_states = distal_disk_state.tendon_states if distal_disk_state is not None else []
    bottom_tendon_states: List[TendonMathState] = (
        bottom_knobbed_tendon_states + bottom_continous_tendon_states)
    bottom_tendon_state_disp_pairs = tuple((s, eval_tendon_guide_bottom_end_disp(
        disk_geometry.length, disk_geometry.bottom_curve_radius, s.model.dist_from_axis, s.model.orientationMF -
        current_disk_model.bottom_orientationMF
    ))for s in bottom_tendon_states)

    return bottom_knobbed_tendon_states, bottom_continous_tendon_states, bottom_tendon_state_disp_pairs


class _SolverBase(ABC):
    @abstractmethod
    def solve(self, manipulator_model: ManipulatorMathModel, nested_input_forces: Iterable[List[float]]) -> List[DiskMathState]:
        pass


class DirectSolver(_SolverBase):
    def solve(self, manipulator_model: ManipulatorMathModel, nested_input_forces: Iterable[List[float]]) -> List[DiskMathState]:
        disk_states = []
        last_disk_state = None
        for disk_model, input_forces in manipulator_model.get_reversed_disk_model_input_forces_iterable(nested_input_forces):
            last_disk_state = DirectSolver.solve_single_disk(
                disk_model, last_disk_state, input_forces)
            if last_disk_state is None:
                break
            disk_states.insert(0, last_disk_state)
        return disk_states

    @staticmethod
    def solve_single_disk(current_disk_model: DiskMathModel, distal_disk_state: DiskMathState, input_forces: List[float]):
        disk_geometry = current_disk_model.disk_geometry
        top_tendon_states = distal_disk_state.tendon_states if distal_disk_state else []
        top_contact_joint_angle = distal_disk_state.bottom_joint_angle if distal_disk_state is not None else None

        # Top vectors
        (top_contact_forceDF,
         top_contact_pure_momentDF,
         top_forceDF,
         top_momentDF) = _eval_top_components(current_disk_model, distal_disk_state)

        # bottom tendon states
        (bottom_knobbed_tendon_states,
         bottom_continous_tendon_states,
         bottom_tendon_state_disp_pairs) = _eval_bottom_tendon_guide_components(current_disk_model, distal_disk_state, input_forces)

        # Other constants' computation
        sum_tensions = sum((ts.tension_in_disk)
                           for ts, disp in bottom_tendon_state_disp_pairs)
        sum_bottom_guide_x_moment_y = sum(
            (disp[1] * ts.tension_in_disk) for ts, disp in bottom_tendon_state_disp_pairs)
        sum_bottom_guide_x_moment_z = sum(
            (disp[2] * ts.tension_in_disk) for ts, disp in bottom_tendon_state_disp_pairs)

        # P*sin(joint_angle/2) + Q*cos(joint_angle/2) + R = 0
        # Please refers to the note
        P = (disk_geometry.bottom_curve_radius *
             top_forceDF[2] + (disk_geometry.length/2-disk_geometry.bottom_curve_radius) *
             sum_tensions + sum_bottom_guide_x_moment_z)
        Q = (-disk_geometry.bottom_curve_radius *
             top_forceDF[1] - sum_bottom_guide_x_moment_y)
        R = (top_forceDF[1]*(disk_geometry.bottom_curve_radius -
                             disk_geometry.length/2) + top_momentDF[0])

        # Avoid singularity (division by 0) right away

        half_joint_angle = 0
        if P != 0.0:
            # P*sin(theta) + Q*cos(theta) = +-sqrt(P**2+Q**2)*sin(theta+atan(Q/P))
            # There are 2 possible formulae, mathematically speaking, but only 1 of them complies with the constraint
            # Validated by plotting both solutions to see which matches the original formula
            # OR asin(R/sqrt(P**2 + Q**2)) - atan2(Q,P)
            half_joint_angle = asin(-R/sqrt(P**2 + Q**2)) - atan(Q/P)

            # Singularity occurs when zero tension forces are applied to all tendons
            # (Have not confirmed if there exists other cases causing singularity)
            if isnan(half_joint_angle):
                half_joint_angle = 0

            # To check whether the numerical errors are significant
            # Not sure whether this condition will be true in any case, but at least it is
            # confirmed that the solution must be evaluated from either 2 of these formulae
            elif abs(P*sin(half_joint_angle) + Q*cos(half_joint_angle) + R) > 1**-10:
                half_joint_angle = asin(R/sqrt(P**2 + Q**2)) - atan(Q/P)
                if isnan(half_joint_angle) or abs(P*sin(half_joint_angle) + Q*cos(half_joint_angle) + R) > 1**-10:
                    half_joint_angle = 0
                    print(
                        f"Warning: error is larger than 1^(-10) (Value={P*sin(half_joint_angle) + Q*cos(half_joint_angle) + R}), thus it is set to 0")
                    return None
        bottom_joint_angle = 2*half_joint_angle

        # Store the state related data for next proximal disk bottom joint angle evaluation
        # Force displacement has been evaluated
        bottom_tendon_total_forceDF = np.zeros(3)
        bottom_tendon_force_momentDF = np.zeros(3)

        for ts, disp in bottom_tendon_state_disp_pairs:
            bottom_tendon_force = eval_tendon_guide_bottom_force(
                ts.tension_in_disk, bottom_joint_angle)
            bottom_tendon_total_forceDF += bottom_tendon_force
            bottom_tendon_force_momentDF += np.cross(disp, bottom_tendon_force)

        bottom_contact_forceDF = - top_forceDF - bottom_tendon_total_forceDF
        bottom_contact_pure_momentDF = (-top_momentDF - bottom_tendon_force_momentDF -
                                        np.cross(eval_bottom_contact_disp(disk_geometry.length, disk_geometry.bottom_curve_radius, bottom_joint_angle), bottom_contact_forceDF))

        return DiskMathState(current_disk_model,
                             bottom_knobbed_tendon_states,
                             bottom_continous_tendon_states,
                             bottom_contact_forceDF,
                             bottom_contact_pure_momentDF,
                             bottom_joint_angle,
                             top_contact_forceDF,
                             top_contact_pure_momentDF,
                             top_contact_joint_angle)


class IterativeSolver(_SolverBase):
    def __init__(self, precision=0.0000000001):
        self.precision = precision

    def solve(self, manipulator_model: ManipulatorMathModel, nested_input_forces: List[Iterable[float]]) -> List[DiskMathState]:
        disk_states = []
        last_disk_state = None
        for disk_model, input_forces in manipulator_model.get_reversed_disk_model_input_forces_iterable(nested_input_forces):
            last_disk_state = IterativeSolver.solve_single_disk(
                disk_model, last_disk_state, input_forces, self.precision)
            if last_disk_state is None:
                break
            disk_states.insert(0, last_disk_state)
        return disk_states

    @staticmethod
    def solve_single_disk(current_disk_model: DiskMathModel, distal_disk_state: DiskMathState, input_forces: List[float], precision: float):
        disk_geometry = current_disk_model.disk_geometry
        top_tendon_states = distal_disk_state.tendon_states if distal_disk_state else []
        top_contact_joint_angle = distal_disk_state.bottom_joint_angle if distal_disk_state is not None else None

        # Top vectors
        (top_contact_forceDF,
         top_contact_pure_momentDF,
         top_total_forceDF,
         top_total_momentDF) = _eval_top_components(current_disk_model, distal_disk_state)

        # bottom tendon states
        (bottom_knobbed_tendon_states,
         bottom_continous_tendon_states,
         bottom_tendon_state_disp_pairs) = _eval_bottom_tendon_guide_components(current_disk_model, distal_disk_state, input_forces)

        def __equilibrium(bottom_joint_angle):
            bottom_tendon_force_disp_pairs = [(eval_tendon_guide_bottom_force(
                s.tension_in_disk, bottom_joint_angle), d) for (s, d) in bottom_tendon_state_disp_pairs]
            sum_tendon_bottom_end_force = sum((
                f for f, _ in bottom_tendon_force_disp_pairs), np.zeros(3))
            sum_tendon_bottom_end_torque = sum((
                np.cross(d, f) for f, d in bottom_tendon_force_disp_pairs), np.zeros(3))

            bottom_contact_force = (-(top_total_forceDF +
                                      sum_tendon_bottom_end_force))
            bottom_contact_pure_moment = -(top_total_momentDF + sum_tendon_bottom_end_torque + np.cross(
                eval_bottom_contact_disp(disk_geometry.length, disk_geometry.bottom_curve_radius, bottom_joint_angle), bottom_contact_force))
            return bottom_contact_pure_moment[0], (bottom_contact_force, bottom_contact_pure_moment)

        bottom_joint_angle, res = IterativeSolver.equation_binary_solve(
            __equilibrium, lower=-pi/2, upper=pi/2, init=0, precision=precision)

        if bottom_joint_angle is None:
            return None
        (bottom_contact_forceDF, bottom_contact_pure_momentDF) = res
        return DiskMathState(current_disk_model,
                             bottom_knobbed_tendon_states,
                             bottom_continous_tendon_states,
                             bottom_contact_forceDF,
                             bottom_contact_pure_momentDF,
                             bottom_joint_angle,
                             top_contact_forceDF,
                             top_contact_pure_momentDF,
                             top_contact_joint_angle
                             )

    @staticmethod
    def equation_binary_solve(func, lower, upper, init=None, precision=0.0000000001):
        """
        """
        if func(lower)[0] * func(upper)[0] > 0:
            return None, None
        x = (lower+upper)/2 if init is None else init
        y, res = func(x)
        while abs(y) > precision:
            if y < 0:
                lower = x
            else:
                upper = x
            x = (lower + upper)/2
            y, res = func(x)
        return x, res
