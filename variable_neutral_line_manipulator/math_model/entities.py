from typing import List, Dict, Iterable
import copy
from math import pi
import numpy as np

from ..common.entities import *
from .calculation import *

def to_general_tension_state(tension_state, bottom_joint_angle, top_joint_angle, top_orientationDF):
    return TendonState(
            tension_state.model,
            eval_tendon_guide_bottom_force(
                tension_state.tension_in_disk,
                bottom_joint_angle
            )if bottom_joint_angle is not None else
            None,
            eval_tendon_guide_top_force(
                tension_state.tension_in_disk,
                top_joint_angle,
                top_orientationDF
            ) if top_joint_angle is not None and top_orientationDF is not None else
            None,
        )

class MathTendonPrimitiveState(TendonStateBase):
    """
        Geometric definition of the full tendon model for mathemtaic model computation
        @param:
            orientationMF: Orientation of the tendon around the centre axis of the manipulator in neutral state
            dist_from_axis: Distance from the centre axis of the disk
            is_knob: Is tendon a knob
    """

    def __init__(self, model, tension_in_disk):
        super().__init__(model)
        self._tension_in_disk = tension_in_disk

    @property
    def tension_in_disk(self):
        return self._tension_in_disk

# class DiskMathModel(BaseDataClass):
#     """
#         Geometry definition of the disk model (wihtout tendon) for mathemtaic model computation
#         @param:
#             length: Length of disk
#             bottom_orientationMF: Orientation of the bottom curve of the disk
#             bottom_curve_radius: Radius of bottom curve of the disk
#             top_orientationDF: Orientation of the top curve of the disk relative to the disk frame
#             top_curve_radius: Radius of top curve of the disk
#             tendon_models: math tendon models
#     """

#     def __init__(self, disk_geometry, bottom_orientationMF, knotted_tendon_models=[], continuous_tendon_models=[]):
#         self.disk_geometry: DiskGeometryBase = disk_geometry
#         self.bottom_orientationMF = bottom_orientationMF
#         self.knotted_tendon_models = copy.copy(knotted_tendon_models)
#         self.continuous_tendon_models = copy.copy(continuous_tendon_models)

#     def local_attr_keys(self):
#         return ["disk_geometry", "bottom_orientationMF", "knotted_tendon_models", "continuous_tendon_models"]


# class SegmentMathModel(BaseDataClass):
#     """
#         Math definition of segment
#     """

#     def __init__(self, is_2_DoF, n_joints, disk_length, orientationMF, curve_radius, tendon_dist_from_axis, end_disk_length):
#         self.n_joints = n_joints
#         self.is_2_DoF = is_2_DoF
#         self.disk_length = disk_length
#         self.orientationMF = orientationMF
#         self.curve_radius = curve_radius
#         self.tendon_dist_from_axis = tendon_dist_from_axis
#         self.end_disk_length = end_disk_length

#     @property
#     def num_joints(self):
#         return self.n_joints

#     @property
#     def bottom_orientationMF(self):
#         return self.orientationMF

#     @property
#     def bottom_curve_radius(self):
#         return self.curve_radius

#     @property
#     def knotted_tendon_guide_modelsMF(self):
#         return [TendonInDiskMathModel(orientationMF=self.orientationMF + relative_orientation, dist_from_axis=self.tendon_dist_from_axis)
#                 for relative_orientation in ((pi/2, 3*pi/2) if not self.is_2_DoF else (0, pi/2, pi, 3*pi/2))]

#     def generate_disk_math_models(self, end_top_orientationMF=0.0, end_top_curve_radius=None, distal_segment_tendon_models=[]):
#         return indices_entity_pairs_to_ordered_list(self.generate_disk_math_model_indices(end_top_orientationMF=end_top_orientationMF,
#                                                                                           end_top_curve_radius=end_top_curve_radius,
#                                                                                           distal_segment_tendon_models=distal_segment_tendon_models))

#     def generate_disk_math_model_indices(self, end_top_orientationMF=0.0, end_top_curve_radius=None, distal_segment_tendon_models=[]):
#         res = []
#         knotted_tendon_guide_models_at_end = self.knotted_tendon_guide_modelsMF
#         all_tendon_guide_models = (knotted_tendon_guide_models_at_end +
#                                    distal_segment_tendon_models)
#         # 1 DoF
#         if not self.is_2_DoF:

#             # Whether the last disk is identical to any intermediate disk. If true, merge it with other disk
#             if self.n_joints > 1:
#                 res.append((tuple(i for i in range(self.n_joints - 1)),
#                             DiskMathModel(
#                     bottom_orientationMF=self.orientationMF,
#                     disk_geometry=DiskGeometryBase(length=self.disk_length,
#                                                    bottom_curve_radius=self.curve_radius,
#                                                    top_orientationDF=0,
#                                                    top_curve_radius=self.curve_radius,
#                                                    ),
#                     continuous_tendon_models=all_tendon_guide_models
#                 )))

#             res.append(((self.n_joints-1, ), DiskMathModel(
#                 bottom_orientationMF=self.orientationMF,
#                 disk_geometry=DiskGeometryBase(length=end_disk_length if end_disk_length else self.disk_length,
#                                                bottom_curve_radius=self.curve_radius,
#                                                top_orientationDF=end_top_orientationMF,
#                                                top_curve_radius=end_top_curve_radius,),
#                 knotted_tendon_models=knotted_tendon_guide_models_at_end,
#                 continuous_tendon_models=distal_segment_tendon_models
#             )))

#         # 2 DoF
#         else:
#             # Whether the last disk is identical to any one of the intermediate disks. If true, merge it with other disk

#             if self.n_joints > 1:
#                 intermediate_disk_model = DiskMathModel(
#                     bottom_orientationMF=self.orientationMF,
#                     disk_geometry=DiskGeometryBase(length=self.disk_length,
#                                                    bottom_curve_radius=self.curve_radius,
#                                                    top_orientationDF=pi/2,
#                                                    top_curve_radius=self.curve_radius,),

#                     continuous_tendon_models=all_tendon_guide_models
#                 )

#                 res.append(
#                     (tuple(i for i in range(0, self.n_joints - 1, 2)), intermediate_disk_model))

#             if self.n_joints > 2:
#                 intermediate_disk_model = DiskMathModel(
#                     bottom_orientationMF=self.orientationMF + pi/2,
#                     disk_geometry=DiskGeometryBase(length=self.disk_length,
#                                                    bottom_curve_radius=self.curve_radius,
#                                                    top_orientationDF=-pi/2,
#                                                    top_curve_radius=self.curve_radius,),

#                     continuous_tendon_models=all_tendon_guide_models
#                 )
#                 res.append((tuple(i for i in range(1, self.n_joints - 1, 2)),
#                             intermediate_disk_model))

#             disk_orientationMF = (self.orientationMF +
#                                   (pi/2 if self.n_joints % 2 == 0 else 0))

#             end_disk_model = DiskMathModel(
#                 bottom_orientationMF=disk_orientationMF,
#                 disk_geometry=DiskGeometryBase(length=self.end_disk_length if self.end_disk_length else self.disk_length,
#                                                bottom_curve_radius=self.curve_radius,
#                                                top_orientationDF=end_top_orientationMF - disk_orientationMF,
#                                                top_curve_radius=end_top_curve_radius,),
#                 knotted_tendon_models=knotted_tendon_guide_models_at_end,
#                 continuous_tendon_models=distal_segment_tendon_models
#             )

#             res.append(((self.n_joints-1, ), end_disk_model))

#         return res

#     def local_attr_keys(self):
#         return ["is_2_DoF", "n_joints", "disk_length", "orientationMF", "curve_radius", "tendon_dist_from_axis",
#                 "end_disk_length"]


# class ManipulatorMathModel(BaseDataClass):
#     def __init__(self, segments):
#         self.segments = segments
#         self.n_joints = sum(segment.num_joints for segment in segments)
#         self.indices_disk_model_pair = []
#         self.generate()

#     def generate(self):
#         segments = self.segments
#         self.indices_disk_model_pair = []
#         distal_disk_tendon_guide_modelsMF = []

#         n_joints_count = self.n_joints

#         end_top_orientationMF = 0
#         end_top_curve_radius = None

#         for segment in reversed(segments):
#             disk_model_indices_start_from_segment = segment.generate_disk_math_model_indices(
#                 end_top_orientationMF, end_top_curve_radius, distal_disk_tendon_guide_modelsMF)
#             n_joints_count -= segment.num_joints
#             self.indices_disk_model_pair = [(tuple((i + n_joints_count) for i in indices), disk_model)
#                                             for indices, disk_model in disk_model_indices_start_from_segment] + self.indices_disk_model_pair

#             # Update state
#             distal_disk_tendon_guide_modelsMF += segment.knotted_tendon_guide_modelsMF
#             end_top_orientationMF = segment.bottom_orientationMF
#             end_top_curve_radius = segment.bottom_curve_radius

#     def get_reversed_disk_model_input_forces_iterable(self, nested_input_forces):
#         disk_model_flatten_list = [0]*self.n_joints
#         for indices, disk_model in self.indices_disk_model_pair:
#             for i in indices:
#                 disk_model_flatten_list[i] = disk_model

#         for disk_model in reversed(disk_model_flatten_list):
#             if len(disk_model.knotted_tendon_models) > 0:
#                 input_forces = nested_input_forces[-1]
#                 nested_input_forces = nested_input_forces[:-1]
#                 yield disk_model, input_forces
#             else:
#                 yield disk_model, None


# class TendonMathState(BaseDataClass):
#     def __init__(self, tendon_model: TendonInDiskMathModel, tension_in_disk: float):
#         self.model: TendonInDiskMathModel = tendon_model
#         self.tension_in_disk = tension_in_disk

#     def local_attr_keys(self):
#         return ["model", "tension_in_disk", ]


# class DiskMathState(BaseDataClass):
#     def __init__(self,
#                  disk_model: DiskMathModel,
#                  knotted_tendon_states: List[TendonMathState],
#                  continuous_tendon_states: List[TendonMathState],
#                  bottom_contact_forceDF: np.ndarray,
#                  bottom_contact_pure_momentDF: np.ndarray,
#                  bottom_joint_angle: float,
#                  top_contact_forceDF: np.ndarray = None,
#                  top_contact_pure_momentDF: np.ndarray = None,
#                  top_joint_angle: float = None):
#         self.disk_model: DiskMathModel = disk_model
#         self.knotted_tendon_states: List[TendonMathState] = copy.copy(
#             knotted_tendon_states)
#         self.continuous_tendon_states: List[TendonMathState] = copy.copy(
#             continuous_tendon_states)
#         self.bottom_contact_forceDF = bottom_contact_forceDF
#         self.bottom_contact_pure_momentDF = bottom_contact_pure_momentDF
#         self.bottom_joint_angle = bottom_joint_angle
#         self.top_contact_forceDF = top_contact_forceDF
#         self.top_contact_pure_momentDF = top_contact_pure_momentDF
#         self.top_joint_angle = top_joint_angle

#     @property
#     def tendon_guide_top_end_state_force_disp_tuple(self):
#         return [
#             (ts,
#              eval_tendon_guide_top_force(
#                  ts.tension_in_disk,
#                  self.top_joint_angle,
#                  self.disk_model.disk_geometry.top_orientationDF

#              ), eval_tendon_guide_top_end_disp(
#                  self.disk_model.disk_geometry.length,
#                  self.disk_model.disk_geometry.top_curve_radius,
#                  ts.model.dist_from_axis,
#                  ts.model.orientationMF - self.disk_model.bottom_orientationMF,
#                  self.disk_model.disk_geometry.top_orientationDF
#              )
#              ) for ts in self.continuous_tendon_states
#         ]

#     @property
#     def tendon_guide_bottom_end_state_force_disp_tuple(self):
#         return [
#             (ts,
#              eval_tendon_guide_bottom_force(
#                  ts.tension_in_disk,
#                  self.bottom_joint_angle,
#              ), eval_tendon_guide_bottom_end_disp(
#                  self.disk_model.disk_geometry.length,
#                  self.disk_model.disk_geometry.bottom_curve_radius,
#                  ts.model.dist_from_axis,
#                  ts.model.orientationMF - self.disk_model.bottom_orientationMF,
#              )
#              ) for ts in self.tendon_states
#         ]

#     @property
#     def top_contact_force_pure_moment_disp_tuple(self):
#         if self.disk_model.disk_geometry.top_curve_radius is None or self.disk_model.disk_geometry.top_curve_radius <= 0.0:
#             return (np.zeros(3), np.zeros(3), np.zeros(3))
#         return (
#             self.top_contact_forceDF,
#             self.top_contact_pure_momentDF,
#             eval_top_contact_disp(self.disk_model.disk_geometry.length,
#                                   self.disk_model.disk_geometry.top_curve_radius,
#                                   self.top_joint_angle,
#                                   self.disk_model.disk_geometry.top_orientationDF)
#         )

#     @property
#     def bottom_contact_force_pure_moment_disp_tuple(self):
#         return (
#             self.bottom_contact_forceDF,
#             self.bottom_contact_pure_momentDF,
#             eval_bottom_contact_disp(self.disk_model.disk_geometry.length,
#                                      self.disk_model.disk_geometry.bottom_curve_radius,
#                                      self.bottom_joint_angle)
#         )

#     @property
#     def sum_of_forces(self):
#         return (sum((eval_tendon_guide_top_force(
#             ts.tension_in_disk,
#             self.top_joint_angle,
#             self.disk_model.disk_geometry.top_orientationDF
#         ) for ts in self.continuous_tendon_states), np.zeros(3)) +
#             sum((eval_tendon_guide_bottom_force(
#                 ts.tension_in_disk,
#                 self.bottom_joint_angle,
#             ) for ts in self.tendon_states), np.zeros(3)) +
#             self.top_contact_forceDF +
#             self.bottom_contact_forceDF)

#     @property
#     def sum_of_moments(self):
#         top_contact_force, top_contact_moment, top_contact_disp = self.top_contact_force_pure_moment_disp_tuple
#         bottom_contact_force, bottom_contact_moment, bottom_contact_disp = self.bottom_contact_force_pure_moment_disp_tuple

#         return (sum((np.cross(r, f)
#                      for _, f, r in self.tendon_guide_top_end_state_force_disp_tuple), np.zeros(3)) +
#                 sum((np.cross(r, f)
#                      for _, f, r in self.tendon_guide_bottom_end_state_force_disp_tuple), np.zeros(3)) +
#                 np.cross(top_contact_disp, top_contact_force) +
#                 top_contact_moment +
#                 np.cross(bottom_contact_disp, bottom_contact_force) +
#                 bottom_contact_moment)

#     @ property
#     def tendon_states(self):
#         """
#         Get all tendon states.
#         Used for proximal disk's bottom joint angle evaluation
#         """
#         return self.knotted_tendon_states + self.continuous_tendon_states

#     def local_attr_keys(self):
#         return ["disk_model", "knotted_tendon_states", "continuous_tendon_states",
#                 "bottom_contact_forceDF", "bottom_contact_pure_momentDF", "bottom_joint_angle",
#                 "top_contact_forceDF", "top_contact_pure_momentDF", "top_joint_angle"]


# """ TODO"""
# # class ManipulatorState:
# #     def __init__(self, manipulator_model:ManipulatorMathModel,
# #                  tension_inputs:List,
# #                  disk_states:List[DiskModelState]):
# #         super().__init__()
# #         self.model = manipulator_model
# #         self.tension_inputs = tension_inputs
# #         self.disk_states:List[DiskModelState] = disk_states

# #         self.TFs_DF = []
# #         self._generate_TFs()

# #     def _generate_TFs(self):
# #         tf = np.identity(4)
# #         self.TFs_DF.append(tf)

# #         # len(self.model.disks) = len(self.disk_states) + 1
# #         for d, s in zip_longest(self.model.disks, self.disk_states, fillvalue=None):
# #             if s is None:
# #                 break
# #             tf = np.matmul(tf, np.matmul(m4_translation((0.0,0, d.length)),
# #                           m4_rotation((0,0,1.0), d.top_orientationBF - d.bottom_orientationBF)))
# #             tf = np.matmul(tf, eval_proximal_top_to_distal_bottom_TF(s.bottom_joint_angle, d.top_curve_radius))

# #             self.TFs_DF.append(tf)


# #     def get_TF(self, disk_index, side, frame_sys):
# #         """
# #             Get the transformation matrix from the top of the base frame
# #             side = "b": bottom, "c": center, "t": top
# #             frame_sys = "bd": base-disk-orientation, "bc": bottom-curvature-orientation, "tc": top-curvature-orientation
# #         """
# #         length = self.model.disks[disk_index].length
# #         tf = self.TFs_DF[disk_index]

# #         if side == "b":
# #             pass
# #         elif side == "c":
# #             tf = np.matmul(tf, m4_translation((0,0, length/2 )))
# #         elif side == "t":
# #             tf = np.matmul(tf, m4_translation((0,0, length )))
# #         else:
# #             raise AttributeError(f"'side' must not be {side}, but either 'b', 'c' or 't'")

# #         if frame_sys == "bc":
# #             pass
# #         elif frame_sys == "bd":
# #             orientation_change = -self.model.disks[disk_index].bottom_orientationBF
# #             return np.matmul(tf,m4_rotation((0,0,1.0), orientation_change))
# #         elif frame_sys == "tc":
# #             orientation_change = (self.model.disks[disk_index].top_orientationBF
# #                                   - self.model.disks[disk_index].bottom_orientationBF)
# #             return np.matmul(tf, m4_rotation((0,0,1.0), orientation_change))
# #         else:
# #             raise AttributeError(f"'frame' must not be {frame_sys}, but either 'bd', 'bc' or 'tc'")
# #         return tf
