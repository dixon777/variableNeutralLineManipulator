import os
import copy
import json
from math import pi
from abc import ABC
from typing import List, Dict, Iterable, Tuple
import numpy as np

from ..util.calculation import m4_translation, m4_rotation
from .calculation import eval_proximal_top_to_distal_bottom_TF
from ..util import BaseDataClass, indices_entity_pairs_to_ordered_list, normalise_angle, normalise_to_range, normalise_half_angle
# Courtesy to dan_waterworth
# https://stackoverflow.com/questions/4544630/automatically-growing-lists-in-python


class BaseDiskGeometry(BaseDataClass):
    """
        Basic definition of the disk model (wihtout tendon) for mathemtaic model computation
        @param:
            length: Length of disk
            bottom_orientationMF: Orientation of the bottom curve of the disk
            bottom_curve_radius: Radius of bottom curve of the disk
            top_orientationDF: Orientation of the top curve of the disk relative to the disk frame
            top_curve_radius: Radius of top curve of the disk
    """

    def __init__(self, length, bottom_curve_radius=None, top_orientationDF=None, top_curve_radius=None):
        self.length = length
        self.bottom_curve_radius = bottom_curve_radius
        self.top_orientationDF = normalise_angle(top_orientationDF)
        self.top_curve_radius = top_curve_radius

    @property
    def top_orientationDF_normalised_copy(self):
        import copy
        copied = copy.deepcopy(self)
        copied.top_orientationDF = normalise_angle(self.top_orientationDF)
        return copied

    def top_orientationMF(self, disk_orientationMF):
        return self.top_orientationDF + disk_orientationMF

    def local_attr_keys(self):
        return ["length", "bottom_curve_radius",
                "top_orientationDF", "top_curve_radius"]

    def local_eq_attr_keys(self):
        return ["length", "bottom_curve_radius", "top_curve_radius"]

    def __eq__(self, other):
        return (super().__eq__(other) and
                (normalise_half_angle(self.top_orientationDF,) == normalise_half_angle(other.top_orientationDF) if self.top_curve_radius
                 else True))  # Meaningless to check the top_orientationDF if top_curve_radius is None or 0 (top_curve_radius has been checked)


class TendonModel(BaseDataClass):
    """
        Base geometric definition of the tendon model
        @param:
            orientationMF: Orientation of the tendon around the centre axis of the manipulator in neutral state
            dist_from_axis: Distance from the centre axis of the disk
    """

    def __init__(self, dist_from_axis, orientation):
        self.dist_from_axis = dist_from_axis
        self.orientation = normalise_angle(orientation)

    def local_attr_keys(self):
        return ["dist_from_axis", "orientation"]

    def local_eq_attr_keys(self):
        return ["dist_from_axis"]

    def __eq__(self, other):
        return super().__eq__(other) and normalise_angle(self.orientation) == normalise_angle(other.orientation)

    def __hash__(self):
        return hash(normalise_angle(self.orientation))


class DiskModel(BaseDataClass):
    """
        Geometry definition of the disk model (wihtout tendon) for mathemtaic model computation
        @param:
            length: Length of disk
            bottom_orientationMF: Orientation of the bottom curve of the disk
            bottom_curve_radius: Radius of bottom curve of the disk
            top_orientationDF: Orientation of the top curve of the disk relative to the disk frame
            top_curve_radius: Radius of top curve of the disk
            tendon_models: math tendon models
    """

    def __init__(self, disk_geometry, bottom_orientationMF, knotted_tendon_models=[], continuous_tendon_models=[]):
        self.disk_geometry: BaseDiskGeometry = disk_geometry
        self.bottom_orientationMF = normalise_angle(bottom_orientationMF)
        self.knotted_tendon_models = copy.copy(knotted_tendon_models)
        self.continuous_tendon_models = copy.copy(continuous_tendon_models)

    @property
    def tendon_models(self):
        return self.knotted_tendon_models + self.continuous_tendon_models

    def local_attr_keys(self):
        return ["disk_geometry", "bottom_orientationMF", "knotted_tendon_models", "continuous_tendon_models"]


class BaseSegmentModel(BaseDataClass):
    @property
    def num_joints(self):
        return 0

    @property
    def bottom_orientationMF(self):
        return 0.0

    @property
    def bottom_curve_radius(self):
        return None
    
    
    def generate_base_disk_model(self, distal_tendon_models=[], length=None):
        pass
    
    def generate_indices_disk_model_pairs(self,
                                          end_top_orientationMF=0.0,
                                          end_top_curve_radius=None,
                                          distal_segment_tendon_models=[]):
        return []
    
    def get_knotted_tendon_modelsMF(self, distal_segment_tendon_models=[]):
        return []
    
class TwoDoFSegmentModel(BaseSegmentModel):
    def __init__(self, n_joints, disk_length, curve_radius, end_disk_length, base_orientationMF=0.0, distal_orientationDF=0.0, tendon_models:List[TendonModel]=[]):
        self.n_joints = n_joints
        self.disk_length = disk_length
        self.curve_radius = curve_radius
        self.end_disk_length = end_disk_length
        self.base_orientationMF = normalise_angle(base_orientationMF)
        self.distal_orientationDF = normalise_angle(distal_orientationDF)
        self.tendon_models = tendon_models
        
    @property
    def num_joints(self):
        return self.n_joints

    @property
    def bottom_orientationMF(self):
        return self.base_orientationMF

    @property
    def bottom_curve_radius(self):
        return self.curve_radius
    
    def get_knotted_tendon_modelsMF(self, distal_segment_tendon_models=[]):
        return sorted([t for t in self.tendon_models if t not in distal_segment_tendon_models],
                      key=lambda x: x.orientation)
        
    def generate_base_disk_model(self, distal_tendon_models=[], length=None):
        all_tendon_models = self.get_knotted_tendon_modelsMF(
            distal_tendon_models) + distal_tendon_models
        return DiskModel(
            bottom_orientationMF=self.base_orientationMF,
            disk_geometry=BaseDiskGeometry(length=length if length else self.disk_length,
                                           bottom_curve_radius=None,
                                           top_orientationDF=0,
                                           top_curve_radius=self.curve_radius),
            continuous_tendon_models=all_tendon_models,)
        
    def generate_indices_disk_model_pairs(self,
                                          end_top_orientationMF=0.0,
                                          end_top_curve_radius=None,
                                          distal_segment_tendon_models=[]):
        res = []
        knotted_tendon_modelsMF = self.get_knotted_tendon_modelsMF(
            distal_segment_tendon_models)
        all_tendon_models = (knotted_tendon_modelsMF +
                             distal_segment_tendon_models)
        if self.n_joints > 1:
            res.append((tuple(i for i in range(0, self.n_joints - 1, 1 if self.distal_orientationDF == 0 else 2)),
                        DiskModel(
                bottom_orientationMF=self.base_orientationMF,
                disk_geometry=BaseDiskGeometry(length=self.disk_length,
                                               bottom_curve_radius=self.curve_radius,
                                               top_orientationDF=self.distal_orientationDF,
                                               top_curve_radius=self.curve_radius,
                                               ),
                continuous_tendon_models=all_tendon_models
            )))

            if self.distal_orientationDF != 0:
                res.append((tuple(i for i in range(1, self.n_joints - 1, 2)),
                            DiskModel(
                    bottom_orientationMF=self.base_orientationMF+self.distal_orientationDF,
                    disk_geometry=BaseDiskGeometry(length=self.disk_length,
                                                   bottom_curve_radius=self.curve_radius,
                                                   top_orientationDF=-self.distal_orientationDF,
                                                   top_curve_radius=self.curve_radius,
                                                   ),
                    continuous_tendon_models=all_tendon_models
                ))) 
                
        distal_disk_base_orientationMF = (self.base_orientationMF +
                                          (self.distal_orientationDF if self.n_joints % 2 == 0 else 0))
        res.append(((self.n_joints-1, ), DiskModel(
            bottom_orientationMF=distal_disk_base_orientationMF,
            disk_geometry=BaseDiskGeometry(length=self.end_disk_length if self.end_disk_length else self.disk_length,
                                           bottom_curve_radius=self.curve_radius,
                                           top_orientationDF=end_top_orientationMF-distal_disk_base_orientationMF,
                                           top_curve_radius=end_top_curve_radius,),
            knotted_tendon_models=knotted_tendon_modelsMF,
            continuous_tendon_models=distal_segment_tendon_models
        )))
        return res
    
    def local_attr_keys(self):
        return ["n_joints", "disk_length", "curve_radius",
                "end_disk_length", "base_orientationMF", "distal_orientationDF"]
        
class TwoDOFParallelSegmentModel(BaseSegmentModel):
    def __init__(self, n_joints, disk_length, curve_radius, tendon_dist_from_axis, end_disk_length, base_orientationMF=0.0, distal_orientationDF=0.0):
        self.n_joints = n_joints
        self.disk_length = disk_length
        self.curve_radius = curve_radius
        self.tendon_dist_from_axis = tendon_dist_from_axis
        self.end_disk_length = end_disk_length
        self.base_orientationMF = normalise_angle(base_orientationMF)
        self.distal_orientationDF = normalise_angle(distal_orientationDF)

    @property
    def num_joints(self):
        return self.n_joints

    @property
    def bottom_orientationMF(self):
        return self.base_orientationMF

    @property
    def bottom_curve_radius(self):
        return self.curve_radius

    def get_knotted_tendon_modelsMF(self, distal_segment_tendon_models=[]):
        """
        Generate all the tendon models controlling this segment. 
        Note that if there is any overlapping tendon harnessing this segment and any distal segment (Same manipulator frame orientation and distance), respectively
        both tendons are merged into one and such tendon is consideredd to be the knotted tendon of the respective distal segment instead of this segment.
        """
        all_possible_tendon_models = [TendonModel(orientation=self.base_orientationMF + relative_orientation,
                                                  dist_from_axis=self.tendon_dist_from_axis)
                                      for relative_orientation in
                                      ((pi/2, 3*pi/2) + (()
                                                         if self.distal_orientationDF == 0 else
                                                         (self.distal_orientationDF+pi/2, self.distal_orientationDF+3*pi/2)))]
        return sorted([t for t in all_possible_tendon_models if t not in distal_segment_tendon_models],
                      key=lambda x: x.orientation)

    def generate_base_disk_model(self, distal_tendon_models=[], length=None):
        disk_orientationMF = self.base_orientationMF
        all_tendon_models = self.get_knotted_tendon_modelsMF(
            distal_tendon_models) + distal_tendon_models
        return DiskModel(
            bottom_orientationMF=disk_orientationMF,
            disk_geometry=BaseDiskGeometry(length=length if length else self.disk_length,
                                           bottom_curve_radius=None,
                                           top_orientationDF=0,
                                           top_curve_radius=self.curve_radius),
            continuous_tendon_models=all_tendon_models,)

    def generate_disk_models(self, end_top_orientationMF=0.0, end_top_curve_radius=None, distal_segment_tendon_models=[]):
        return indices_entity_pairs_to_ordered_list(
            self.generate_indices_disk_model_pairs(
                end_top_orientationMF=end_top_orientationMF,
                end_top_curve_radius=end_top_curve_radius,
                distal_segment_tendon_models=distal_segment_tendon_models
            )
        )

    def generate_indices_disk_model_pairs(self,
                                          end_top_orientationMF=0.0,
                                          end_top_curve_radius=None,
                                          distal_segment_tendon_models=[]):
        res = []
        knotted_tendon_modelsMF = self.get_knotted_tendon_modelsMF(
            distal_segment_tendon_models)
        all_tendon_models = (knotted_tendon_modelsMF +
                             distal_segment_tendon_models)

        if self.n_joints > 1:
            res.append((tuple(i for i in range(0, self.n_joints - 1, 1 if self.distal_orientationDF == 0 else 2)),
                        DiskModel(
                bottom_orientationMF=self.base_orientationMF,
                disk_geometry=BaseDiskGeometry(length=self.disk_length,
                                               bottom_curve_radius=self.curve_radius,
                                               top_orientationDF=self.distal_orientationDF,
                                               top_curve_radius=self.curve_radius,
                                               ),
                continuous_tendon_models=all_tendon_models
            )))

            if self.distal_orientationDF != 0:
                res.append((tuple(i for i in range(1, self.n_joints - 1, 2)),
                            DiskModel(
                    bottom_orientationMF=self.base_orientationMF+self.distal_orientationDF,
                    disk_geometry=BaseDiskGeometry(length=self.disk_length,
                                                   bottom_curve_radius=self.curve_radius,
                                                   top_orientationDF=-self.distal_orientationDF,
                                                   top_curve_radius=self.curve_radius,
                                                   ),
                    continuous_tendon_models=all_tendon_models
                )))

        distal_disk_base_orientationMF = (self.base_orientationMF +
                                          (self.distal_orientationDF if self.n_joints % 2 == 0 else 0))
        res.append(((self.n_joints-1, ), DiskModel(
            bottom_orientationMF=distal_disk_base_orientationMF,
            disk_geometry=BaseDiskGeometry(length=self.end_disk_length if self.end_disk_length else self.disk_length,
                                           bottom_curve_radius=self.curve_radius,
                                           top_orientationDF=end_top_orientationMF-distal_disk_base_orientationMF,
                                           top_curve_radius=end_top_curve_radius,),
            knotted_tendon_models=knotted_tendon_modelsMF,
            continuous_tendon_models=distal_segment_tendon_models
        )))
        return res

    def local_attr_keys(self):
        return ["n_joints", "disk_length", "curve_radius", "tendon_dist_from_axis",
                "end_disk_length", "base_orientationMF", "distal_orientationDF"]


class ManipulatorModel(BaseDataClass):
    def __init__(self, segments:List[BaseSegmentModel], outer_diameter:float=None):
        self.segments = segments
        self.outer_diameter = outer_diameter
        self.n_joints = sum(segment.num_joints for segment in segments)
        self.indices_disk_model_pairs = []
        self.generate()

    def generate(self):
        segments = self.segments
        self.indices_disk_model_pairs = []

        n_joints_count = self.n_joints

        end_top_orientationMF = 0
        end_top_curve_radius = None
        distal_disk_tendon_modelsMF = []

        for i, segment in enumerate(reversed(segments)):
            segment_only_indices_disk_model_pairs = segment.generate_indices_disk_model_pairs(
                end_top_orientationMF, end_top_curve_radius, distal_disk_tendon_modelsMF)
            n_joints_count -= segment.num_joints
            self.indices_disk_model_pairs = (
                [(tuple((i + n_joints_count + 1) for i in indices), disk_model)  # +1 due to base disk
                 for indices, disk_model in segment_only_indices_disk_model_pairs]
                + self.indices_disk_model_pairs)

            if i == len(segments)-1:
                break

            # Update state
            distal_disk_tendon_modelsMF += segment.get_knotted_tendon_modelsMF(
                distal_disk_tendon_modelsMF)
            end_top_orientationMF = segment.bottom_orientationMF
            end_top_curve_radius = segment.bottom_curve_radius

        self.indices_disk_model_pairs.insert(
            0, ((0,), segments[0].generate_base_disk_model(distal_disk_tendon_modelsMF)))

    @property
    def disk_models(self):
        return indices_entity_pairs_to_ordered_list(
            self.indices_disk_model_pairs
        )

    @property
    def num_joints(self):
        return sum(s.num_joints for s in self.segments)

    @property
    def tendon_models(self):
        return sorted(set(tm for s in self.segments for tm in s.get_knotted_tendon_modelsMF()), key=lambda x: x.orientation)

    def get_disk_model(self, index) -> DiskModel:
        return self.disk_models[index]

    def get_indices_disk_model_pairs(self, include_base: bool):
        return [
            ((i if include_base else i-1 for i in indices), model)
            for indices, model in self.indices_disk_model_pairs[0 if include_base else 1:]
        ]

    def get_disk_models(self, include_base: bool):
        return indices_entity_pairs_to_ordered_list(
            indices_entity_pairs=self.get_indices_disk_model_pairs(
                include_base)
        )

    def get_tendon_model_to_input_force_map(self, applied_tensions: List[float]):
        return {tendon_model: input_force for tendon_model, input_force in zip(self.tendon_models, applied_tensions)}

    def get_reversed_disk_model_applied_tensions_iterable(self, applied_tensions: List[float], include_base: bool):
        tendon_model_to_input_force_map = self.get_tendon_model_to_input_force_map(
            applied_tensions)
        disk_models = self.get_disk_models(include_base)

        for disk_model in reversed(disk_models):
            if len(disk_model.knotted_tendon_models) > 0:
                yield disk_model, [tendon_model_to_input_force_map[tendon_model] for tendon_model in disk_model.knotted_tendon_models]
            else:
                yield disk_model, None

    def is_input_force_valid(self, applied_tensions):
        tendon_models = self.tendon_models
        if len(applied_tensions) != len(tendon_models):
            print(
                f"Error: There should be {len(tendon_models)} force components, but you only input {len(applied_tensions)} components")
            return False

        return True
    
    def local_attr_keys(self):
        return ["segments", ]


class TendonStateBase(BaseDataClass):
    def __init__(self, model: TendonModel):
        self.model: TendonModel = model

    @property
    def tension_in_disk(self):
        return 0

    def local_attr_keys(self):
        return ["model", "tension_in_disk"]


class TendonState(TendonStateBase):
    def __init__(self, model: TendonModel,
                 bottom_tensionDF: np.ndarray,
                 top_tensionDF: np.ndarray):
        super().__init__(model)
        self.bottom_tensionDF = np.array(
            bottom_tensionDF) if bottom_tensionDF is not None else None
        self.top_tensionDF = np.array(
            top_tensionDF) if top_tensionDF is not None else None

    @property
    def tension_in_disk(self):
        return (np.linalg.norm(self.bottom_tensionDF)
                if self.bottom_tensionDF is not None else
                np.linalg.norm(self.top_tensionDF)
                if self.top_tensionDF is not None else
                0)

    def local_attr_keys(self):
        return ["bottom_tensionDF", "top_tensionDF"]


class DiskState(BaseDataClass):
    def __init__(self,
                 disk_model: DiskModel,
                 bottom_contact_forceDF: np.ndarray,
                 bottom_contact_pure_momentDF: np.ndarray,
                 bottom_joint_angle: float,
                 knotted_tendon_states: List[TendonState] = [],
                 continuous_tendon_states: List[TendonState] = [],
                 top_contact_forceDF: np.ndarray = None,
                 top_contact_pure_momentDF: np.ndarray = None,
                 top_joint_angle: float = None):
        self.disk_model: DiskModel = disk_model
        self.knotted_tendon_states: List[TendonState] = copy.copy(
            knotted_tendon_states)
        self.continuous_tendon_states: List[TendonState] = copy.copy(
            continuous_tendon_states)
        self.bottom_contact_forceDF = bottom_contact_forceDF
        self.bottom_contact_pure_momentDF = bottom_contact_pure_momentDF
        self.bottom_joint_angle = bottom_joint_angle
        self.top_contact_forceDF = top_contact_forceDF
        self.top_contact_pure_momentDF = top_contact_pure_momentDF
        self.top_joint_angle = top_joint_angle

    @property
    def has_bottom_contact(self):
        return self.bottom_contact_forceDF is not None

    @property
    def has_top_contact(self):
        return self.top_contact_forceDF is not None

    @property
    def bottom_contact_force_and_pure_momentDF(self):
        return list(self.bottom_contact_forceDF) + list(self.bottom_contact_pure_momentDF)

    @property
    def top_contact_force_and_pure_momentDF(self):
        return list(self.top_contact_forceDF) + list(self.top_contact_pure_momentDF)

    @property
    def tendon_states(self):
        """
        Get all tendon states.
        Used for proximal disk's bottom joint angle evaluation
        """
        return self.knotted_tendon_states + self.continuous_tendon_states

    def local_attr_keys(self):
        return ["disk_model",
                "knotted_tendon_states",
                "continuous_tendon_states",
                "bottom_contact_forceDF",
                "bottom_contact_pure_momentDF",
                "bottom_joint_angle",
                "top_contact_forceDF",
                "top_contact_pure_momentDF",
                "top_joint_angle"]


class ManipulatorState(BaseDataClass):
    def __init__(self,
                 manipulator_model: ManipulatorModel,
                 applied_tensions: List[float],
                 disk_states: List[DiskState],
                 tip_disp: np.ndarray = None):
        self.manipulator_model = manipulator_model
        self.applied_tensions = applied_tensions
        self.disk_states = disk_states
        self._tip_disp = tip_disp

    @property
    def tendon_model_to_input_force_map(self):
        return self.manipulator_model.get_tendon_model_to_input_force_map(applied_tensions)

    def local_attr_keys(self):
        return ["manipulator_model",
                "applied_tensions",
                "disk_states", ]

    def get_TF(self, disk_index, pos="bottom"):
        """
            Get TF generated from joint angles
            (The model adheres to rigid body assumption)
        """
        m = np.identity(4)
        for i in range(disk_index):
            disk_geometry = self.disk_states[i].disk_model.disk_geometry
            m = np.matmul(m, m4_translation((0, 0, disk_geometry.length)))
            m = np.matmul(m, m4_rotation(
                (0, 0, 1), disk_geometry.top_orientationDF))
            m = np.matmul(m, eval_proximal_top_to_distal_bottom_TF(
                self.disk_states[i].top_joint_angle,
                disk_geometry.top_curve_radius))
            

        if pos == "top":
            m = np.matmul(m, m4_translation(
                (0, 0, self.disk_states[disk_index].disk_model.disk_geometry.length)))
        elif pos == "centre":
            m = np.matmul(m, m4_translation(
                (0, 0, self.disk_states[disk_index].disk_model.disk_geometry.length/2)))

        return m

    @property
    def tip_disp(self):
        return (self._tip_disp if self._tip_disp is not None else self.get_TF(self.manipulator_model.num_joints, "top")[:3,3])

    # @property
    # def tendon_guide_top_end_state_force_disp_tuple(self):
    #     return [
    #         (ts,
    #          eval_tendon_guide_top_force(
    #              ts.tension_in_disk,
    #              self.top_joint_angle,
    #              self.disk_model.disk_geometry.top_orientationDF

    #          ), eval_tendon_guide_top_end_disp(
    #              self.disk_model.disk_geometry.length,
    #              self.disk_model.disk_geometry.top_curve_radius,
    #              ts.model.dist_from_axis,
    #              ts.model.orientationMF - self.disk_model.bottom_orientationMF,
    #              self.disk_model.disk_geometry.top_orientationDF
    #          )
    #          ) for ts in self.continuous_tendon_states
    #     ]

    # @property
    # def tendon_guide_bottom_end_state_force_disp_tuple(self):
    #     return [
    #         (ts,
    #          eval_tendon_guide_bottom_force(
    #              ts.tension_in_disk,
    #              self.bottom_joint_angle,
    #          ), eval_tendon_guide_bottom_end_disp(
    #              self.disk_model.disk_geometry.length,
    #              self.disk_model.disk_geometry.bottom_curve_radius,
    #              ts.model.dist_from_axis,
    #              ts.model.orientationMF - self.disk_model.bottom_orientationMF,
    #          )
    #          ) for ts in self.tendon_states
    #     ]

    # @property
    # def top_contact_force_pure_moment_disp_tuple(self):
    #     if self.disk_model.disk_geometry.top_curve_radius is None or self.disk_model.disk_geometry.top_curve_radius <= 0.0:
    #         return (np.zeros(3), np.zeros(3), np.zeros(3))
    #     return (
    #         self.top_contact_forceDF,
    #         self.top_contact_pure_momentDF,
    #         eval_top_contact_disp(self.disk_model.disk_geometry.length,
    #                               self.disk_model.disk_geometry.top_curve_radius,
    #                               self.top_joint_angle,
    #                               self.disk_model.disk_geometry.top_orientationDF)
    #     )

    # @property
    # def bottom_contact_force_pure_moment_disp_tuple(self):
    #     return (
    #         self.bottom_contact_forceDF,
    #         self.bottom_contact_pure_momentDF,
    #         eval_bottom_contact_disp(self.disk_model.disk_geometry.length,
    #                                  self.disk_model.disk_geometry.bottom_curve_radius,
    #                                  self.bottom_joint_angle)
    #     )

    # @property
    # def sum_of_forces(self):
    #     return (sum((eval_tendon_guide_top_force(
    #         ts.tension_in_disk,
    #         self.top_joint_angle,
    #         self.disk_model.disk_geometry.top_orientationDF
    #     ) for ts in self.continuous_tendon_states), np.zeros(3)) +
    #         sum((eval_tendon_guide_bottom_force(
    #             ts.tension_in_disk,
    #             self.bottom_joint_angle,
    #         ) for ts in self.tendon_states), np.zeros(3)) +
    #         self.top_contact_forceDF +
    #         self.bottom_contact_forceDF)

    # @property
    # def sum_of_moments(self):
    #     top_contact_force, top_contact_moment, top_contact_disp = self.top_contact_force_pure_moment_disp_tuple
    #     bottom_contact_force, bottom_contact_moment, bottom_contact_disp = self.bottom_contact_force_pure_moment_disp_tuple

    #     return (sum((np.cross(r, f)
    #                  for _, f, r in self.tendon_guide_top_end_state_force_disp_tuple), np.zeros(3)) +
    #             sum((np.cross(r, f)
    #                  for _, f, r in self.tendon_guide_bottom_end_state_force_disp_tuple), np.zeros(3)) +
    #             np.cross(top_contact_disp, top_contact_force) +
    #             top_contact_moment +
    #             np.cross(bottom_contact_disp, bottom_contact_force) +
    #             bottom_contact_moment)

