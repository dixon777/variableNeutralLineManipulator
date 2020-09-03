import os
import copy
import json
from math import pi
from abc import ABC, abstractmethod
import numpy as np
from typing import List, Dict, Iterable, Tuple

from ..common.entities import *
from ..common.calculation import normalise_angle, normalise_to_range


class SimDiskGeometry(DiskGeometryBase):
    def __init__(self,
                 length,
                 bottom_curve_radius,
                 top_orientationDF,
                 top_curve_radius,
                 outer_diameter):
        super().__init__(length, bottom_curve_radius, top_orientationDF, top_curve_radius)
        self.outer_diameter = outer_diameter
    
    @staticmethod
    def from_base_geometry(disk_geometry: DiskGeometryBase, tendon_models:List[TendonModel]):
        max_tendon_dist_from_axis = max((ts.dist_from_axis
                                        for ts in
                                        tendon_models), default=0.0)
        min_curve_radius = 0
        if disk_geometry.bottom_curve_radius is None:
            min_curve_radius = disk_geometry.top_curve_radius
        elif disk_geometry.top_curve_radius is None:
            min_curve_radius = disk_geometry.bottom_curve_radius
        else:
            min_curve_radius = min(disk_geometry.bottom_curve_radius, disk_geometry.top_curve_radius)
        
        assert(min_curve_radius > max_tendon_dist_from_axis)
        return SimDiskGeometry(
            outer_diameter=(0.7*min_curve_radius + 0.3*max_tendon_dist_from_axis)*2,
            **{
                key: disk_geometry.__getattribute__(key) for key in disk_geometry.attr_keys
            }
        )
    
    def local_attr_keys(self):
        return ["outer_diameter"]
    
    def local_eq_attr_keys(self):
        return ["outer_diameter"]

# class SimTendonData(TendonData):
#     def __init__(self,
#                  orientationMF,
#                  dist_from_axis,
#                  ):
#         super().__init__(dist_from_axis)
#         self.orientationMF = normalise_angle(orientationMF)

#     def orientationDF(self, disk_orientationMF):
#         return self.orientationMF - disk_orientationMF

#     def local_attr_keys(self):
#         return ["orientationMF"]

#     def local_eq_attr_keys(self):
#         return []

#     def __eq__(self, other):
#         return super().__eq__(other) and normalise_angle(self.orientationMF) == normalise_angle(other.orientationMF)

#     def __hash__(self):
#         return super().__hash__()





# class SimDiskModel(BaseDataClass):
#     def __init__(self, bottom_orientationMF, geometry, tendon_datas):
#         self.bottom_orientationMF = bottom_orientationMF
#         self.geometry: SimDiskGeometry = geometry
#         self.tendon_datas: List[SimTendonData] = copy.copy(tendon_datas)

#     def local_attr_keys(self):
#         return ["bottom_orientationMF", "geometry", "tendon_datas"]

#     def local_eq_attr_keys(self):
#         return ["geometry", "tendon_datas"]

#     def __eq__(self, other):
#         return super().__eq__(other) and normalise_angle(self.bottom_orientationMF) == normalise_angle(other.bottom_orientationMF)


# class SimSegment2DoFModel(BaseDataClass):
#     """
#         Math definition of segment
#     """

#     def __init__(self,
#                  is_2_DoF,
#                  n_joints,
#                  disk_length,
#                  orientationMF,
#                  curve_radius,
#                  tendon_dist_from_axis,
#                  end_disk_length=None,
#                  disk_outer_diameter=None):
#         self.n_joints = n_joints
#         self.is_2_DoF = is_2_DoF
#         self.disk_length = disk_length
#         self.orientationMF = orientationMF
#         self.curve_radius = curve_radius
#         self.tendon_dist_from_axis = tendon_dist_from_axis
#         self.end_disk_length = end_disk_length
#         self.disk_outer_diameter = disk_outer_diameter

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
#     def tendon_datas(self):
#         return sorted((SimTendonData(orientationMF=normalise_angle(self.orientationMF + relative_orientation),
#                                      dist_from_axis=self.tendon_dist_from_axis)
#                        for relative_orientation in ((pi/2, 3*pi/2) if not self.is_2_DoF else (0, pi/2, pi, 3*pi/2))),
#                       key=lambda x: x.orientationMF)

#     # @staticmethod
#     # def _to_all_tendon_guide_geometriesDF(tendon_datas, disk_orientation):
#     #     return sorted((TendonGuideGeometryDF.from_MF(geometryMF, disk_orientation) for geometryMF in tendon_datas), key=lambda x: x.orientationDF)

#     def generate_base_disk_model(self, length=None, distal_tendon_data=[]):
#         disk_orientationMF = self.orientationMF
#         all_tendon_datas = (self.tendon_datas + distal_tendon_data)
#         return SimDiskModel(
#             bottom_orientationMF=disk_orientationMF,
#             tendon_datas=all_tendon_datas,
#             geometry=SimDiskGeometry(length=length if length else self.disk_length,
#                                      outer_diameter=self.disk_outer_diameter,
#                                      bottom_curve_radius=0,
#                                      top_orientationDF=0,
#                                      top_curve_radius=self.curve_radius),)

#     def generate_disk_models(self, end_top_orientationMF=None, end_top_curve_radius=None, tendon_datas=[]):
#         return indices_entity_pairs_to_ordered_list(
#             self.generate_indices_disk_model_pairs(end_top_orientationMF=end_top_orientationMF,
#                                                    end_top_curve_radius=end_top_curve_radius,
#                                                    tendon_datas=tendon_datas)
#         )

#     def generate_indices_disk_model_pairs(self, end_top_orientationMF=None, end_top_curve_radius=None, tendon_datas=[]):
#         res = []
#         all_tendon_datas = (self.tendon_datas + tendon_datas)

#         # 1 DoF
#         if not self.is_2_DoF:
#             disk_orientationMF = self.orientationMF

#             # Whether the last disk is identical to any intermediate disk. If true, merge it with other disk
#             last_disk_mergable = self.n_joints > 1 and self.orientationMF == end_top_orientationMF and self.curve_radius == end_top_curve_radius
#             if self.n_joints > 1:
#                 res.append((tuple(i for i in range(self.n_joints - (0 if last_disk_mergable else 1))),
#                             SimDiskModel(
#                     bottom_orientationMF=disk_orientationMF,
#                     tendon_datas=all_tendon_datas,
#                     geometry=SimDiskGeometry(length=self.disk_length,
#                                              outer_diameter=self.disk_outer_diameter,
#                                              bottom_curve_radius=self.curve_radius,
#                                              top_orientationDF=0,
#                                              top_curve_radius=self.curve_radius,))))

#             if not last_disk_mergable:
#                 res.append(((self.n_joints-1, ), SimDiskModel(
#                     bottom_orientationMF=disk_orientationMF,
#                     tendon_datas=all_tendon_datas,
#                     geometry=SimDiskGeometry(length=self.disk_length,
#                                              outer_diameter=self.disk_outer_diameter,
#                                              bottom_curve_radius=self.curve_radius,
#                                              top_orientationDF=(
#                                                  end_top_orientationMF - disk_orientationMF if end_top_orientationMF else None),
#                                              top_curve_radius=end_top_curve_radius,))))

#         # 2 DoF
#         else:
#             # Whether the last disk is identical to any one of the intermediate disks. If true, merge it with other disk
#             last_disk_mergable = self.n_joints > 1 and (self.orientationMF + (
#                 pi/2 if self.n_joints % 2 == 0 else pi)) == end_top_orientationMF and self.curve_radius == end_top_curve_radius

#             if self.n_joints > 1:
#                 disk_orientationMF = self.orientationMF
#                 res.append((tuple(i for i in range(0, self.n_joints - (0 if last_disk_mergable else 1), 2)),
#                             SimDiskModel(disk_orientationMF,
#                                          tendon_datas=all_tendon_datas,
#                                          geometry=SimDiskGeometry(length=self.disk_length,
#                                                                   outer_diameter=self.disk_outer_diameter,
#                                                                   bottom_curve_radius=self.curve_radius,
#                                                                   top_orientationDF=pi/2,
#                                                                   top_curve_radius=self.curve_radius,))))

#             if self.n_joints > 2:
#                 disk_orientationMF = normalise_angle(self.orientationMF + pi/2)
#                 res.append((tuple(i for i in range(1, self.n_joints - (0 if last_disk_mergable else 1), 2)),
#                             SimDiskModel(disk_orientationMF,
#                                          tendon_datas=all_tendon_datas,
#                                          geometry=SimDiskGeometry(length=self.disk_length,
#                                                                   outer_diameter=self.disk_outer_diameter,
#                                                                   bottom_curve_radius=self.curve_radius,
#                                                                   top_orientationDF=-pi/2,
#                                                                   top_curve_radius=self.curve_radius,))))

#             if not last_disk_mergable:
#                 disk_orientationMF = normalise_angle(
#                     self.orientationMF + (pi/2 if self.n_joints % 2 == 0 else 0))
#                 res.append(
#                     ((self.n_joints-1, ),
#                      SimDiskModel(disk_orientationMF,
#                                   tendon_datas=all_tendon_datas,
#                                   geometry=SimDiskGeometry(length=self.disk_length,
#                                                            outer_diameter=self.disk_outer_diameter,
#                                                            bottom_curve_radius=self.curve_radius,
#                                                            top_orientationDF=(end_top_orientationMF -
#                                                                               disk_orientationMF if end_top_orientationMF else None),
#                                                            top_curve_radius=end_top_curve_radius,))))

#         return res

#     def local_attr_keys(self):
#         return ["is_2_DoF", "n_joints", "disk_length", "orientationMF", "curve_radius", "tendon_dist_from_axis",
#                 "end_disk_length", "disk_outer_diameter"]


# class SimManipulatorModel:
#     def __init__(self, segments):
#         self.segments: List[SimSegment2DoFModel] = segments

#     @property
#     def tendon_datas(self):
#         return list(data for s in self.segments for data in s.tendon_datas)

#     def generate_disk_models(self, base_disk_length=None):
#         return indices_entity_pairs_to_ordered_list(self.generate_indices_disk_model_pairs(base_disk_length))

#     def generate_indices_disk_model_pairs(self, base_disk_length=None):
#         segments = self.segments

#         indices_disk_model_pairs = []
#         tendon_datas = []

#         n_joints_count = (sum((segment.num_joints for segment in segments)
#                               ) + 1)

#         end_top_orientationMF = 0
#         end_top_curve_radius = None

#         for i, segment in enumerate(reversed(segments)):
#             segment_specific_indices_disk_model_pairs = segment.generate_indices_disk_model_pairs(
#                 end_top_orientationMF, end_top_curve_radius, tendon_datas)
#             n_joints_count -= segment.num_joints
#             indices_disk_model_pairs = [(tuple((i + n_joints_count) for i in indices), disk_model)
#                                         for indices, disk_model in segment_specific_indices_disk_model_pairs] + indices_disk_model_pairs

#             if i == len(segments)-1:
#                 break

#             # Update state
#             tendon_datas += segment.tendon_datas
#             end_top_orientationMF = segment.bottom_orientationMF
#             end_top_curve_radius = segment.bottom_curve_radius

#         indices_disk_model_pairs.insert(0, ((0,), segments[0].generate_base_disk_model(
#             base_disk_length, tendon_datas)))

#         return indices_disk_model_pairs


# class SimDiskState(BaseDataClass):
#     def __init__(self,
#                  disk_model: SimDiskModel,
#                  bottom_contact_forceDF: np.ndarray,
#                  bottom_contact_pure_momentDF: np.ndarray,
#                  bottom_joint_angle: float,):
#         self.disk_model: SimDiskModel = disk_model
#         self.bottom_contact_forceDF = np.array(bottom_contact_forceDF)
#         self.bottom_contact_pure_momentDF = np.array(
#             bottom_contact_pure_momentDF)
#         self.bottom_joint_angle = float(bottom_joint_angle)

#     @ property
#     def local_attr_keys(self):
#         return ["disk_model", "bottom_contact_forceDF", "bottom_contact_pure_momentDF", "bottom_joint_angle"]


# # def generate_CAD_indices(disk_geometry_model_indices: List[Tuple[Tuple[int], SimDiskModel]], curve_only=True):
# #     from ..cad.cadquery_disk_generator import generate_disk_CAD

# #     res = []
# #     indices_disk_model_pairs_copy = copy.copy(disk_geometry_model_indices)
# #     while len(indices_disk_model_pairs_copy) > 0:
# #         _, first_unique_disk_geometry_model = indices_disk_model_pairs_copy[0]
# #         first_unique_disk_geometry = first_unique_disk_geometry_model.geometry
# #         cad_model = generate_disk_CAD(first_unique_disk_geometry, curve_only)

# #         matched_indices = [i for i, (_, disk_geometry_model) in enumerate(indices_disk_model_pairs_copy) if ((curve_only and DiskGeometryBase.__eq__(first_unique_disk_geometry, disk_geometry_model.geometry)) or
# #                                                                                                           not curve_only and SimDiskGeometry.__eq__(first_unique_disk_geometry, disk_geometry_model.geometry))]

# #         res.append((*(index for index in (indices for indices,
# #                                           _ in (indices_disk_model_pairs_copy[i] for i in matched_indices))), first_unique_disk_geometry, cad_model))

# #         # Remove from the list
# #         for i in reversed(matched_indices):
# #             del indices_disk_model_pairs_copy[i]

# #     return res


# # def export_CAD_indices_for_simulation(model_indices, geometry_CAD_indices, dir_path):
# #     from ..cad.cadquery_disk_generator import export_CAD
# #     import os
# #     import json
# #     os.makedirs(dir_path, exist_ok=True)

# #     data_json = {"geometries": {}, "bottom_orientationMF": {}}

# #     for i, (indices, geometry, cad_model) in enumerate(geometry_CAD_indices):
# #         export_CAD(cad_model, os.path.join(
# #             dir_path, str(i)), "step", tolerance=0.0001)

# #         # data_json["geometries"][i] = dict(geometry)

# #         # for m_indices, _ in model_indices:
# #         #     if m_indices[0] in indices:
# #         #         for model_index in m_indices:
# #         #             data_json["bottom_orientationMF"][model_index] = i

# #     # with open(os.path.join(dir_path, ".cad_details.json"), "w") as f:
# #     #     f.write(json.dumps(data_json))


# if __name__ == "__main__":
#     pass
