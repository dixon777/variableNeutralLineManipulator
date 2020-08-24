import os
import copy
import json
from abc import ABC, abstractmethod
import numpy as np
from typing import List, Dict, Iterable, Tuple


def jsonify(item):
    if isinstance(item, (int, float, str, bool)):
        return item
    
    if isinstance(item, (list,np.ndarray)):
        return [jsonify(i) for i in item]
    
    if isinstance(item, dict):
        return {(str(i), jsonify(j)) for i, j in item.items()}
    
    if hasattr(item, '__iter__'):
        try:
            first_subitem = next(item.__iter__())
            if len(first_subitem) == 2:
                d = {}
                for i, j in dict(item).items():
                    d[str(i)] = jsonify(j)
                return d
            else:
                return [jsonify(i) for i in item]
        except StopIteration:
            return {}
    return item


class BaseDataClass(ABC):
    @property
    def attr_keys(self):
        return []

    def __eq__(self, other):
        return isinstance(other, self.__class__) and all(self.__dict__[key] == other.__dict__[key] for key in self.attr_keys)

    def __iter__(self):
        yield "__class_name__", self.__class__.__name__
        for k in self.attr_keys:
            yield k, jsonify(self.__dict__[k])

    def __repr__(self):
        return str(dict(self))


class DiskGeometryBase(BaseDataClass):
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
        self.top_orientationDF = top_orientationDF
        self.top_curve_radius = top_curve_radius

    @property
    def attr_keys(self):
        return super().attr_keys + ["length", "bottom_curve_radius",
                                    "top_orientationDF", "top_curve_radius"]


class TendonGeometryBase(BaseDataClass):
    """
        Base geometric definition of the tendon model
        @param:
            orientationMF: Orientation of the tendon around the centre axis of the manipulator in neutral state
            dist_from_axis: Distance from the centre axis of the disk
    """

    def __init__(self, dist_from_axis):
        self.dist_from_axis = dist_from_axis

    @property
    def attr_keys(self):
        return super().attr_keys + ["dist_from_axis"]


class TendonGuideGeometryMF(TendonGeometryBase):
    def __init__(self,
                 orientationMF,
                 dist_from_axis,
                 diameter
                 ):
        super().__init__(dist_from_axis)
        self.orientationMF = orientationMF
        self.diameter = diameter

    def orientationDF(self, disk_orientationMF):
        return self.orientationMF - disk_orientationMF

    @property
    def attr_keys(self):
        return super().attr_keys + ["orientationMF", "diameter"]


class TendonGuideGeometryDF(TendonGeometryBase):
    def __init__(self,
                 orientationDF,
                 dist_from_axis,
                 diameter
                 ):
        super().__init__(dist_from_axis)
        self.orientationDF = orientationDF
        self.diameter = diameter

    @staticmethod
    def from_MF(target, disk_orientationMF):
        if isinstance(target, TendonGuideGeometryDF):
            return target
        elif isinstance(target, TendonGuideGeometryMF):
            return TendonGuideGeometryDF(orientationDF=target.orientationDF(disk_orientationMF), dist_from_axis=target.dist_from_axis, diameter=target.diameter)
        raise ValueError()

    def orientationMF(self, disk_orientationMF):
        return self.orientationDF + disk_orientationMF

    @property
    def attr_keys(self):
        return super().attr_keys + ["orientationDF", "diameter"]


class DiskGeometry(DiskGeometryBase):
    def __init__(self,
                 outer_diameter,
                 centre_hole_diameter=None,
                 tendon_guide_geometriesDF=[],
                 *args, **kwargs
                 ):
        super().__init__(*args, **kwargs)
        self.outer_diameter = outer_diameter
        self.centre_hole_diameter = centre_hole_diameter
        self.tendon_guide_geometriesDF = tendon_guide_geometriesDF

    @property
    def top_orientationMF(self, disk_orientationMF):
        return self.top_orientationDF + disk_orientationMF

    @property
    def attr_keys(self):
        return super().attr_keys + ["outer_diameter", "centre_hole_diameter", "tendon_guide_geometriesDF"]


class DiskGeometryModel(BaseDataClass):
    def __init__(self, bottom_orientationMF, geometry):
        self.bottom_orientationMF = bottom_orientationMF
        self.geometry = geometry

    @property
    def attr_keys(self):
        return super().attr_keys + ["bottom_orientationMF", "geometry"]


class Segment2DoFGeometryModel():
    """
        Math definition of segment
    """

    def __init__(self,
                 is_2_DoF,
                 n_joints,
                 disk_length,
                 orientationMF,
                 curve_radius,
                 tendon_dist_from_axis,
                 end_disk_length=None,
                 disk_outer_diameter=None,
                 centre_hole_diameter=None,
                 tendon_guide_diameter=None):
        self.n_joints = n_joints
        self.is_2_DoF = is_2_DoF
        self.disk_length = disk_length
        self.orientationMF = orientationMF
        self.curve_radius = curve_radius
        self.tendon_dist_from_axis = tendon_dist_from_axis
        self.end_disk_length = end_disk_length
        self.disk_outer_diameter = disk_outer_diameter
        self.centre_hole_diameter = centre_hole_diameter
        self.tendon_guide_diameter = tendon_guide_diameter

    @property
    def num_joints(self):
        return self.n_joints

    @property
    def bottom_orientationMF(self):
        return self.orientationMF

    @property
    def bottom_curve_radius(self):
        return self.curve_radius

    @property
    def tendon_guide_geometriesMF(self):
        return [TendonGuideGeometryMF(orientationMF=self.orientationMF + relative_orientation, dist_from_axis=self.tendon_dist_from_axis, diameter=self.tendon_guide_diameter)
                for relative_orientation in ((pi/2, 3*pi/2) if not self.is_2_DoF else (0, pi/2, pi, 3*pi/2))]

    @staticmethod
    def __to_all_tendon_guide_geometriesDF(tendon_guide_geometriesMF, disk_orientation):
        return list(TendonGuideGeometryDF.from_MF(geometryMF, disk_orientation) for geometryMF in tendon_guide_geometriesMF)

    def generate_base_disk(self, length, distal_tendon_geometriesMF=[]):
        disk_orientationMF = self.orientationMF
        all_tendon_guide_geometriesMF = self.tendon_guide_geometriesMF + \
            distal_tendon_geometriesMF
        return DiskGeometryModel(
            bottom_orientationMF=disk_orientationMF, geometry=DiskGeometry(length=length,
                                                                           outer_diameter=self.disk_outer_diameter,
                                                                           bottom_curve_radius=0,
                                                                           top_orientationDF=0,
                                                                           top_curve_radius=self.curve_radius,
                                                                           centre_hole_diameter=self.centre_hole_diameter,
                                                                           tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF)))

    def generate_disk_geometry_indices(self, end_top_orientationMF=0.0, end_top_curve_radius=None, distal_tendon_geometriesMF=[]):
        res = []
        all_tendon_guide_geometriesMF = self.tendon_guide_geometriesMF + \
            distal_tendon_geometriesMF

        # 1 DoF
        if not self.is_2_DoF:
            disk_orientationMF = self.orientationMF

            # Whether the last disk is identical to any intermediate disk. If true, merge it with other disk
            last_disk_mergable = self.n_joints > 1 and self.orientationMF == end_top_orientationMF and self.curve_radius == end_top_curve_radius
            if self.n_joints > 1:
                res.append((tuple(i for i in range(self.n_joints - (0 if last_disk_mergable else 1))),
                            DiskGeometryModel(
                    bottom_orientationMF=disk_orientationMF, geometry=DiskGeometry(length=self.disk_length,
                                                                                   outer_diameter=self.disk_outer_diameter,
                                                                                   bottom_curve_radius=self.curve_radius,
                                                                                   top_orientationDF=0,
                                                                                   top_curve_radius=self.curve_radius,
                                                                                   centre_hole_diameter=self.centre_hole_diameter,
                                                                                   tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF)))))

            if not last_disk_mergable:
                res.append(((self.n_joints-1, ), DiskGeometryModel(
                    bottom_orientationMF=disk_orientationMF, geometry=DiskGeometry(length=self.disk_length,
                                                                                   outer_diameter=self.disk_outer_diameter,
                                                                                   bottom_curve_radius=self.curve_radius,
                                                                                   top_orientationDF=(
                                                                                       end_top_orientationMF if end_top_orientationMF else 0) - disk_orientationMF,
                                                                                   top_curve_radius=end_top_curve_radius,
                                                                                   centre_hole_diameter=self.centre_hole_diameter,
                                                                                   tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF)))))

        # 2 DoF
        else:
            # Whether the last disk is identical to any one of the intermediate disks. If true, merge it with other disk
            last_disk_mergable = self.n_joints > 1 and (self.orientationMF + (
                pi/2 if self.n_joints % 2 == 0 else pi)) == end_top_orientationMF and self.curve_radius == end_top_curve_radius

            if self.n_joints > 1:
                disk_orientationMF = self.orientationMF
                intermediate_disk_geometry1 = DiskGeometry(length=self.disk_length,
                                                           outer_diameter=self.disk_outer_diameter,
                                                           bottom_curve_radius=self.curve_radius,
                                                           top_orientationDF=pi/2,
                                                           top_curve_radius=self.curve_radius,
                                                           centre_hole_diameter=self.centre_hole_diameter,
                                                           tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF))
            res.append((tuple(i for i in range(0, self.n_joints - (0 if last_disk_mergable else 1), 2)),
                        DiskGeometryModel(disk_orientationMF, intermediate_disk_geometry1)))

            if self.n_joints > 2:
                disk_orientationMF = self.orientationMF + pi/2
                intermediate_disk_geometry2 = DiskGeometry(length=self.disk_length,
                                                           outer_diameter=self.disk_outer_diameter,
                                                           bottom_curve_radius=self.curve_radius,
                                                           top_orientationDF=pi/2,
                                                           top_curve_radius=self.curve_radius,
                                                           centre_hole_diameter=self.centre_hole_diameter,
                                                           tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF))
                res.append((tuple(i for i in range(1, self.n_joints - (0 if last_disk_mergable else 1), 2)),
                            DiskGeometryModel(disk_orientationMF, intermediate_disk_geometry2)))

            if not last_disk_mergable:
                disk_orientationMF = (
                    self.orientationMF + (pi/2 if self.n_joints % 2 == 0 else 0))
                end_disk_geometry = DiskGeometry(length=self.disk_length,
                                                 outer_diameter=self.disk_outer_diameter,
                                                 bottom_curve_radius=self.curve_radius,
                                                 top_orientationDF=end_top_orientationMF - disk_orientationMF,
                                                 top_curve_radius=self.curve_radius,
                                                 centre_hole_diameter=self.centre_hole_diameter,
                                                 tendon_guide_geometriesDF=self.__to_all_tendon_guide_geometriesDF(all_tendon_guide_geometriesMF, disk_orientationMF))

                res.append(
                    ((self.n_joints-1, ), DiskGeometryModel(disk_orientationMF, end_disk_geometry)))

        return res

    @property
    def attr_keys(self):
        return super().attr_keys + ["is_2_DoF", "n_joints", "disk_length", "orientationMF", "curve_radius", "tendon_dist_from_axis",
                                    "end_disk_length", "disk_outer_diameter", "centre_hole_diameter", "tendon_guide_diameter"]


def generate_disk_geometry_indices(segments, base_disk_length=None):
    disk_geometry_indices = []
    tendon_guide_geometriesMF = []

    n_joints_count = sum((segment.num_joints for segment in segments)
                         ) + (1 if base_disk_length and base_disk_length > 0.0 else 0)

    end_top_orientationMF = 0
    end_top_curve_radius = None

    for i, segment in enumerate(reversed(segments)):
        disk_geomerty_indices_start_from_segment = segment.generate_disk_geometry_indices(
            end_top_orientationMF, end_top_curve_radius, tendon_guide_geometriesMF)
        n_joints_count -= segment.num_joints
        disk_geometry_indices = [(tuple((i + n_joints_count) for i in indices), disk_geometry)
                                 for indices, disk_geometry in disk_geomerty_indices_start_from_segment] + disk_geometry_indices

        if i == len(segments)-1:
            break

        # Update state
        tendon_guide_geometriesMF += segment.tendon_guide_geometriesMF
        end_top_orientationMF = segment.bottom_orientationMF
        end_top_curve_radius = segment.bottom_curve_radius

    if base_disk_length is not None:
        disk_geometry_indices.insert(0, ((0,), segments[0].generate_base_disk(
            base_disk_length, tendon_guide_geometriesMF)))

    return disk_geometry_indices


def generate_CAD_indices(disk_geometry_model_indices: List[Tuple[Tuple[int], DiskGeometryModel]], curve_only=True):
    from ..cad.cadquery_disk_generator import generate_disk_CAD

    res = []
    disk_geometry_indices_copy = copy.copy(disk_geometry_model_indices)
    while len(disk_geometry_indices_copy) > 0:
        _, first_unique_disk_geometry_model = disk_geometry_indices_copy[0]
        first_unique_disk_geometry = first_unique_disk_geometry_model.geometry
        cad_model = generate_disk_CAD(first_unique_disk_geometry, curve_only)

        matched_indices = [i for i, (_, disk_geometry_model) in enumerate(disk_geometry_indices_copy) if ((curve_only and DiskGeometryBase.__eq__(first_unique_disk_geometry, disk_geometry_model.geometry)) or
                                                                                                          not curve_only and DiskGeometry.__eq__(first_unique_disk_geometry, disk_geometry_model.geometry))]

        res.append((*(index for index in (indices for indices,
                                          _ in (disk_geometry_indices_copy[i] for i in matched_indices))), first_unique_disk_geometry, cad_model))

        # Remove from the list
        for i in reversed(matched_indices):
            del disk_geometry_indices_copy[i]

    return res


def export_CAD_indices_for_simulation(model_indices, geometry_CAD_indices, dir_path):
    from ..cad.cadquery_disk_generator import export_CAD
    import os
    import json
    os.makedirs(dir_path, exist_ok=True)

    data_json = {"geometries": {}, "bottom_orientationMF": {}}

    for i, (indices, geometry, cad_model) in enumerate(geometry_CAD_indices):
        export_CAD(cad_model, os.path.join(
            dir_path, str(i)), "step", tolerance=0.0001)
        data_json["geometries"][i] = dict(geometry)

        for m_indices, _ in model_indices:
            if m_indices[0] in indices:
                for model_index in m_indices:
                    data_json["bottom_orientationMF"][model_index] = i

    with open(os.path.join(dir_path, ".cad_details.json"), "w") as f:
        f.write(json.dumps(data_json))


if __name__ == "__main__":
    pass
