import os
import copy
import json
from math import pi
from abc import ABC
from typing import List, Dict, Iterable, Tuple

from .calculation import normalise_angle, normalise_to_range

# Courtesy to dan_waterworth
# https://stackoverflow.com/questions/4544630/automatically-growing-lists-in-python


class GrowingList(list):
    def __setitem__(self, index, value):
        if index >= len(self):
            self.extend([None]*(index + 1 - len(self)))
        list.__setitem__(self, index, value)


def indices_entity_pairs_to_ordered_list(indices_entity_pairs):
    growing_list = GrowingList()
    for indices, entity in indices_entity_pairs:
        for i in indices:
            growing_list[i] = entity
    return growing_list


class BaseDataClass(ABC):
    @classmethod
    def all_subclasses(cls):
        return set(cls.__subclasses__()).union([s for c in cls.__subclasses__() for s in c.all_subclasses()])  
    
    @property
    def attr_keys(self):
        keys = []
        for base in [*self.__class__.__bases__, self.__class__]:
            keys += base.local_attr_keys(self) or []
        return keys

    @property
    def eq_attr_keys(self):
        keys = []
        for base in [*self.__class__.__bases__, self.__class__]:
            keys += base.local_eq_attr_keys(
                self) or base.local_attr_keys(self) or []
        return keys

    def local_attr_keys(self):
        return []

    def local_eq_attr_keys(self):
        return None

    def __eq__(self, other):
        return isinstance(other, self.__class__) and all(self.__dict__[key] == other.__dict__[key] for key in self.eq_attr_keys)
    
    def __hash__(self):
        return hash((self.__dict__[k] for k in self.eq_attr_keys))

    def __iter__(self):
        yield "__class_name__", self.__class__.__name__
        for k in self.attr_keys:
            j = self.__dict__[k]
            yield k, j

    def __repr__(self):
        return str(dict(self))

    class Encoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, BaseDataClass):
                return dict(obj)
            return super().default(obj)

    class Decoder(json.JSONDecoder):
        def __init__(self, *args, **kwargs):
            super().__init__(object_hook=self.object_hook, *args, **kwargs)

        @staticmethod
        def object_hook(obj):
            if '__class_name__' not in obj:
                return obj
            cls_str = obj['__class_name__']
            del obj['__class_name__']
            cl = next((c for c in BaseDataClass.all_subclasses() if c.__name__ == cls_str), None)
            if cl is None:
                print("Error in json ")
                return None
            return cl(**obj)
        
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
    def top_orientationDF_normalised_copy(self):
        import copy
        copied = copy.deepcopy(self)
        copied.top_orientationDF = normalise_angle(self.top_orientationDF)
        return copied

    def local_attr_keys(self):
        return ["length", "bottom_curve_radius",
                "top_orientationDF", "top_curve_radius"]

    def local_eq_attr_keys(self):
        return ["length", "bottom_curve_radius", "top_curve_radius"]

    def __eq__(self, other):
        return super().__eq__(other) and normalise_to_range(self.top_orientationDF, pi) == normalise_to_range(other.top_orientationDF, pi)


class TendonData(BaseDataClass):
    """
        Base geometric definition of the tendon model
        @param:
            orientationMF: Orientation of the tendon around the centre axis of the manipulator in neutral state
            dist_from_axis: Distance from the centre axis of the disk
    """

    def __init__(self, dist_from_axis):
        self.dist_from_axis = dist_from_axis

    def local_attr_keys(self):
        return ["dist_from_axis"]
