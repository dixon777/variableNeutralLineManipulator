import numpy as np
from .entities import BaseDataClass


class GlobalExternalLoad(BaseDataClass):
    def __init__(self,
                 disk_index: int,
                 force: np.ndarray = np.zeros(3),
                 torque: np.ndarray = np.zeros(3),
                 location: np.ndarray = np.zeros(3),
                 is_attached_to_disk:bool=True,
                 is_force_vector_global: bool = True,
                 is_location_global: bool = True,
                 ):
        """
        Parameters
        ----------
        disk_index : int
            Index of disk on which the force and torque are acted (0 = base disk)
        force : np.ndarry[float,float,float]
            Force vector (Default: np.zeros(3))
        torque : np.ndarry[float,float,float]
            Torque vector (Default: np.zeros(3))
        location : np.ndarry[float,float,float]
            Initial location at which the force and torque are acted (Default: np.zeros(3))
        is_attached_to_disk : bool (Default: True)
            Whether the force vector's location is fixed to the disk or the ground
        is_force_vector_global : bool (Default: True)
            Whether the "force" and "torque" are defined according to the global or the disk coordinate systems
        is_location_global : bool (Default: True)
            Whether the "location" is defined according to the global or the disk coordinate systems

        """
        self.disk_index = disk_index
        self.force = force
        self.torque = torque
        self.location = location
        self.is_attached_to_disk = is_attached_to_disk
        self.is_location_global = is_location_global
        self.is_force_vector_global = is_force_vector_global

    def local_attr_keys(self):
        return ["disk_index", "force", "torque", "location", "is_attached_to_disk", "is_location_global", "is_force_vector_global"]
