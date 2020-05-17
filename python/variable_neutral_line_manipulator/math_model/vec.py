import numpy as np

class ForceMomentVec():
    """
        Contains force, displacement and moment vector
        displacemnt X force + pure_moment = total_moment
        For computation of the constraint of the force model:
            sum of [force total_moment] (6 entry vector) = 0 
    """
    def __init__(self, force=np.zeros(3), disp=np.zeros(3), pure_moment=np.zeros(3)):
        self.force = np.array(force)
        self.disp = np.array(disp)
        self.pure_moment = np.array(pure_moment)
        
    @staticmethod
    def from_force_disp_pure_moment(force, disp, pure_moment=np.zeros(3)):
        return ForceMomentVec(force, disp, pure_moment)
        
    @staticmethod
    def from_force_disp_total_moment(force, disp, total_moment):
        return ForceMomentVec(force, disp, total_moment - np.cross(disp, force))
    
    @property
    def moment_by_force(self):
        return np.cross(self.disp, self.force)
        
    @property
    def total_moment(self):
        return self.moment_by_force + self.pure_moment

    @property
    def flat_pure(self):
        return np.concatenate((self.force, self.pure_moment))
    
    @property
    def flat_total(self):
        return np.concatenate((self.force, self.total_moment))

    @property
    def flat_force_only(self):
        return np.concatenate((self.force, np.zeros(3)))

    @property
    def flat_total_moment_only(self):
        return np.concatenate((np.zeros(3), self.total_moment))
    
    def add(self, other):
        if other.__class__ == ForceMomentVec:
            return np.concatenate((self.force + other.force, self.total_moment + other.total_moment))
        elif isinstance(other, np.array):
            return np.concatenate((self.force + other[:3], self.total_moment + other[3:])) 
        raise NotImplementedError()

    def __radd__(self, other):
        return self.add(other)

    def __add__(self, other):
        return self.__radd__(other)

    def __repr__(self):
        return f"force: {self.force}, disp: {self.disp}, pure moment: {self.pure_moment}"


if __name__ == "__main__":
    v = ForceMomentVec((1,1,1), (2,2,3))
    v2 = ForceMomentVec((3,2,3), (2,2,2))
    print(v+v2)