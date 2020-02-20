import numpy as np

class ForceMomentComponent():
    def __init__(self, force=None, moment=None):
        self.force = np.zeros(3) if force is None else np.array(force)
        self.moment = np.zeros(3) if moment is None else np.array(moment)

    @property
    def flat(self):
        return np.concatenate((self.force, self.moment))

    @property
    def flatForceOnly(self):
        return np.concatenate((self.force, np.zeros(3)))

    @property
    def flatMomentOnly(self):
        return np.concatenate((np.zeros(3), self.moment))

    def __radd__(self, other):
        if other.__class__ == VectorComponent:
            return ForceMomentComponent(force=self.force + other.force, moment=self.moment + other.sumMoment)
        elif isinstance(other, ForceMomentComponent):
            return ForceMomentComponent(force=self.force + other.force, moment=self.moment + other.moment)
        elif isinstance(other, (int, float)):
            return ForceMomentComponent(force=self.force, moment=self.moment)
        raise NotImplementedError()

    def __add__(self, other):
        return self.__radd__(other)

    def __repr__(self):
        return f"force: {self.force}, moment: {self.moment} [{self.__class__.__name__}]"

class VectorComponent(ForceMomentComponent):
    def __init__(self, force: np.ndarray = None, disp: np.ndarray = None, moment: np.ndarray = None):
        super().__init__(force, moment)
        self.disp = np.zeros(3) if disp is None else np.array(disp)

    @property
    def momentByForce(self):
        return np.cross(self.disp, self.force)

    @property
    def sumMoment(self):
        return self.momentByForce + self.moment

    @property
    def flat(self):
        return np.concatenate((self.force, self.sumMoment))

    def __add__(self, other):
        return self.__radd__(other)

    def __radd__(self, other):
        if other.__class__ == VectorComponent:
            return ForceMomentComponent(force=self.force + other.force, moment=self.sumMoment + other.sumMoment)
        elif other.__class__ == ForceMomentComponent:
            return ForceMomentComponent(force=self.force + other.force, moment=self.sumMoment + other.moment)
        elif isinstance(other, (int, float)):
            return ForceMomentComponent(force=self.force, moment=self.sumMoment)
        raise NotImplementedError()
    
    def __repr__(self):
        return f"force: {self.force}, disp: {self.disp}, moment: {self.moment} [{self.__class__.__name__}]"
