import numpy as np

from .definition import *
from .math_components.force_components import *
from .math_components.displacement_components import *

class CableState():
    """
        Record the parameters of all cables running through the ring
    """

    def __init__(self, cableLocation: CableLocation, tensionInRing: float, isKnob: bool):
        self.cableLocation = cableLocation
        self.tensionInRing = tensionInRing
        self.isKnob = isKnob

    @staticmethod
    def createKnobs(knobCableLocations: List[CableLocation], tensionsInRing: List[float]):
        return [CableState(cableLocation=cl, tensionInRing=t, isKnob=True) for t, cl in zip(tensionsInRing, knobCableLocations)]

    def toProximalRingState(self, fricCoef, jointBendingAngle):
        return CableState(cableLocation=self.cableLocation,
                          tensionInRing=evalCapstan(
                              tensionEnd=self.tensionInRing, fricCoef=fricCoef, totalAngle=-jointBendingAngle),
                          isKnob=False)

class ReactionState(ForceMomentComponent):
    """
        Record the reactions
    """
    def __init__(self, force=np.zeros(3), moment=np.zeros(3)):
        super().__init__(force, moment)

    def toProximalRingState(self, jointBendingAngle, orientationDiffRF):
        return ReactionState(force=evalTopContactComp(
            bottomReactionComponentInDistalRingInDistalRF=self.force,
            jointBendingAngle=jointBendingAngle,
            topOrientationRF=orientationDiffRF),
            moment=evalTopContactComp(
            bottomReactionComponentInDistalRingInDistalRF=self.moment,
            jointBendingAngle=jointBendingAngle,
            topOrientationRF=orientationDiffRF))


class SnakeJointRingState():
    def __init__(self,
                 ring: SnakeJointRing,
                 cableStates: List[CableState],
                 bottomReactionState: ReactionState,
                 bottomJointBendingAngle: float,
                 distalRingState=None):
        self.ring = ring
        self.cableStates = cableStates
        self.bottomReactionState = bottomReactionState
        self.bottomJointBendingAngle = bottomJointBendingAngle
        self.distalRingState = distalRingState

    @property
    def topJointBendingAngle(self):
        return self.distalRingState.bottomJointBendingAngle

    def getCableStatesProximalRing(self):
        jointBendingAngle = self.bottomJointBendingAngle
        fricCoef = self.ring.fricCoef
        return [cs.toProximalRingState(fricCoef=fricCoef, jointBendingAngle=jointBendingAngle) for cs in self.cableStates]

    def getVectorComponents(self):
        from .math_components_wrapper import topCableReactionForce, \
            topCableDisplacement, bottomCableReactionForce, bottomCableDisplacement, bottomReactionDisplacement,  \
            topReactionComponent, topReactionDisplacement
        ring = self.ring
        components = []

        # cable
        for cs in self.cableStates:
            components.append(Component(
                topCableReactionForce(
                    ring, cs, None if cs.isKnob else self.topJointBendingAngle),
                topCableDisplacement(ring, cs)
            ))

            components.append(Component(
                bottomCableReactionForce(
                    ring, cs, self.bottomJointBendingAngle),
                bottomCableDisplacement(ring, cs)
            ))

        # reaction
        components.append(Component(
            self.bottomReactionState.force,
            bottomReactionDisplacement(ring, self.bottomJointBendingAngle),
            self.bottomReactionState.moment
        ))

        if self.distalRingState:
            components.append(Component(
                topReactionComponent(self.ring, self.topJointBendingAngle,
                                     self.distalRingState.bottomReactionState.force),
                topReactionDisplacement(self.ring, self.topJointBendingAngle),
                topReactionComponent(self.ring, self.topJointBendingAngle,
                                     self.distalRingState.bottomReactionState.moment)
            ))

        return components


class SnakeJointResult():
    def __init__(self, arm, state: SnakeJointRingState, error=None):
        self.arm = arm
        self.states = [state,]
        while state.distalRingState:
            state = state.distalRingState
            self.states.append(state)
            
        self.error = error

    def computeTransform(self) -> np.ndarray:
        return None

    def computeCableLengths(self) -> List[List[float]]:
        return None
