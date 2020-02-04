import numpy as np

from .entities import *
from .vector_components import *
from .math_components.force_components import *
from .math_components.displacement_components import *
from .math_components.vector_computation import *


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

    def toProximalRingState(self, fricCoef, jointAngle):
        return CableState(cableLocation=self.cableLocation,
                          tensionInRing=evalCapstan(
                              tensionEnd=self.tensionInRing, fricCoef=fricCoef, totalAngle=jointAngle),
                          isKnob=False)


class ContactReactionComponent(ForceMomentComponent):
    """
        Record the reactions
    """

    def __init__(self, force=np.zeros(3), moment=np.zeros(3)):
        super().__init__(force, moment)

    def toProximalRingState(self, jointAngle, orientationDiffRF):
        return ContactReactionComponent(force=evalTopContactComp(
            DRBottomContactCompDRF=self.force,
            topJointAngle=jointAngle,
            topOrientationRF=orientationDiffRF),
            moment=evalTopContactComp(
            DRBottomContactCompDRF=self.moment,
            topJointAngle=jointAngle,
            topOrientationRF=orientationDiffRF))


class RingState():
    def __init__(self,
                 ring: Ring,
                 cableStates: List[CableState],
                 bottomContactReactionComponent: ContactReactionComponent,
                 bottomJointAngle: float,
                 distalRingState=None):
        self.ring = ring
        self.cableStates = cableStates
        self.bottomContactReactionComponent = bottomContactReactionComponent
        self.bottomJointAngle = bottomJointAngle
        self.distalRingState = distalRingState

    @property
    def topJointAngle(self):
        return self.distalRingState.bottomJointAngle

    def getCableStatesProximalRing(self):
        jointAngle = self.bottomJointAngle
        fricCoefRingCable = self.ring.fricCoefRingCable
        return [cs.toProximalRingState(fricCoef=fricCoefRingCable, jointAngle=jointAngle) for cs in self.cableStates]

    def getVectorComponents(self):
        from .math_wrapper import topCableReactionForce, \
            topCableDisplacement, bottomCableReactionForce, bottomCableDisplacement, bottomReactionDisplacement,  \
            topReactionComponent, topReactionDisplacement
        ring = self.ring
        components = []

        # cable
        for cs in self.cableStates:
            components.append(Component(
                topCableReactionForce(
                    ring, cs, None if cs.isKnob else self.topJointAngle),
                topCableDisplacement(ring, cs)
            ))

            components.append(Component(
                bottomCableReactionForce(
                    ring, cs, self.bottomJointAngle),
                bottomCableDisplacement(ring, cs)
            ))

        # reaction
        components.append(Component(
            self.bottomContactReactionComponent.force,
            bottomReactionDisplacement(ring, self.bottomJointAngle),
            self.bottomContactReactionComponent.moment
        ))

        if self.distalRingState:
            components.append(Component(
                topReactionComponent(self.ring, self.topJointAngle,
                                     self.distalRingState.bottomContactReactionComponent.force),
                topReactionDisplacement(self.ring, self.topJointAngle),
                topReactionComponent(self.ring, self.topJointAngle,
                                     self.distalRingState.bottomContactReactionComponent.moment)
            ))

        return components

class StateResult():
    def __init__(self, mostProximalRingState: RingState, error=None):
        state = mostProximalRingState
        self.states = []
        while state:
            self.states.append(state)
            state = state.distalRingState

        self.error = error
        
    def getTF(self, ringIndex=-1, side="b"):
        """
            side = "b": bottom, "c": center, "tr": top in ring frame, "td": top in top curve orientation
        """
        side = "b" if side is None else side
        c = np.identity(4)
        for s in self.states[:ringIndex]:
            c = np.matmul(c, getTFProximalTopToDistalBottom(s.bottomJointAngle, s.ring.bottomCurveRadius))
            c = np.matmul(np.matmul(c, composeTF(t=(0,0, s.ring.length))),  composeTF(RM=getRMFromAxis((0,0,1), s.ring.topOrientationRF)))
        
        s = self.states[ringIndex]
        c = np.matmul(c, getTFProximalTopToDistalBottom(s.bottomJointAngle, s.ring.bottomCurveRadius))
        if side == "b":
            return c
        elif side == "c":
            return np.matmul(c, composeTF(t=(0,0, s.ring.length/2)))

        c = np.matmul(c, composeTF(t=(0,0, s.ring.length)))
        if side == "tr":
            return c
        elif side == "td":
            return np.matmul(c, composeTF(RM=getRMFromAxis((0,0,1), s.ring.topOrientationRF)))
        else:
            raise NotImplementedError()    
     

    def computeCableLengths(self) -> List[List[float]]:
        return None
