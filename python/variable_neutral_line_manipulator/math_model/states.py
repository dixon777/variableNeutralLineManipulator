import numpy as np
import pyrr.matrix44 as m4

from .entities import *
from .vector_components import *
from .helper_functions import *
from .force_comp import changeContactCompFrame


class TendonState():
    """
        Record the parameters of all tendons running through the ring
    """

    def __init__(self, tendonLocation: TendonLocation, tensionInRing: float, isKnob: bool):
        self.tendonLocation = tendonLocation
        self.tensionInRing = tensionInRing
        self.isKnob = isKnob

    @staticmethod
    def createKnobs(knobTendonLocations: List[TendonLocation], tensionsInRing: List[float]):
        return [TendonState(tendonLocation=cl, tensionInRing=t, isKnob=True) for t, cl in zip(tensionsInRing, knobTendonLocations)]

    def toProximalRingState(self, fricCoef, jointAngle):
        return TendonState(tendonLocation=self.tendonLocation,
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
        return ContactReactionComponent(force=changeContactCompFrame(
            DRBottomContactCompDRF=self.force,
            topJointAngle=jointAngle,
            topOrientationRF=orientationDiffRF),
            moment=changeContactCompFrame(
            DRBottomContactCompDRF=self.moment,
            topJointAngle=jointAngle,
            topOrientationRF=orientationDiffRF))


class RingState():
    def __init__(self,
                 ring: Ring,
                 tendonStates: List[TendonState],
                 bottomContactReactionComponent: ContactReactionComponent,
                 bottomJointAngle: float,
                 distalRingState=None):
        self.ring = ring
        self.tendonStates = tendonStates
        self.bottomContactReactionComponent = bottomContactReactionComponent
        self.bottomJointAngle = bottomJointAngle
        self.distalRingState = distalRingState

    @property
    def topJointAngle(self):
        return self.distalRingState.bottomJointAngle

    def getTendonStatesProximalRing(self):
        jointAngle = self.bottomJointAngle
        fricCoefRingTendon = self.ring.fricCoefRingTendon
        return [cs.toProximalRingState(fricCoef=fricCoefRingTendon, jointAngle=jointAngle) for cs in self.tendonStates]

    def getVectorComponents(self):
        from .math_wrapper import topTendonReactionForce, \
            topTendonDisplacement, bottomTendonReactionForce, bottomTendonDisplacement, bottomReactionDisplacement,  \
            topReactionComponent, topReactionDisplacement
        ring = self.ring
        components = []

        # tendon
        for cs in self.tendonStates:
            components.append(Component(
                topTendonReactionForce(
                    ring, cs, None if cs.isKnob else self.topJointAngle),
                topTendonDisplacement(ring, cs)
            ))

            components.append(Component(
                bottomTendonReactionForce(
                    ring, cs, self.bottomJointAngle),
                bottomTendonDisplacement(ring, cs)
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
    
def getTFProximalTopToDistalBottom(jointAngle: float, curveRadius: float):
    rot = m4.create_from_axis_rotation((1,0,0), jointAngle/2)
    return np.matmul(rot, 
                     m4.create_from_translation((0,0,2*curveRadius*(1-math.cos(jointAngle/2)))) +
                     rot) # Trick: m_rot * m_trans * m_rot = m_rot * (m_trans + m_rot)

class StateResult():
    def __init__(self, mostProximalRingState: RingState, error=None):
        state = mostProximalRingState
        self.states = []
        while state:
            self.states.append(state)
            state = state.distalRingState

        self.error = error # Not used
        
    def getTF(self, ringIndex=-1, side="c"):
        """
            side = "b": bottom, "c": center, "tr": top in ring frame, "td": top in top curve orientation
        """
        side = "c" if side is None else side
        c = np.identity(4)
        for s in self.states[:ringIndex]:
            c = np.matmul(c, getTFProximalTopToDistalBottom(s.bottomJointAngle, s.ring.bottomCurveRadius))
            # Trick: m_rot * m_trans * m_rot = m_rot * (m_trans + m_rot)
            c = np.matmul(c, m4.create_from_translation((0,0, s.ring.length)) + 
                          m4.create_from_axis_rotation((0,0,1), s.ring.topOrientationRF))

        
        s = self.states[ringIndex]
        c = np.matmul(c, getTFProximalTopToDistalBottom(s.bottomJointAngle, s.ring.bottomCurveRadius))
        if side == "b":
            return c
        elif side == "c":
            return np.matmul(c, m4.create_from_translation((0,0, s.ring.length/2)))

        c = np.matmul(c, m4.create_from_translation((0,0, s.ring.length)))
        if side == "tr":
            return c
        elif side == "td":
            return np.matmul(c, m4.create_from_axis_rotation((0,0,1), s.ring.topOrientationRF))
        
        raise NotImplementedError()    
     

    def computeTendonLengths(self) -> List[List[float]]:
        raise NotImplementedError()
