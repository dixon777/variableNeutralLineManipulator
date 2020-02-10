from .entities import *
from .states import *


def topCableReactionForce(ring: Ring, cableState: CableState, topJointBendingAngle: float):
    return (np.zeros(3)
            if cableState.isKnob else
            evalTopCableGuideCompWithCable(tensionInRing=cableState.tensionInRing,
                                  fricCoef=ring.fricCoefRingCable,
                                  topJointAngle=topJointBendingAngle,
                                  topOrientationRF=ring.topOrientationRF))


def bottomCableReactionForce(ring: Ring, cableState: CableState, bottomJointBendingAngle: float):
    return evalBottomCableGuideCompWithCable(tensionInRing=cableState.tensionInRing,
                                    fricCoef=ring.fricCoefRingCable,
                                    bottomJointAngle=bottomJointBendingAngle)


def topCableDisplacement(ring: Ring, cableState: CableState):
    return (evalKnobDisp(ringLength=ring.length,
                        knobLength=cableState.cableLocation.knobLength,
                        horizontalDispFromCableToAxis=cableState.cableLocation.horizontalDistFromAxis,
                        cableOrientationRF=cableState.cableLocation.orientationBF - ring.orientationBF) 
        if cableState.isKnob else 
        evalTopGuideEndDisp(ringLength=ring.length,
                            topCurveRadius=ring.topCurveRadius,
                            horizontalDispFromCableToAxis=cableState.cableLocation.horizontalDistFromAxis,
                            cableOrientationRF=cableState.cableLocation.orientationBF -
                            ring.orientationBF,
                            topOrientationRF=ring.topOrientationRF))


def bottomCableDisplacement(ring: Ring, cableState: CableState):
    return evalBottomGuideEndDisp(ringLength=ring.length,
                                  bottomCurveRadius=ring.bottomCurveRadius,
                                  horizontalDispFromCableToAxis=cableState.cableLocation.horizontalDistFromAxis,
                                  cableOrientationRF=cableState.cableLocation.orientationBF - ring.orientationBF)


def topReactionComponent(ring: Ring, topJointAngle: float, distalRingBottomReaction: np.ndarray):
    return evalTopContactComp(DRBottomContactCompDRF=distalRingBottomReaction,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def topReactionDisplacement(ring: Ring, topJointBendingAngle: float):
    return evalTopContactDisp(ringLength=ring.length,
                              topCurveRadius=ring.topCurveRadius,
                              topJointAngle=topJointBendingAngle,
                              topOrientationRF=ring.topOrientationRF)


def bottomReactionDisplacement(ring: Ring, bottomJointBendingAngle: float):
    return evalBottomContactDisp(ringLength=ring.length,
                                 bottomCurveRadius=ring.bottomCurveRadius,
                                 bottomJointAngle=bottomJointBendingAngle)
