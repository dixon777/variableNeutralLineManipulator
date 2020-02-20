from .definition import *
from .states import *


def topCableReactionForce(ring: SnakeJointRing, cableState: CableState, topJointBendingAngle: float):
    return (evalKnobComp(cableState.tensionInRing)
            if cableState.isKnob else
            evalTopCableGuideComp(tensionInRing=cableState.tensionInRing,
                                  fricCoef=ring.fricCoef,
                                  topJointAngle=topJointBendingAngle,
                                  topOrientationRF=ring.topOrientationRF))


def bottomCableReactionForce(ring: SnakeJointRing, cableState: CableState, bottomJointBendingAngle: float):
    return evalBottomCableGuideComp(tensionInRing=cableState.tensionInRing,
                                    fricCoef=ring.fricCoef,
                                    bottomJointAngle=bottomJointBendingAngle)


def topCableDisplacement(ring: SnakeJointRing, cableState: CableState):
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


def bottomCableDisplacement(ring: SnakeJointRing, cableState: CableState):
    return evalBottomGuideEndDisp(ringLength=ring.length,
                                  bottomCurveRadius=ring.bottomCurveRadius,
                                  horizontalDispFromCableToAxis=cableState.cableLocation.horizontalDistFromAxis,
                                  cableOrientationRF=cableState.cableLocation.orientationBF - ring.orientationBF)


def topReactionComponent(ring: SnakeJointRing, topJointAngle: float, distalRingBottomReaction: np.ndarray):
    return evalTopContactComp(DRBottomContactCompDRF=distalRingBottomReaction,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def topReactionDisplacement(ring: SnakeJointRing, topJointBendingAngle: float):
    return evalTopContactDisp(ringLength=ring.length,
                              topCurveRadius=ring.topCurveRadius,
                              topJointAngle=topJointBendingAngle,
                              topOrientationRF=ring.topOrientationRF)


def bottomReactionDisplacement(ring: SnakeJointRing, bottomJointBendingAngle: float):
    return evalBottomContactDisp(ringLength=ring.length,
                                 bottomCurveRadius=ring.bottomCurveRadius,
                                 bottomJointAngle=bottomJointBendingAngle)
