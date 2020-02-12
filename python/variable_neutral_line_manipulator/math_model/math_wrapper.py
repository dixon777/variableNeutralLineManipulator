import numpy as np

from .force_comp import *
from .disp_comp import *


def topGuideForceComp(ring: Ring, tendonState: TendonState, topJointAngle: float):
    return (np.zeros(3) if tendonState.isKnob else
            evalTopGuideCompWithTendon(tensionInRing=tendonState.tensionInRing,
                                  fricCoef=ring.fricCoefRingTendon,
                                  topJointAngle=topJointAngle,
                                  topOrientationRF=ring.topOrientationRF))


def bottomGuideForceComp(ring: Ring, tendonState: TendonState, bottomJointAngle: float):
    return evalBottomGuideCompWithTendon(tensionInRing=tendonState.tensionInRing,
                                    fricCoef=ring.fricCoefRingTendon,
                                    bottomJointAngle=bottomJointAngle)


def topGuideDisp(ring: Ring, tendonState: TendonState):
    return (evalKnobDisp(ringLength=ring.length,
                        knobLength=tendonState.tendonLocation.knobLength,
                        horizontalDispFromTendonToAxis=tendonState.tendonLocation.horizontalDistFromAxis,
                        tendonOrientationRF=tendonState.tendonLocation.orientationBF - ring.orientationBF) 
        if tendonState.isKnob else 
        evalTopGuideEndDisp(ringLength=ring.length,
                            topCurveRadius=ring.topCurveRadius,
                            horizontalDispFromTendonToAxis=tendonState.tendonLocation.horizontalDistFromAxis,
                            tendonOrientationRF=tendonState.tendonLocation.orientationBF -
                            ring.orientationBF,
                            topOrientationRF=ring.topOrientationRF))


def bottomGuideDisp(ring: Ring, tendonState: TendonState):
    return evalBottomGuideEndDisp(ringLength=ring.length,
                                  bottomCurveRadius=ring.bottomCurveRadius,
                                  horizontalDispFromTendonToAxis=tendonState.tendonLocation.horizontalDistFromAxis,
                                  tendonOrientationRF=tendonState.tendonLocation.orientationBF - ring.orientationBF)


def topReactionComp(ring: Ring, topJointAngle: float, distalRingBottomReaction: np.ndarray):
    return changeContactCompFrame(DRBottomContactCompDRF=distalRingBottomReaction,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def topReactionDisp(ring: Ring, topJointAngle: float):
    return evalTopContactDisp(ringLength=ring.length,
                              topCurveRadius=ring.topCurveRadius,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def bottomReactionDisplacement(ring: Ring, bottomJointAngle: float):
    return evalBottomContactDisp(ringLength=ring.length,
                                 bottomCurveRadius=ring.bottomCurveRadius,
                                 bottomJointAngle=bottomJointAngle)
