import numpy as np

from .force_comp import *
from .disp_comp import *
from .entities import *
from .states import *


def topGuideForceComp(ring: RingModel, tendonState: TendonModelState, topJointAngle: float):
    return (np.zeros(3) if tendonState.isKnob else
            evalTopGuideCompWithTendon(tensionInRing=tendonState.tensionInRing,
                                  fricCoef=ring.fricCoefRingTendon,
                                  topJointAngle=topJointAngle,
                                  topOrientationRF=ring.topOrientationRF))


def bottomGuideForceComp(ring: RingModel, tendonState: TendonModelState, bottomJointAngle: float):
    return evalBottomGuideCompWithTendon(tensionInRing=tendonState.tensionInRing,
                                    fricCoef=ring.fricCoefRingTendon,
                                    bottomJointAngle=bottomJointAngle)


def topGuideDisp(ring: RingModel, tendonState: TendonModelState):
    return (evalKnobDisp(ringLength=ring.length,
                        knobLength=tendonState.tendonModel.knobLength,
                        horizontalDispFromTendonToAxis=tendonState.tendonModel.horizontalDistFromAxis,
                        tendonOrientationRF=tendonState.tendonModel.orientationBF - ring.orientationBF) 
        if tendonState.isKnob else 
        evalTopGuideEndDisp(ringLength=ring.length,
                            topCurveRadius=ring.topCurveRadius,
                            horizontalDispFromTendonToAxis=tendonState.tendonModel.horizontalDistFromAxis,
                            tendonOrientationRF=tendonState.tendonModel.orientationBF -
                            ring.orientationBF,
                            topOrientationRF=ring.topOrientationRF))


def bottomGuideDisp(ring: RingModel, tendonState: TendonModelState):
    return evalBottomGuideEndDisp(ringLength=ring.length,
                                  bottomCurveRadius=ring.bottomCurveRadius,
                                  horizontalDispFromTendonToAxis=tendonState.tendonModel.horizontalDistFromAxis,
                                  tendonOrientationRF=tendonState.tendonModel.orientationBF - ring.orientationBF)


def topReactionComp(ring: RingModel, topJointAngle: float, distalRingBottomReaction: np.ndarray):
    return changeContactCompFrame(DRBottomContactCompDRF=distalRingBottomReaction,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def topReactionDisp(ring: RingModel, topJointAngle: float):
    return evalTopContactDisp(ringLength=ring.length,
                              topCurveRadius=ring.topCurveRadius,
                              topJointAngle=topJointAngle,
                              topOrientationRF=ring.topOrientationRF)


def bottomReactionDisplacement(ring: RingModel, bottomJointAngle: float):
    return evalBottomContactDisp(ringLength=ring.length,
                                 bottomCurveRadius=ring.bottomCurveRadius,
                                 bottomJointAngle=bottomJointAngle)
    
 