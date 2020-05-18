from typing import List

import numpy as np
import math as cal

from .entities import *
from .states import *
from .vec_objects import *
from .math_wrapper import *
from ..common import Logger

@Logger.hierarchy
def evalBottomAngleBound(ring: RingModel, tendonStates: List[TendonModelState]):
    minAngle = 0
    maxAngle = 0
    for cs in tendonStates:
        horizontalDist = (cs.tendonModel.horizontalDistFromAxis *
                          math.sin(cs.tendonModel.orientationBF - ring.orientationBF))
        angle = math.asin(horizontalDist/ring.bottomCurveRadius)
        minAngle = angle if angle < minAngle else minAngle
        maxAngle = angle if angle > maxAngle else maxAngle
    Logger.D(f"evalBottomAngleBound: {(minAngle, maxAngle)}")
    return (minAngle, maxAngle)
class NotWithinBoundError(Exception):
    def __init__(self, lowerValue, upperValue, cut):
        self.lowerValue = lowerValue
        self.upperValue = upperValue
        self.cut = cut
    
    def __repr__(self):
        return f"{self.__class__.__name__}: {self.cut} is not within ({self.lowerValue}, {self.upperValue})"

@Logger.hierarchy
def checkBounds(func, bounds,  **kwargs):
    Logger.D(f"Check values at bounds: {bounds}")
    upperResult = func(bounds[1], **kwargs)
    lowerResult = func(bounds[0], **kwargs)
    if (upperResult[0] > 0 and lowerResult[0] > 0) or (upperResult[0] < 0 and lowerResult[0] < 0):
        raise NotWithinBoundError(
            lowerValue=lowerResult[0], upperValue=upperResult[0], cut=0)


def binarySolver(func, bounds, threshold=0.0001, yieldIteratively=False, **kwargs):
    lowerBound = bounds[0]
    upperBound = bounds[1]
    result = (threshold + 1,)
    while abs(result[0]) > threshold:
        guess = (lowerBound+upperBound)/2
        result = func(guess, **kwargs)

        if yieldIteratively:
            yield result

        if result[0] > 0:
            lowerBound = guess
        else:
            upperBound = guess

    if not yieldIteratively:
        return result


def NewtonSolver(func, xInit=0.0, xDiff=0.001, threshold=0.001, **kwargs):
    x = xInit
    fx = func(x)
    while abs(fx[0]) > threshold:
        # x'=x - f(x)/f'(x)
        x -= fx[0]/(func(x+xDiff)[0] - fx[0])*xDiff
        fx = func(x)
    return fx

@Logger.hierarchy
def _evalBottomJointAngleIndependentComponent(ring: RingModel, tendonStates: List[TendonModelState], distalRingState: RingModelState = None):
    c = VectorComponent()
    Logger.D(f"Independent comp:")
    for cs in tendonStates:
        v = VectorComponent(
            topGuideForceComp(
                ring, cs, None if distalRingState is None else distalRingState.bottomJointAngle),
            topGuideDisp(ring, cs)
        )
        Logger.D(f"Top guide component: {v}",1)
        c += v

    if distalRingState:
        v =  VectorComponent(
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.force),
            topReactionDisp(
                ring, distalRingState.bottomJointAngle),
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.moment)
        )
        Logger.D(f"Top contact component: {v}",)
        c += v
    Logger.D(f"Total: {c}")
    return c

@Logger.hierarchy
def _evalBottomJointAngleDependentComponentFunc(ring: RingModel, tendonStates: List[TendonModelState]):
    disps = [bottomGuideDisp(ring, cs) for cs in tendonStates]
    @Logger.hierarchy
    def __compute(bottomJointAngle):
        Logger.D(f"Dedependent comp: [angle: {bottomJointAngle}]")
        c = VectorComponent()
        for disp, cs in zip(disps, tendonStates):
            v = VectorComponent(
                bottomGuideForceComp(ring, cs, bottomJointAngle),
                disp
            )
            c += v
            Logger.D(f"Bottom guide comp: {v}",1)
        Logger.D(f" Total: {c}")
        return c
    return __compute


# def _getBottomJointAngleIndependentComponents(ring: RingModel, tendonStates: List[TendonModelState], distalRingState: RingModelState = None):
#     components = []
#     for cs in tendonStates:
#         components.append(VectorComponent(
#             topGuideForceComp(
#                 ring, cs, None if distalRingState is None else distalRingState.bottomJointAngle),
#             topGuideDisp(ring, cs)
#         ))

#     if distalRingState:
#         components.append(VectorComponent(
#             topReactionComp(ring, distalRingState.bottomJointAngle,
#                                  distalRingState.bottomContactReactionComponent.force),
#             topReactionDisp(
#                 ring, distalRingState.bottomJointAngle),
#             topReactionComp(ring, distalRingState.bottomJointAngle,
#                                  distalRingState.bottomContactReactionComponent.moment)
#         ))
#     return components


# def _getBottomJointAngleDependentComponentFunc(ring: RingModel, tendonStates: List[TendonModelState]):
#     disps = [bottomGuideDisp(ring, cs) for cs in tendonStates]

#     def __compute(bottomJointAngle):
#         return [VectorComponent(
#                 bottomGuideForceComp(ring, cs, bottomJointAngle),
#                 disp
#                 ) for disp, cs in zip(disps, tendonStates)]
#     return __compute





def evalAllTendonStates(ring: RingModel, distalRingState: RingModelState, knobTensions: List[float]):
    tendonStates = TendonModelState.createKnobs(
        ring.knobTendonModels, knobTensions if knobTensions else [])
    if distalRingState is not None:
        tendonStates += distalRingState.getTendonModelStatesProximalRing()
    return tendonStates

@Logger.hierarchy
def defineBottomJointAngleFunc(ring: RingModel, distalRingState: RingModelState = None, knobTensions: List[float] = []) -> RingModelState:
    tendonStates = evalAllTendonStates(ring, distalRingState, knobTensions)
    independentComponent = _evalBottomJointAngleIndependentComponent(
        ring, tendonStates, distalRingState)
    dependentComponentFunc = _evalBottomJointAngleDependentComponentFunc(
        ring, tendonStates)

    @Logger.hierarchy
    def __bottomJointAngleCompute(bottomJointAngle):
        Logger.D(f"__bottomJointAngleCompute: [angle: {bottomJointAngle}]")
        sumComponent = (independentComponent +
                        dependentComponentFunc(bottomJointAngle))
        Logger.D(f"Sum comp without bottom contact: {sumComponent}",1)
        pointForceComponent = VectorComponent(-sumComponent.force,
                                        bottomReactionDisplacement(ring, bottomJointAngle))
        Logger.D(f"Bottom contact point force: {pointForceComponent}",1)
        reactionTorque = -(sumComponent.moment +
                           pointForceComponent.momentByForce)
        Logger.D(f"Bottom contact moment: {reactionTorque}",1)
        return reactionTorque[0], RingModelState(ring, tendonStates, ContactReactionComponent(pointForceComponent.force, reactionTorque), bottomJointAngle, distalRingState)

    return __bottomJointAngleCompute

@Logger.hierarchy
def computeFromEndTensions(rings: List[float], endTensions: List[List[float]], callback=None) -> StateResult:
    Logger.D(f"computeFromEndTensions()")
    Logger.D(f"endTensions: {endTensions}")
    tensionIndex = -1
    distalRingState = None
    for i, r in enumerate(reversed(rings)):
        Logger.D(f"Compute ring {i}")
        if r.numKnobs():
            endTension = endTensions[tensionIndex]
            tensionIndex -= 1
        else:
            endTension = None
        func = defineBottomJointAngleFunc(r, distalRingState, endTension)
        angleBound = evalBottomAngleBound(
                r, evalAllTendonStates(r, distalRingState, endTension))
        if callback:
            callback(func=func, angleBound=angleBound)
        try:
            checkBounds(func, angleBound)
        except NotWithinBoundError as err:
            Logger.D(f"Check bound error: {err.__repr__()}")
            return StateResult(mostProximalRingState=distalRingState, error=err)
        res, distalRingState = NewtonSolver(
            func, distalRingState.bottomJointAngle if distalRingState else 0.0)

    return StateResult(mostProximalRingState=distalRingState)
