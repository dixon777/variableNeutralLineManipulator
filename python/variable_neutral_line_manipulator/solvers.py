from typing import List

import numpy as np
import math as cal

from .entities import *
from .states import *
from .vector_components import *
from .math_wrapper import *


class NotWithinBoundError(Exception):
    def __init__(self, lowerValue, upperValue, cut):
        self.lowerValue = lowerValue
        self.upperValue = upperValue
        self.cut = cut


def checkBounds(func, bounds,  **kwargs):
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


def _evalBottomJointAngleIndependentComponent(ring: Ring, tendonStates: List[TendonState], distalRingState: RingState = None):
    c = Component()
    for cs in tendonStates:
        c += Component(
            topGuideForceComp(
                ring, cs, None if distalRingState is None else distalRingState.bottomJointAngle),
            topGuideDisp(ring, cs)
        )

    if distalRingState:
        c += Component(
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.force),
            topReactionDisplacement(
                ring, distalRingState.bottomJointAngle),
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.moment)
        )
    return c


def _evalBottomJointAngleDependentComponentFunc(ring: Ring, tendonStates: List[TendonState]):
    disps = [bottomGuideDisp(ring, cs) for cs in tendonStates]

    def __compute(bottomJointAngle):
        c = Component()
        for disp, cs in zip(disps, tendonStates):
            c += Component(
                bottomGuideForceComp(ring, cs, bottomJointAngle),
                disp
            )
        return c
    return __compute


def _getBottomJointAngleIndependentComponents(ring: Ring, tendonStates: List[TendonState], distalRingState: RingState = None):
    components = []
    for cs in tendonStates:
        components.append(Component(
            topGuideForceComp(
                ring, cs, None if distalRingState is None else distalRingState.bottomJointAngle),
            topGuideDisp(ring, cs)
        ))

    if distalRingState:
        components.append(Component(
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.force),
            topReactionDisplacement(
                ring, distalRingState.bottomJointAngle),
            topReactionComp(ring, distalRingState.bottomJointAngle,
                                 distalRingState.bottomContactReactionComponent.moment)
        ))
    return components


def _getBottomJointAngleDependentComponentFunc(ring: Ring, tendonStates: List[TendonState]):
    disps = [bottomGuideDisp(ring, cs) for cs in tendonStates]

    def __compute(bottomJointAngle):
        return [Component(
                bottomGuideForceComp(ring, cs, bottomJointAngle),
                disp
                ) for disp, cs in zip(disps, tendonStates)]
    return __compute


def evalBottomAngleBound(ring: Ring, tendonStates: List[TendonState]):
    minAngle = 0
    maxAngle = 0
    for cs in tendonStates:
        horizontalDist = (cs.tendonLocation.horizontalDistFromAxis *
                          math.sin(cs.tendonLocation.orientationBF - ring.orientationBF))
        angle = math.asin(horizontalDist/ring.bottomCurveRadius)
        minAngle = angle if angle < minAngle else minAngle
        maxAngle = angle if angle > maxAngle else maxAngle
    return (minAngle, maxAngle)


def computeAllTendonStates(ring: Ring, distalRingState: RingState, knobTensions: List[float]):
    tendonStates = TendonState.createKnobs(
        ring.knobTendonLocations, knobTensions if knobTensions else [])
    if distalRingState is not None:
        tendonStates += distalRingState.getTendonStatesProximalRing()
    return tendonStates


def defineBottomJointAngleFunc(ring: Ring, distalRingState: RingState = None, knobTensions: List[float] = []) -> RingState:
    tendonStates = computeAllTendonStates(ring, distalRingState, knobTensions)
    independentComponent = _evalBottomJointAngleIndependentComponent(
        ring, tendonStates, distalRingState)
    dependentComponentFunc = _evalBottomJointAngleDependentComponentFunc(
        ring, tendonStates)

    def __compute(bottomJointAngle):
        sumComponent = (independentComponent +
                        dependentComponentFunc(bottomJointAngle))
        pointForceComponent = Component(-sumComponent.force,
                                        bottomReactionDisplacement(ring, bottomJointAngle))
        reactionTorque = -(sumComponent.moment +
                           pointForceComponent.momentByForce)
        return reactionTorque[0], RingState(ring, tendonStates, ContactReactionComponent(pointForceComponent.force, reactionTorque), bottomJointAngle, distalRingState)

    return __compute


def computeFromEndTensions(rings: List[float], endTensions: List[List[float]]) -> StateResult:
    tensionIndex = -1
    distalRingState = None
    for r in reversed(rings):
        if r.numKnobs():
            endTension = endTensions[tensionIndex]
            tensionIndex -= 1
        else:
            endTension = None
        func = defineBottomJointAngleFunc(r, distalRingState, endTension)
        try:
            checkBounds(func, evalBottomAngleBound(
                r, computeAllTendonStates(r, distalRingState, endTension)))
        except Exception as error:
            return StateResult(mostProximalRingState=distalRingState)
        res, distalRingState = NewtonSolver(
            func, distalRingState.bottomJointAngle if distalRingState else 0.0)

    return StateResult(mostProximalRingState=distalRingState)
