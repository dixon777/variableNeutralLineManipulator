from typing import List

import numpy as np
import math as cal

from .definition import *
from .states import *
from .math_components_wrapper import *

        
class NotWithinBoundError(Exception):
    def __init__(self, lowerValue, upperValue, cut):
        self.lowerValue = lowerValue
        self.upperValue = upperValue
        self.cut = cut
        
def checkBounds(func, bounds,  **kwargs):
    upperResult = func(bounds[1], **kwargs)
    lowerResult = func(bounds[0], **kwargs)
    if (upperResult[0] > 0 and lowerResult[0] > 0) or (upperResult[0] < 0 and lowerResult[0] < 0):
        raise NotWithinBoundError(lowerValue=lowerResult[0], upperValue=upperResult[0], cut=0)
    

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

def _getBottomJointAngleIndependentComponents(ring: SnakeJointRing, cableStates: List[CableState], distalRingState: SnakeJointRingState=None):
    components = []
    for cs in cableStates:
        components.append(Component(
            topCableReactionForce(ring, cs, None if distalRingState is None else distalRingState.bottomJointBendingAngle),
            topCableDisplacement(ring, cs)
        ))
    
    if distalRingState is not None:
        components.append(Component(
            topReactionComponent(ring, distalRingState.bottomJointBendingAngle, distalRingState.bottomReactionState.force),
            topReactionDisplacement(ring, distalRingState.bottomJointBendingAngle),
            topReactionComponent(ring, distalRingState.bottomJointBendingAngle, distalRingState.bottomReactionState.moment)
        ))
    return components

def _getBottomJointAngleDependentComponentFuncs(ring: SnakeJointRing, cableStates: List[CableState]):
    funcs = []
    for cs in cableStates:
        disp = bottomCableDisplacement(ring, cs)
        def __compute(bottomJointBendingAngle):
            return Component(
                bottomCableReactionForce(ring, cs, bottomJointBendingAngle),
                disp
            )
        funcs.append(__compute)
    return funcs

def evalBottomAngleBound(ring:SnakeJointRing, cableStates: List[CableState]):
    minAngle = 0
    maxAngle = 0
    for cs in cableStates:
        horizontalDist = cs.cableLocation.horizontalDistFromAxis*math.sin(cs.cableLocation.orientationBF - ring.orientationBF)
        angle = math.asin(horizontalDist/ring.bottomCurveRadius)
        minAngle = angle if angle < minAngle else minAngle
        maxAngle = angle if angle > maxAngle else maxAngle
    return (minAngle, maxAngle)
        
        
def evalRingCableStates(ring: SnakeJointRing, distalRingState: SnakeJointRingState, knobTensions:List[float]):
    cableStates = CableState.createKnobs(ring.knobCableLocations, knobTensions if knobTensions else []) 
    if distalRingState is not None:
        cableStates += distalRingState.getCableStatesProximalRing()
    return cableStates

def defineBottomJointAngleFunc(ring: SnakeJointRing, distalRingState: SnakeJointRingState=None, knobTensions: List[float]=[]) -> SnakeJointRingState:
    cableStates = evalRingCableStates(ring, distalRingState, knobTensions)
    independentComponents = _getBottomJointAngleIndependentComponents(ring, cableStates, distalRingState)
    dependentComponentFuncs = _getBottomJointAngleDependentComponentFuncs(ring, cableStates)
    def __compute(bottomJointBendingAngle):
        components = independentComponents + [f(bottomJointBendingAngle) for f in dependentComponentFuncs]
        sumComponent = sum(components)
        pointForceComponent = Component(-sumComponent.force, bottomReactionDisplacement(ring, bottomJointBendingAngle))
        reactionTorque = -(sumComponent.moment + pointForceComponent.momentByForce)
        return reactionTorque[0], SnakeJointRingState(ring, cableStates, ReactionState(pointForceComponent.force, reactionTorque), bottomJointBendingAngle, distalRingState)
        
    return __compute