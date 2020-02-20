import pytest
import random as rd

from ..math_components.force_components import evalCapstan
from ..math_components.vector_computation import *
from ..entities import *
from ..states import *
from .test_entities import ring2DoF


def test_cableState():
    ts = [rd.random()*20 for _ in range(10)]
    ds = [rd.random()*20 for _ in range(10)]
    os = [rd.random()*20 for _ in range(10)]
    for t, d, o in zip(ts, ds,os):
        cl = CableLocation(orientationBF=o, horizontalDistFromAxis=d,knobLength=4)
        state = CableState(cableLocation=cl, tensionInRing=t, isKnob=False)
        assert(state.cableLocation == cl)
        assert(state.tensionInRing == t)
        assert(not state.isKnob)
    
        
def test_cableState_toProximalState():
    ts = [rd.random()*20 for _ in range(10)]
    ds = [rd.random()*20 for _ in range(10)]
    os = [rd.random()*20 for _ in range(10)]
    for t, d, o in zip(ts, ds,os):
        cl = CableLocation(orientationBF=o, horizontalDistFromAxis=d,knobLength=4)
        cs = CableState(cableLocation=cl, tensionInRing=t, isKnob=True).toProximalRingState(fricCoef=0.4, jointAngle=5)
        assert(cs.tensionInRing == evalCapstan(tensionEnd=t, fricCoef=0.4, totalAngle=5))
        assert(not cs.isKnob)
    

def test_contactReactionComponent_toProximalState():
    for _ in range(20):
        force = [rd.random()*3 for _ in range(3)]
        moment=[rd.random()*4 for _ in range(3)]
        jointAngle = rd.random()*1.5
        orientationDiffRF = rd.random()*3
        c = ContactReactionComponent(force=force, moment=moment)
        d = c.toProximalRingState(jointAngle=jointAngle,orientationDiffRF=orientationDiffRF)
        
        m = np.matmul(getRMFromAxis(np.array((0,0,1)), jointAngle),getRMFromAxis(np.array((1,0,0)), orientationDiffRF))
        assert(c.force == np.dot)
        assert(c.moment == m*moment)
            
        
        
    