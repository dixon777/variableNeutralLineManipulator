# from snake_joint_model_iterative_solvers import compute
# import math
# print(compute(tensionLeft=5, tensionRight=4.2, curveRadius=2, curvatureAngle=math.pi/2,
#               numJoints=3, length=3, fricCoef=0.0, initJointBendingAngles=[0, 0, 0, ]))

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import math
from variable_neutral_line_manipulator.entities import *
from variable_neutral_line_manipulator.math_components.vector_computation import *
from variable_neutral_line_manipulator.states import *
from variable_neutral_line_manipulator.solvers import *

class Range1d(object):
    def __init__(self):
        self.min = None
        self.max = None
        
    @property
    def bound(self):
        if self.min is None or self.max is None:
            return None
        return (self.min, self.max)
        
    @property
    def total(self):
        if self.min is None:
            return self.max
        elif self.max is None:
            return self.min
        return self.max + self.min
    
    @property
    def avg(self):
        total = self.total
        return None if total is None else total/2
            
    
    @property
    def diff(self):
        if self.min is None:
            return self.max
        elif self.max is None:
            return -self.min
        return self.max - self.min
    
    def update(self, vals):
        if not isinstance(vals, (list, tuple)):
            vals = (vals,)
        for v in vals:
            if self.min is None or self.min > v:
                self.min = v
            elif self.max is None or self.max < v:
                self.max = v
                
    def __repr__(self):
        return f"bound({self.min}, {self.max})"
                
class Range3d(object):
    def __init__(self):
        self.ranges = (Range1d(), Range1d(), Range1d())
    
    @property
    def x(self):
        return self.ranges[0]

    @property
    def y(self):
        return self.ranges[1]

    @property
    def z(self):
        return self.ranges[2]
        
    def update(self, x=(), y=(), z=()):
        self.updateX(x)
        self.updateY(y)
        self.updateZ(z)
        
    def updateX(self, vals):
        self.ranges[0].update(vals)
        
    def updateY(self, vals):
        self.ranges[1].update(vals)
        
    def updateZ(self, vals):
        self.ranges[2].update(vals)
        
    def __repr__(self):
        return f"range3d[x: {self.x}, y: {self.y}, z: {self.z}]"

def enforceRange(ax, range3d: Range3d):
    maxDiff = 0.5* max(r.diff for r in range3d.ranges)
    g = np.mgrid[-1:2:2, -1:2:2, -1:2:2]
    
    for i, j, k in zip(maxDiff*g[0].flatten() + range3d.x.avg, 
                       maxDiff*g[1].flatten() + range3d.y.avg, 
                       maxDiff*g[2].flatten() + range3d.z.avg):
        print(i,j,k)
        ax.plot((i,), (j,), (k,), 'w')
    
def plotFreeBody(ax, state, TFCenter=np.identity(4), range3d: Range3d = None, plotForce=True, plotMoment=False):
    arrow_length_ratio = 0.1
    for s in state.getVectorComponents():
        d = np.matmul(TFCenter, toTFVec(s.disp))
        if plotForce:
            f = np.matmul(TFCenter, toTFVec(s.force))
            ax.quiver(d[0], d[1], d[2], f[0], f[1], f[2],
                  length=0.3, arrow_length_ratio=arrow_length_ratio)
            if range3d is not None:
                t = d + f*arrow_length_ratio
                range3d.update((d[0], t[0]), (d[1], t[1]), (d[2], t[2]))
        if plotMoment:
            m = np.matmul(TFCenter, toTFVec(s.moment))
            ax.quiver(d[0], d[1], d[2], m[0], m[1], m[2],
                  length=0.3, arrow_length_ratio=arrow_length_ratio)   
            if range3d is not None:
                t = d + m*arrow_length_ratio
                range3d.update((d[0], t[0]), (d[1], t[1]), (d[2], t[2]))



def main():
    rings = generateRings(segments=[
        Segment(is1DoF=True,
                numJoints=2,
                ringLength=2,
                orientationBF=0,
                cableHorizontalDistFromAxis=1,
                knobLength=1,
                curveRadius=3),
        Segment(is1DoF=True,
                numJoints=3,
                ringLength=2,
                orientationBF=math.pi/2,
                cableHorizontalDistFromAxis=1,
                knobLength=1,
                curveRadius=3
                )
    ],
        fricCoefRingCable=0.1)

    funcEnd = defineBottomJointAngleFunc(rings[-1],
                                         distalRingState=None,
                                         knobTensions=[2, 1])
    
    res = computeFromEndTensions(rings, endTensions=[[3, 0.2],[2,1]])
    
    range3d = Range3d()

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')
    # plotFreeBody(ax, res.states[-2], 
    #              range3d=range3d,
    #                 plotForce=True)
    for i, s in enumerate(res.states):
        plotFreeBody(ax, s, 
                     res.getTF(i, "c"),
                     range3d,
                     plotForce=True)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_xlim(range3d.x.bound)
    # ax.set_ylim(range3d.y.bound)
    # ax.set_zlim(range3d.z.bound)
    print(range3d)
    enforceRange(ax, range3d)
    plt.show()


if __name__ == "__main__":
    main()
