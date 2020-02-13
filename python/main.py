import numpy as np
np.set_printoptions(3, suppress =True)
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

import math

from variable_neutral_line_manipulator.math_model.helper_functions import *
from variable_neutral_line_manipulator.math_model.entities import *
from variable_neutral_line_manipulator.math_model.states import *
from variable_neutral_line_manipulator.math_model.solvers import *


# from variable_neutral_line_manipulator.display.display import *
# from variable_neutral_line_manipulator.display.ranges import *

    
# def plotFreeBody(ax, state, TFCenter=np.identity(4), range3d: Range3d = None, plotForce=True, plotMoment=False):
#     arrow_length_ratio = 0.1
#     for s in state.getVectorComponents():
#         d = np.matmul(TFCenter, toTFVec(s.disp))
#         if plotForce:
#             f = np.matmul(TFCenter, toTFVec(s.force))
#             ax.quiver(d[0], d[1], d[2], f[0], f[1], f[2],
#                   length=0.3, arrow_length_ratio=arrow_length_ratio)
#             if range3d is not None:
#                 t = d + f*arrow_length_ratio
#                 range3d.update((d[0], t[0]), (d[1], t[1]), (d[2], t[2]))
#         if plotMoment:
#             m = np.matmul(TFCenter, toTFVec(s.moment))
#             ax.quiver(d[0], d[1], d[2], m[0], m[1], m[2],
#                   length=0.3, arrow_length_ratio=arrow_length_ratio)   
#             if range3d is not None:
#                 t = d + m*arrow_length_ratio
#                 range3d.update((d[0], t[0]), (d[1], t[1]), (d[2], t[2]))


def main():
    rings = generateRingModels(segments=[
        SegmentModel(is1DoF=True,
                numJoints=4,
                ringLength=2,
                orientationBF=0,
                tendonHorizontalDistFromAxis=0.5,
                knobLength=1,
                curveRadius=6),
        # SegmentModel(is1DoF=True,
        #         numJoints=3,
        #         ringLength=2,
        #         orientationBF=0,
        #         tendonHorizontalDistFromAxis=1.5,
        #         knobLength=1,
        #         curveRadius=3
        #         )
    ],
        fricCoefRingTendon=0.0)

    # funcEnd = defineBottomJointAngleFunc(rings[-1],
    #                                      distalRingState=None,
    #                                      knobTensions=[1, 1])
    # funcEnd(0)
    
    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    
    # def _plot(func, angleBound):
    #     x = np.linspace(*angleBound)
    #     y = []
    #     for i in x:
    #         res, _ = func(i)
    #         y.append(res)
    #     ax.plot(x,y)
    #     plt.show()
    
    # _plot(funcEnd, (-1, 1))
        
    
    res = computeFromEndTensions(rings, endTensions=[[0,0],[1.1,1]])

    
    print(len(res.states))
    # plt.show()
    # range3d = Range3d()

    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1, projection='3d')
    # plotRings(ax, res)
    # # plotFreeBody(ax, res.states[-2], 
    # #              range3d=range3d,
    # #                 plotForce=True)
    # # for i, s in enumerate(res.states):
    # #     plotFreeBody(ax, s, 
    # #                  res.getTF(i, "c"),
    # #                  range3d,
    # #                  plotForce=True)
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # # ax.set_xlim(range3d.x.bound)
    # # ax.set_ylim(range3d.y.bound)
    # # ax.set_zlim(range3d.z.bound)
    # # enforceRange(ax, range3d)
    # plt.show()

# def main2():
    
#     ring = RingGeometry(2, 1, 0.3, 1.5, 1, 1.5)
#     fig = plt.figure()
#     ax = fig.add_subplot(1, 1, 1, projection='3d')
#     t = np.identity(4)
#     a = math.pi/3
#     t[1:3,1:3] = ((math.cos(a), -math.sin(a)), (math.sin(a), math.cos(a)))
#     plotRing(ax, ring, transform=t, angleDivision=40) 
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plt.show()
    

if __name__ == "__main__":
    main()
