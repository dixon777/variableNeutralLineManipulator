from snake_joint_model_iterative_solvers import compute
import math
print(compute(tensionLeft=5, tensionRight=4.2, curvatureRadius=2, curvatureAngle=math.pi/2,
              numJoints=3, length=3, frictionCoefficient=0.0, initJointBendingAngles=[0, 0, 0, ]))
