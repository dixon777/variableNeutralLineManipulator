import math
import numpy as np
import matplotlib.pyplot as plt
import snake_joint_2d.solvers as its

def main():
    curvatureAngle = math.pi/4
    func = its.defineEndPieceIteractiveFunc(tensionLeftBeforeCableGuide=1.0, tensionRightBeforeCableGuide=0.3, curvatureAngle=curvatureAngle, curvatureRadius=5, frictionCoefficient=0.8, length=3)

    x = []
    y =[]
    for angle in np.arange(-curvatureAngle/2, curvatureAngle/2, 0.01):
        res, _, _, _ = func(angle, returnResultNormalFrictionOnly=True)
        x.append(angle)
        y.append(res)
    plt.plot(x,y)
    plt.show()
    
if __name__ == "__main__":
    main()


