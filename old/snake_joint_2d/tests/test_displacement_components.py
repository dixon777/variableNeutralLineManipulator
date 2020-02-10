import math
import numpy as np

import snake_joint_2d.displacement_components as hf

def test_cornerDisplacementFromCentroid():
    length = 20
    curvatureRadius = 20
    curvatureAngle = math.pi/4
    res = hf.cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=True, isLeft=True)
    assert(all(res == (np.array((0, length/2-curvatureRadius)) + np.array((-curvatureRadius*math.sin(curvatureAngle/2), curvatureRadius*math.cos(curvatureAngle/2))))))
    res = hf.cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=True, isLeft=False)
    assert(all(res == (np.array((0, length/2-curvatureRadius)) + np.array((curvatureRadius*math.sin(curvatureAngle/2), curvatureRadius*math.cos(curvatureAngle/2))))))
    res = hf.cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=True)
    assert(all(res == (np.array((0, curvatureRadius-length/2)) + np.array((-curvatureRadius*math.sin(curvatureAngle/2), -curvatureRadius*math.cos(curvatureAngle/2))))))
    res = hf.cornerDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, curvatureAngle=curvatureAngle, isTop=False, isLeft=False)
    assert(all(res == (np.array((0, curvatureRadius-length/2)) + np.array((curvatureRadius*math.sin(curvatureAngle/2), -curvatureRadius*math.cos(curvatureAngle/2))))))
    
    
def test_contactingDisplacementFromCentroid():
    length = 20
    curvatureRadius = 18
    jointBendingAngle = 0.44
    
    res = hf.contactingDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, jointBendingAngle=jointBendingAngle, isTop=True)
    assert(all(res == (np.array((0, length/2-curvatureRadius)) + np.array((-curvatureRadius*math.sin(jointBendingAngle/2), curvatureRadius*math.cos(jointBendingAngle/2))))))
    res = hf.contactingDisplacementFromCentroid(length=length, curvatureRadius=curvatureRadius, jointBendingAngle=jointBendingAngle, isTop=False)
    assert(all(res == (np.array((0, curvatureRadius-length/2)) + np.array((-curvatureRadius*math.sin(jointBendingAngle/2), -curvatureRadius*math.cos(jointBendingAngle/2))))))
