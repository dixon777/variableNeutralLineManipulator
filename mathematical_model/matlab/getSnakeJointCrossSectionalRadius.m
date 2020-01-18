function radius = getSnakeJointCrossSectionalRadius(curvatureRadius, curvatureAngle)
radius =  curvatureRadius*sin(curvatureAngle/2);
end