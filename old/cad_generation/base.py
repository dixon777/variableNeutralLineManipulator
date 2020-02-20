import openpyscad as ops
import os
import math
import numpy as np

def cutCurvature(shape, length, outerDiameter, curvatureRadius, startOrientation):
    curvatureShape = (ops.Cube([outerDiameter+1, outerDiameter+1, length/2+1], center=True).translate([0,0, length/2 - curvatureRadius + math.sqrt(curvatureRadius**2-(length/2)**2)]) - \
        ops.Cylinder(h=length+3, r=curvatureRadius, center=True, _fn=300).rotate(v=[0,1,0], a=90).translate([0,0,length/2 - curvatureRadius]))
    if not startOrientation == 0:
        curvatureShape = curvatureShape.rotate(a=math.degrees(startOrientation), v=[0,0,1])
    return shape - curvatureShape
        
def cutHolesRevolute(shape, length, diameter, displacementFromAxis, startOrientation, density, span=math.pi*2):
    for orientation in np.linspace(0, span, density+1,):
        shape -= ops.Cylinder(h=length, 
                          d=diameter, 
                          center=True, _fn=100).translate([displacementFromAxis*math.cos(orientation), displacementFromAxis*math.sin(orientation), 0])
    return shape


def ringBase(length, outerDiameter, curvatureRadius, cableGuideDiameter, cableGuideDisplacementFromAxis):
    s = ops.Cylinder(h=length, d=outerDiameter, center=True, _fn=300)
    s = cutCurvature(s, length, outerDiameter, curvatureRadius, math.pi/4)
    

    s = cutHolesRevolute(s, length+1, cableGuideDiameter, cableGuideDisplacementFromAxis, 0, 8)
    return s

def main():
    s = ringBase(5, 5, 4, 1, 1.8)
    s.write(os.path.join(os.path.dirname(os.path.realpath(__file__)),"sample.scad"))
    
if __name__ == "__main__":
    main()



c1 = ops.Cube([10, 50, 10])
c2 = ops.Cube([20, 10, 10])
