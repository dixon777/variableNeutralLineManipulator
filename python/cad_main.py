import math, os,datetime

from variable_neutral_line_manipulator.cad.freecad import *
        
def main():
    
    rg = RingGeometry(
        length = 5,
        cylindricalRadius = 2.5,
        orientationBF = 0,
        topOrientationRF = math.pi/2,
        bottomCurveRadius = 3,
        topCurveRadius = 3.5,
        tendonGuideFilletRadius=0.08,
        centerHoleRadius=1,
        tendonGuideGeometries=[
            TensionKnobGuideGeometry(
                distFromAxis=1.8,
                orientationBF=i*math.pi/4,
                radius=0.5,  
                knobLength = 3, 
                knobSlotRadius=0.7,
                              
            ) for i in range(8)
        ],
    )
    cad = RingCAD(rg, "ring1")
    cad.generate()
    cad.saveSrc()
    
    
if __name__ == "__main__":
    main()