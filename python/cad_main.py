import math, os,datetime

from variable_neutral_line_manipulator.cad.freecad import *

def rg1():
    return RingGeometry(
        length = 5,
        cylindricalRadius = 3,
        orientationBF = 0,
        topOrientationRF = math.pi/2,
        bottomCurveRadius = 4.5,
        topCurveRadius = 4.5,
        tendonGuideFilletRadius=0.1,
        centerHoleRadius=1,
        tendonGuideGeometries=[
            TendonGuideGeometry(
                distFromAxis=2.0,
                orientationBF=i*math.pi/4,
                radius=0.5,                                
            ) for i in range(8)
        ],
    )
    
def rg2():
    return RingGeometry(
        length = 5,
        cylindricalRadius = 3,
        orientationBF = 0,
        topOrientationRF = math.pi/4,
        bottomCurveRadius = 4.5,
        topCurveRadius = 4.5,
        tendonGuideFilletRadius=0.1,
        centerHoleRadius=1,
        tendonGuideGeometries=[
            TendonGuideGeometry(
                distFromAxis=2.0,
                orientationBF=i*math.pi/2 + math.pi/4,
                radius=0.5,  
                # knobLength = 3, 
                # knobSlotRadius=0.7,
                              
            ) for i in range(4)
        ] + [
            TendonKnobGuideGeometry(
                distFromAxis=2.0,
                orientationBF=i*math.pi/2,
                radius=0.5,  
                knobLength = 3.4, 
                knobSlotRadius=0.7,
                              
            ) for i in range(4)
        ],
    )
    
def main():
    rg = rg1()
    
    cad = RingCAD(rg, "it_45")
    cad.generate()
    path = os.path.join(os.path.realpath(os.getcwd()), "outputs/")
    cad.saveSrc(path)
    cad.saveStl(path)
    
    
if __name__ == "__main__":
    main()