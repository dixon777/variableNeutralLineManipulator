import math, os,datetime

from variable_neutral_line_manipulator.cad.freecad import *
        
def main():
    
    rg = RingGeometry(
        length = 5,
        cylindricalRadius = 3,
        orientationBF = 0,
        # topOrientationRF = math.pi/2,
        bottomCurveRadius = 3.5,
        # topCurveRadius = 3.5,
        tendonGuideFilletRadius=0.08,
        centerHoleRadius=1,
        tendonGuideGeometries=[
            TendonGuideGeometry(
                distFromAxis=2.0,
                orientationBF=i*math.pi/4,
                radius=0.5,  
                # knobLength = 3, 
                # knobSlotRadius=0.7,
                              
            ) for i in range(8)
        ],
    )
    cad = RingCAD(rg, "3_b")
    cad.generate()
    cad.saveSrc(os.path.join(os.path.realpath(os.getcwd()), "outputs/"))
    
    
if __name__ == "__main__":
    main()