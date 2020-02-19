

class TendonGuideGeometry():
    def __init__(self,
                 distFromAxis,
                 orientationBF,
                 radius):
        self.distFromAxis = distFromAxis
        self.orientationBF = orientationBF
        self.radius = radius
    
    def orientationRF(self, ringOrientationBF):
        return self.orientationBF - ringOrientationBF
    
class TensionKnobGuideGeometry(TendonGuideGeometry):
    def __init__(self, distFromAxis, orientationBF, radius, knobLength, knobSlotRadius):
        super().__init__(distFromAxis, orientationBF, radius)
        self.knobLength = knobLength
        self.knobSlotRadius = knobSlotRadius
        
    
class RingGeometry():
    def __init__(self, 
                 length, 
                 cylindricalRadius,
                 orientationBF,
                 tendonGuideGeometries, 
                 bottomCurveRadius=None, 
                 topOrientationRF=None, 
                 topCurveRadius=None,
                 tendonGuideFilletRadius=0,
                 centerHoleRadius=0,
                 ):
        self.length = length
        self.cylindricalRadius = cylindricalRadius
        self.orientationBF = orientationBF
        self.topOrientationRF =topOrientationRF
        self.bottomCurveRadius = bottomCurveRadius
        self.topCurveRadius = topCurveRadius
        self.tendonGuideGeometries = [] if tendonGuideGeometries is None else tendonGuideGeometries
        self.tendonGuideFilletRadius = tendonGuideFilletRadius
        self.centerHoleRadius = centerHoleRadius
        
        
    @property
    def topOrientationBF(self):
        return self.orientationBF + self.topOrientationRF

            
