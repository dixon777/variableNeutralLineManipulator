import math
import numpy as np
"""
Based on the frame aligned with the bottom curvature orientation 
(Bottom curvature rotates about the x-axis of the frame)
"""


def evalDispBetweenCenters(curveRadius:float, ringLength:float, isTop:bool):
    """
        Evaluate 
    """
    return ringLength/2 - curveRadius if isTop else curveRadius - ringLength/2
    

# def evalHorizontalDispFromCableToAxis(CurveRadius: float, 
#                                 curvatureSpanAngle: float):
#     return CurveRadius*math.sin(curvatureSpanAngle/2)

def evalKnobDisp(ringLength: float, 
                knobLength: float,
                horizontalDispFromCableToAxis: float,
                cableOrientationRF: float) -> np.ndarray:
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
        knobLength - ringLength/2
    ))

def evalTopGuideEndDisp(ringLength: float,
                        topCurveRadius: float,
                        horizontalDispFromCableToAxis: float,
                        cableOrientationRF: float,
                        topOrientationRF: float)-> np.ndarray:

    horizontalDispAlongCurve = horizontalDispFromCableToAxis*math.sin(cableOrientationRF - topOrientationRF)
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
        math.sqrt(topCurveRadius**2 - horizontalDispAlongCurve**2) + evalDispBetweenCenters(curveRadius=topCurveRadius, 
                                                                                            ringLength=ringLength, 
                                                                                            isTop=True)
    ))

   
    
def evalBottomGuideEndDisp(ringLength: float,
                            bottomCurveRadius: float,
                            horizontalDispFromCableToAxis: float,
                            cableOrientationRF: float)-> np.ndarray:

    horizontalDispAlongCurve = horizontalDispFromCableToAxis*math.sin(cableOrientationRF)
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
        -math.sqrt(bottomCurveRadius**2 - horizontalDispAlongCurve**2) + evalDispBetweenCenters(curveRadius=bottomCurveRadius, 
                                                                                                ringLength=ringLength, 
                                                                                                isTop=False)
    ))
    
    
def evalTopContactDisp(ringLength: float,
                    topCurveRadius: float,
                    topJointAngle: float, 
                    topOrientationRF: float):
    halfJointAngle = topJointAngle/2
    return np.array((
        topCurveRadius*math.sin(halfJointAngle)*math.sin(topOrientationRF),
        -topCurveRadius*math.sin(halfJointAngle)*math.cos(topOrientationRF),
        topCurveRadius*math.cos(halfJointAngle) + evalDispBetweenCenters(curveRadius=topCurveRadius, 
                                                                        ringLength=ringLength, 
                                                                        isTop=True)
    ))

def evalBottomContactDisp(ringLength: float,
                        bottomCurveRadius: float,
                        bottomJointAngle: float):
    
    halfJointAngle = bottomJointAngle/2
    return np.array((
        0,
        -bottomCurveRadius*math.sin(halfJointAngle),
        -bottomCurveRadius*math.cos(halfJointAngle) + evalDispBetweenCenters(curveRadius=bottomCurveRadius, 
                                                                            ringLength=ringLength, 
                                                                            isTop=False)
    ))

    




