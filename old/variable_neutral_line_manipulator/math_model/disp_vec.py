import math
import numpy as np
"""
Based on the frame aligned with the bottom curvature orientation 
(Bottom curvature rotates about the x-axis of the frame)
"""

# Used as a reference, commented out due to simplicity
# def evalDispBetweenCenters(curveRadius:float, ringLength:float, isTop:bool):
#     """
#         Evaluate 
#     """
#     return ringLength/2 - curveRadius if isTop else curveRadius - ringLength/2
    

# Not required for model with tendons within tendon guide as part of free body
def evalKnobDisp(ringLength: float, 
                knobLength: float,
                horizontalDispFromTendonToAxis: float,
                tendonOrientationRF: float) -> np.ndarray:
    """
        Evaluate displacement from ring center to knob
    """
    return np.array((
        horizontalDispFromTendonToAxis*math.cos(tendonOrientationRF),
        horizontalDispFromTendonToAxis*math.sin(tendonOrientationRF),
        knobLength - ringLength/2
    ))

def evalTopGuideEndDisp(ringLength: float,
                        topCurveRadius: float,
                        horizontalDispFromTendonToAxis: float,
                        tendonOrientationRF: float,
                        topOrientationRF: float) -> np.ndarray:
    """
        Evaluate displacement from ring center to any top end of tendon guide
    """
    horizontalDispAlongCurve = horizontalDispFromTendonToAxis*math.sin(tendonOrientationRF - topOrientationRF)
    return np.array((
        horizontalDispFromTendonToAxis*math.cos(tendonOrientationRF),
        horizontalDispFromTendonToAxis*math.sin(tendonOrientationRF),
        math.sqrt(topCurveRadius**2 - horizontalDispAlongCurve**2) + ringLength/2 - topCurveRadius
    ))

   
    
def evalBottomGuideEndDisp(ringLength: float,
                            bottomCurveRadius: float,
                            horizontalDispFromTendonToAxis: float,
                            tendonOrientationRF: float)-> np.ndarray:
    """
        Evaluate displacement from ring center to any bottom end of tendon guide
    """
    horizontalDispAlongCurve = horizontalDispFromTendonToAxis*math.sin(tendonOrientationRF)
    return np.array((
        horizontalDispFromTendonToAxis*math.cos(tendonOrientationRF),
        horizontalDispFromTendonToAxis*math.sin(tendonOrientationRF),
        -math.sqrt(bottomCurveRadius**2 - horizontalDispAlongCurve**2) + bottomCurveRadius - ringLength/2
    ))
    
    
def evalTopContactDisp(ringLength: float,
                    topCurveRadius: float,
                    topJointAngle: float, 
                    topOrientationRF: float):
    """
        Evaluate displacement from ring center to center of top contacting line
    """
    halfJointAngle = topJointAngle/2
    return np.array((
        topCurveRadius*math.sin(halfJointAngle)*math.sin(topOrientationRF),
        -topCurveRadius*math.sin(halfJointAngle)*math.cos(topOrientationRF),
        topCurveRadius*math.cos(halfJointAngle) + ringLength/2 - topCurveRadius
    ))

def evalBottomContactDisp(ringLength: float,
                        bottomCurveRadius: float,
                        bottomJointAngle: float):
    """
        Evaluate displacement from ring center to center of bottom contacting line
    """
    halfJointAngle = bottomJointAngle/2
    return np.array((
        0,
        -bottomCurveRadius*math.sin(halfJointAngle),
        -bottomCurveRadius*math.cos(halfJointAngle) + bottomCurveRadius - ringLength/2
    ))

    




