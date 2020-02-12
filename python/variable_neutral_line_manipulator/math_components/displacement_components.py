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
    

# Not required for model with tendon in the cable guide removed
def evalKnobDisp(ringLength: float, 
                knobLength: float,
                horizontalDispFromCableToAxis: float,
                cableOrientationRF: float) -> np.ndarray:
    """
        Evaluate displacement from ring center to knob
    """
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
        knobLength - ringLength/2
    ))

def evalTopGuideEndDisp(ringLength: float,
                        topCurveRadius: float,
                        horizontalDispFromCableToAxis: float,
                        cableOrientationRF: float,
                        topOrientationRF: float) -> np.ndarray:
    """
        Evaluate displacement from ring center to any top end of tendon guide
    """
    horizontalDispAlongCurve = horizontalDispFromCableToAxis*math.sin(cableOrientationRF - topOrientationRF)
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
        math.sqrt(topCurveRadius**2 - horizontalDispAlongCurve**2) + ringLength/2 - topCurveRadius
    ))

   
    
def evalBottomGuideEndDisp(ringLength: float,
                            bottomCurveRadius: float,
                            horizontalDispFromCableToAxis: float,
                            cableOrientationRF: float)-> np.ndarray:
    """
        Evaluate displacement from ring center to any bottom end of tendon guide
    """
    horizontalDispAlongCurve = horizontalDispFromCableToAxis*math.sin(cableOrientationRF)
    return np.array((
        horizontalDispFromCableToAxis*math.cos(cableOrientationRF),
        horizontalDispFromCableToAxis*math.sin(cableOrientationRF),
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

    




