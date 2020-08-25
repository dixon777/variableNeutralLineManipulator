from math import floor, pi
import numpy as np

import pyrr.matrix33 as m3
import pyrr.matrix44 as m4


def normalise_angle(angle):
    return angle - floor(angle/2/pi)*2*pi if angle is not None else None
    
