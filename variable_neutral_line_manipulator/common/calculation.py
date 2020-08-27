from math import floor, pi
import numpy as np

import pyrr.matrix33 as m3
import pyrr.matrix44 as m4

def normalise_to_range(val, max_val):
    return val - floor(val/max_val)*max_val if val is not None else None

def normalise_angle(angle):
    return normalise_to_range(angle, 2*pi)
    
