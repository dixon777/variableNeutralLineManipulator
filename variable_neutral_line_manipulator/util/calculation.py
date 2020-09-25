from math import floor, pi
import numpy as np

def normalise_to_range(val, max_val):
    return val - floor(val/max_val)*max_val if val is not None else None

def normalise_half_angle(angle):
    return normalise_to_range(angle, pi)

def normalise_angle(angle):
    return normalise_to_range(angle, 2*pi)
    
