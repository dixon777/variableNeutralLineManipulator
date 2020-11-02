from math import floor, pi, acos, atan2
import numpy as np

import pyrr.matrix33 as m3
import pyrr.matrix44 as m4

def remove_dec(val):
    count = 0
    while int(val) != val:
        val *= 10
        count += 1
    return int(val), count

def normalise_to_range(val, max_val):
    return val - floor(val/max_val)*max_val if val is not None else None

def normalise_half_angle(angle):
    return normalise_to_range(angle, pi)

def normalise_angle(angle):
    return normalise_to_range(angle, 2*pi)

def m3_rotation(axis, radian):
    return m3.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()

def m4_translation(vec):
    return m4.create_from_translation(np.array(vec)*1.0).transpose()

def m4_rotation(axis, radian):
    return m4.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()
    
def m4_rotation(axis, radian):
    return m4.create_from_axis_rotation(np.array(axis)*1.0, radian).transpose()


def rot_mat_to_zxz_euler(mat):
    if mat[2,2] -1 < 1e-6 or mat[2,2] + 1 < 1e-6:
        return (acos(mat[0,0]), acos(mat[2,2]), 0.0)
    
    return (atan2(mat[0,2], -mat[1,2]), acos(mat[2,2]), atan2(mat[2,0],mat[2,1]))