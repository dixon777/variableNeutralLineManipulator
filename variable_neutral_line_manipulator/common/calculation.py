from math import cos
import numpy as np
from ..util import m4_rotation, m4_translation

def eval_proximal_top_to_distal_bottom_TF(jointAngle: float, curveRadius: float):
    rot = m4_rotation((1.0, 0, 0), jointAngle/2)
    return np.matmul(rot,
                     np.matmul(m4_translation((0.0, 0, 2*curveRadius*(1-cos(jointAngle/2)))),
                               rot))  # (Wrong) Trick: m_rot * m_trans * m_rot = m_rot * (m_trans + m_rot)