from math import sin, cos, sqrt
import numpy as np

import pyrr.matrix33 as m3
import pyrr.matrix44 as m4


from ..util import m3_rotation


def distal_to_proximal_frame(vec_in_distal_disk_frame: np.ndarray,
                             top_joint_angle: float,
                             top_orientationDF: float) -> np.ndarray:
    """
        Change distal disk's vector from distal disk's frame to the proximal disk's frame
        (Used primarily for transformation of contact reaction vectors)
    """
    return np.dot(m3_rotation((0, 0, 1.0), top_orientationDF),
                  np.dot(m3_rotation((1.0, 0, 0), top_joint_angle), vec_in_distal_disk_frame))


def eval_tendon_guide_top_force(tensionInDisk: float,
                      top_joint_angle: float,
                      top_orientationDF: float) -> np.ndarray:
    """
        Evaluate force component at any top end of tendon guide
    """
    half_joint_angle = top_joint_angle/2
    return np.array((
        tensionInDisk*sin(half_joint_angle)*sin(top_orientationDF),
        -tensionInDisk*sin(half_joint_angle)*cos(top_orientationDF),
        tensionInDisk*cos(half_joint_angle)
    ))


def eval_tendon_guide_bottom_force(tensionInDisk: float,
                         bottom_joint_angle: float) -> np.ndarray:
    """
        Evaluate force component at any bottom end of tendon guide
    """
    half_joint_angle = bottom_joint_angle/2
    return np.array((
        0,
        -tensionInDisk*sin(half_joint_angle),
        -tensionInDisk*cos(half_joint_angle)
    ))


def eval_tendon_guide_top_end_disp(length: float,
                        top_curve_radius: float,
                        tendon_dist_from_axis: float,
                        tendon_orientationDF: float,
                        top_orientationDF: float) -> np.ndarray:
    """
        Evaluate displacement from disk center to any top end of tendon guide
    """
    horizontalDispAlongCurve = tendon_dist_from_axis * \
        sin(tendon_orientationDF - top_orientationDF)
    return np.array((
        tendon_dist_from_axis*cos(tendon_orientationDF),
        tendon_dist_from_axis*sin(tendon_orientationDF),
        sqrt(top_curve_radius**2 - horizontalDispAlongCurve**2) +
        length/2 - top_curve_radius
    ))


def eval_tendon_guide_bottom_end_disp(length: float,
                               bottom_curve_radius: float,
                               tendon_dist_from_axis: float,
                               tendon_orientationDF: float) -> np.ndarray:
    """
        Evaluate displacement from disk center to any bottom end of tendon guide
    """
    horizontalDispAlongCurve = tendon_dist_from_axis*sin(tendon_orientationDF)
    return np.array((
        tendon_dist_from_axis*cos(tendon_orientationDF),
        tendon_dist_from_axis*sin(tendon_orientationDF),
        (-sqrt(bottom_curve_radius**2 - horizontalDispAlongCurve**2) +
        bottom_curve_radius - length/2) if bottom_curve_radius is not None else (-length/2)
    ))


def eval_top_contact_disp(length: float,
                       top_curve_radius: float,
                       top_joint_angle: float,
                       top_orientationDF: float):
    """
        Evaluate displacement from disk center to center of top contacting line
    """
    half_joint_angle = top_joint_angle/2
    return np.array((
        top_curve_radius*sin(half_joint_angle)*sin(top_orientationDF),
        -top_curve_radius*sin(half_joint_angle)*cos(top_orientationDF),
        top_curve_radius*cos(half_joint_angle) + length/2 - top_curve_radius
    ))


def eval_bottom_contact_disp(length: float,
                          bottom_curve_radius: float,
                          bottom_joint_angle: float):
    """
        Evaluate displacement from disk center to center of bottom contacting line
    """
    half_joint_angle = bottom_joint_angle/2
    return np.array((
        0,
        -bottom_curve_radius*sin(half_joint_angle),
        -bottom_curve_radius*cos(half_joint_angle) +
        bottom_curve_radius - length/2
    ))






