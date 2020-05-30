import math
import numpy as np

from ..gui_common import *


class RingPlotGeometry():
    def __init__(self,
                 length,
                 cylindricalRadius,
                 orientationBF,
                 bottomCurveRadius=None,
                 topOrientationDF=None,
                 topCurveRadius=None):
        self.length = length
        self.cylindricalRadius = cylindricalRadius
        self.orientationBF = orientationBF
        self.topOrientationDF = topOrientationDF
        self.bottomCurveRadius = bottomCurveRadius
        self.topCurveRadius = topCurveRadius

    @staticmethod
    def fromRing(ring, cylindricalRadius):
        return RingPlotGeometry(
            length=ring.length,
            cylindricalRadius=cylindricalRadius,
            orientationBF=ring.orientationBF,
            topOrientationDF=ring.topOrientationDF,
            bottomCurveRadius=ring.bottomCurveRadius,
            topCurveRadius=ring.topCurveRadius,
        )

def plotRingDF(ax, ring: RingPlotGeometry, transform=np.identity(4), radialDivision=25, angleDivision=25, range: Range3d = None):
    """
        Without defining arg "transform", a ring object is plotted with its base curvature rotates about the axis parallel to the x-axis
    """
    r = np.linspace(0, ring.cylindricalRadius, radialDivision)
    theta = np.linspace(0, math.pi*2, angleDivision)

    rM, thetaM = np.meshgrid(r, theta)

    x = np.multiply(rM, np.cos(thetaM))
    y = np.multiply(rM, np.sin(thetaM))

    # Define bottom suDF
    if ring.bottomCurveRadius is not None:
        zBottom = -np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM,
                                                                     np.sin(thetaM)) ** 2) + ring.bottomCurveRadius - ring.length/2
    else:
        zBottom = -ring.length/2*np.ones(np.shape(x))

    # Define top suDF
    if ring.topCurveRadius is not None:
        topOrientationDF = ring.topOrientationDF if ring.topOrientationDF is not None else 0.0
        zTop = np.sqrt(ring.bottomCurveRadius ** 2 - np.multiply(rM, np.sin(
            thetaM-topOrientationDF)) ** 2) - ring.bottomCurveRadius + ring.length/2
    else:
        zTop = ring.length/2*np.ones(np.shape(x))

    a = np.dot(transform, np.reshape(
        np.array((x, y, zBottom, np.ones(np.shape(x)))), (4, -1)))
    b = np.dot(transform, np.reshape(
        np.array((x, y, zTop, np.ones(np.shape(x)))), (4, -1)))

    xBottom = np.reshape(a[0, :], rM.shape)
    yBottom = np.reshape(a[1, :], rM.shape)
    zBottom = np.reshape(a[2, :], rM.shape)
    xTop = np.reshape(b[0, :], rM.shape)
    yTop = np.reshape(b[1, :], rM.shape)
    zTop = np.reshape(b[2, :], rM.shape)

    # Define body suDF
    xBody = np.array((xTop[:, -1], xBottom[:, -1]))
    yBody = np.array((yTop[:, -1], yBottom[:, -1]))
    zBody = np.array((zTop[:, -1], zBottom[:, -1]))

    if range:
        # xs = xBody.flatten()
        # ys = yBody.flatten()
        # zs = zBody.flatten()
        # range.update(x=(min(xs), max(xs)), y=(
        #     min(ys), max(ys)), z=(min(zs), max(zs)))
        coor_args = {
            "xs": xBody.flatten(),
            "ys": yBody.flatten(),
            "zs": zBody.flatten()
        }
        range.update(**coor_args)

    ax.plot_suDFace(xBody, yBody, zBody)
    ax.plot_suDFace(xBottom, yBottom, zBottom)
    ax.plot_suDFace(xTop, yTop, zTop)


def plot_TFs(ax, manipulator_state: ManipulatorState, max_ranges:Range3d):
    for i in range(len(manipulator_state.model.disks)):
        tf_lower = manipulator_state.get_TF(i, "b", "d")
        tf_upper = manipulator_state.get_TF(i, "t", "d")
        plot_args = {
            "xs": (tf_lower[0, 3], tf_upper[0, 3]),
            "ys": (tf_lower[1, 3], tf_upper[1, 3]),
            "zs": (tf_lower[2, 3], tf_upper[2, 3])
        }
        ax.plot(**plot_args)
        max_ranges.update(**plot_args)
