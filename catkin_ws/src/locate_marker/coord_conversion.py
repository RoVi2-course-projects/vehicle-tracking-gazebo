#!/usr/bin/env python2
# Third party libraries
from geometry_msgs.msg import Point
import numpy as np


def get_normalized_gsd():
    #TODO: Complete function
    gsd = 10
    return gsd


def gsd_correction(gsd_0, height, alpha, theta, pix_distance):
    """
    Correct the GSD when the camera is not perpendicular to the ground.

    parameters
    ----------
    gsd_0 : GSD at the pixel which is at the short side of the camera to
        the ground.
    height : Drone height to the ground
    alpha : Field-of-View angle of the camera. [radians]
    theta : camera angle (drone angle + fixed camera angle). [radians]
    pix_distance : Distance in pixels from the closest side to the
        ground.
    """
    # d_0 is the distance of the shortest side of the camera to the ground.
    d_0 = height / np.cos(90-alpha-theta)
    # d_1 is the distance of the petotal rpendicular of d_0 to the ground at the
    # position of the actual pixel.
    d_1 = (gsd_0 * pix_distance * np.sin(90-alpha-theta)) / np.sin(theta)
    corrected_gsd = gsd_0 * d_1 / d_0
    # Get the corrected value for the coordinate.
    corrected_coord = (height*np.tan(theta)) - (pix_distance*corrected_gsd)
    return corrected_coord

def get_corrected_coords(point):
    coord_x = point.x
    coord_y = point.y
    coord_z = point.z
    # TODO: Convert, if necessary, the coordinates origin from centre to corner.
    alpha = [0, 0, 0]
    theta = [0, 0, 0]
    gsd_0 = [0, 0, 0]
    corrected_x = gsd_correction(gsd_0[0], height, alpha[0], theta[0], coord_x)
    corrected_y = gsd_correction(gsd_0[1], height, alpha[1], theta[1], coord_y)
    corrected_z = gsd_correction(gsd_0[2], height, alpha[2], theta[2], coord_z)
    corrected_point = Point(corrected_x, corrected_y, corrected_z)
    return corrected_point