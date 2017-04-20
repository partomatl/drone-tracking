"""Gets the coordinates of a marker wrt the origin marker, in the world frame.
Takes the pose (R_origin and tvec_origin) of the origin and
the pose (rvec_marker and tvec_marker) of the marker as parameters
and returns the (x,y,z) world coordinates, as well as the orientation angle in degrees.

R_origin (3x3 matrix) is cv2.Rodrigues(rvec_origin).
tvec_origin (3x1 vector) is the flattened tvec_origin.
"""

import numpy as np
import cv2


def get_coordinates(R_origin, tvec_origin, rvec_marker, tvec_marker):

    # get the rotation matrix from camera frame to world frame
    RMarkerCam, jac = cv2.Rodrigues(rvec_marker)
    RMarkerWorld = np.dot(np.transpose(R_origin), RMarkerCam)

    # get the orientation of the drone in the world frame
    angleMarker = np.arctan2(RMarkerWorld[1][0], RMarkerWorld[0][0])

    # since the marker is rotated 45 degrees
    angleMarker += np.pi/4

    # in degrees
    angleMarker *= 180/np.pi

    # round the angle to the nearest integer, no need for precision here
    # angleMarker = round(angleMarker)

    # get the position of the marker wrt origin marker
    posMarkerCam = tvec_marker
    posMarkerCam = np.ndarray.flatten(posMarkerCam)

    # get the position of the drone in the world frame wrt to the origin
    # inverse of rotation matrix is its transpose
    # np.linalg.inv(R_origin) = R_origin.T
    posMarkerWorld = np.dot(R_origin.T, posMarkerCam - tvec_origin)

    # round the positions to the nearest integers, no need for precision here
    # xMarker = round(posMarkerWorld[0])
    # yMarker = round(posMarkerWorld[1])
    # zMarker = round(posMarkerWorld[2])

    xMarker = posMarkerWorld[0]
    yMarker = posMarkerWorld[1]
    zMarker = posMarkerWorld[2]

    # z cannot be positive, assume it is 0 if negative
    if zMarker < 0:
        zMarker = 0

    return xMarker, yMarker, zMarker, angleMarker
