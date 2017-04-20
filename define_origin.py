"""Defines the world origin and axis orientation using an ArUco marker.
The origin marker's center will be the origin (0,0,0) of the world frame.
The origin marker's axis will be the axis of the world frame.
Put the marker somewhere, in the orientation you want and run the script.
Only one marker must be in the frame.
Press SPACE when you're satisfied with the origin and axis.
The marker pose is saved in origin.npz."""

import cv2
import cv2.aruco as aruco
import numpy as np

# load the camera calibration parameters
calfile = np.load('calibration.npz')
newcameramtx = calfile['newcameramtx']
mtx = calfile['mtx']
dist = calfile['dist']
roi = calfile['roi']

# use a dictionary of 25 3x3 markers
aruco_dict = aruco.Dictionary_create(25, 3)

# size of the frames
width = 640
height = 480

# create the mappings to undistort the frames
map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, None, (width, height), cv2.CV_32FC1)

# size of the origin marker
# the unit of this length will define the unit of the world frame coordinates
markerLength = 2.6  # 4.1

# start the capture (on camera channel 0)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

while True:
    # capture a frame
    ret, frame = cap.read()

    # undistort and crop the frame
    # cv2.undistort() is slow so we use a remapping
    undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    # undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    x, y, w, h = roi
    cropped = undistorted[y:y + h, x:x + w]

    # blur (optional)
    # blurred = cv2.GaussianBlur(cropped, (3, 3), 0)

    # RGB to gray
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

    # create the (default) parameters for marker detection
    parameters = aruco.DetectorParameters_create()

    # detect the origin marker
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # if the marker was detected
    if ids is not None:
        # get the pose of the marker
        # dist = None because we already give the function an undistorted image
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, None)

        # draw the markers on the undistorted and cropped image
        withMarkers = aruco.drawDetectedMarkers(cropped, corners, ids, (0, 255, 0))

        # draw the axis of the origin
        for i in range(len(ids)):
            withAxis = aruco.drawAxis(withMarkers, mtx, None, rvec[i], tvec[i], markerLength*2)

        # wait 0.1s on each frame
        # when SPACE is pressed, save origin coordinates and stop reading frames
        if cv2.waitKey(100) == 32:
            # save the origin marker pose in origin.npz file
            np.savez('origin', rvec=rvec[i], tvec=tvec[i])
            print(rvec[i])
            print(tvec[i])
            break

    # if the marker was not detected
    else:
        # the frame without marker and axis
        withAxis = cropped

    # display the frame
    cv2.imshow('frame', withAxis)

    # if ESC is pressed, stop
    if cv2.waitKey(1) == 27:
        print("Exit.")
        break

# display the frame
cv2.imshow('frame', withAxis)

# when ESC is pressed, script is done
if cv2.waitKey(0) == 27:
    print("Done.")

# release capture and close all the windows
cap.release()
cv2.destroyAllWindows()
