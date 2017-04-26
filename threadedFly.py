"""Flies the drone.
Uses the serial port to send PPM commands to the drone via an Arduino.
Uses a different thread to grab frames from the webcam.
Implements a PID controller."""

from FPS import *
from WebcamVideoStream import *
import cv2
import cv2.aruco as aruco
import numpy as np
from get_coordinates import *
import serial
import time
import scipy.io
from datetime import datetime
from scipy.signal import butter, lfilter

# load the camera calibration parameters
calfile = np.load('calibration.npz')
newcameramtx = calfile['newcameramtx']
mtx = calfile['mtx']
dist = calfile['dist']
roi = calfile['roi']

# load origin marker pose
origin = np.load('origin.npz')
rvec_origin = origin['rvec']
tvec_origin = origin['tvec']

# get the 3x3 rotation matrix (R_origin) and the 3x1 translation vector (tvec_origin)
R_origin, jac = cv2.Rodrigues(rvec_origin)
tvec_origin = np.ndarray.flatten(tvec_origin)

# use a dictionary of 25 3x3 markers
aruco_dict = aruco.Dictionary_create(25, 3)

# size of the frames
width = 640
height = 480

# create the mappings to undistort the frames
map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (width, height), cv2.CV_32FC1)

# size of the drone marker
# same unit as the unit of the world frame coordinates
markerLength = 2.6

# create the (default) parameters for marker detection
parameters = aruco.DetectorParameters_create()

# id of the drone marker
idDroneMarker = 5

sequence = "hover"

# initialize drone variables
xDrone, yDrone, zDrone, angleDrone = 0, 0, 0, 0

# define a clamping function
def clamp(n, minimum, maximum):
    return max(min(maximum, n), minimum)

# define a Butterworth filter to filter the measurements
# call y = lfilter(b, a, data) to filter the data signal
order = 2  # second order filter
fs = 30  # sampling frequency is around 30 Hz
nyq = 0.5 * fs
lowcut = 2  # cutoff frequency at 2 Hz
low = lowcut / nyq
b, a = butter(order, low, btype='low')

if sequence == "hover":
    xTarget = [0]
    yTarget = [0]
    zTarget = [40]
    angleTarget = [0]
elif sequence == "trajectory":
    xTarget = []
    yTarget = []
    zTarget = []
    angleTarget = []

    # define a trajectory
    for t in range(1, 91):
        xTarget.append(0)
        yTarget.append(0)
        zTarget.append(t/3)
        angleTarget.append(0)
    for t in range(1, 31):
        xTarget.append(0)
        yTarget.append(0)
        zTarget.append(30)
        angleTarget.append(-t*3)
    for t in range(1, 91):
        xTarget.append(0)
        yTarget.append(t/3)
        zTarget.append(30)
        angleTarget.append(-90)
    for t in range(90, -1, -1):
        xTarget.append(0)
        yTarget.append(t/3)
        zTarget.append(30)
        angleTarget.append(-90)
    for t in range(-30, 31, 1):
        xTarget.append(0)
        yTarget.append(0)
        zTarget.append(30)
        angleTarget.append(t*3)
    for t in range(0, -91, -1):
        xTarget.append(0)
        yTarget.append(t/3)
        zTarget.append(30)
        angleTarget.append(90)
    for t in range(-90, 1, 1):
        xTarget.append(0)
        yTarget.append(t/3)
        zTarget.append(30)
        angleTarget.append(90)
    for t in range(-30, 1, 1):
        xTarget.append(0)
        yTarget.append(0)
        zTarget.append(30)
        angleTarget.append(t*3)
    for t in range(90, -1, -1):
        xTarget.append(0)
        yTarget.append(0)
        zTarget.append(t/3)
        angleTarget.append(0)

ii = 0

# record everything
timeRecord = []
xRecord = []
xFilteredRecord = []
yRecord = []
yFilteredRecord = []
zRecord = []
zFilteredRecord = []
angleRecord = []
angleFilteredRecord = []
xErrorRecord = []
yErrorRecord = []
zErrorRecord = []
angleErrorRecord = []
aileronRecord = []
elevatorRecord = []
throttleRecord = []
rudderRecord = []

# initialize PID controller
xError, yError, zError, angleError = 0, 0, 0, 0
xErrorI, yErrorI, zErrorI, angleErrorI = 0, 0, 0, 0
xErrorD, yErrorD, zErrorD, angleErrorD = 0, 0, 0, 0
xError_old, yError_old, zError_old, angleError_old = 0, 0, 0, 0

# define PID gains

# first set of gains (office)
# KPx, KPy, KPz, KPangle = 10, 10, 9, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.15, 0
# KDx, KDy, KDz, KDangle = 160, 160, 115, 10

# # second set with filtering of measurements (30 fps)
KPx, KPy, KPz, KPangle = 3, 3, 4, -3
KIx, KIy, KIz, KIangle = 0, 0, 0.1, 0
KDx, KDy, KDz, KDangle = 120, 120, 80, 10

# third set with video recording (23 fps)
# KPx, KPy, KPz, KPangle = 3, 3, 4, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.1*30/23, 0
# KDx, KDy, KDz, KDangle = 120*23/30, 120*23/30, 80*23/30, 10*23/30

# third set for trajectory
# KPx, KPy, KPz, KPangle = 6, 6, 8, -3
# KIx, KIy, KIz, KIangle = 0, 0, 0.2, 0
# KDx, KDy, KDz, KDangle = 110, 110, 100, 10

# define middle PPM values that make the drone hover
throttle_middle = 1600  # up (+) and down (-)
aileron_middle = 1500  # left (-) and right (+)
elevator_middle = 1500  # forward (+) and backward (-)
rudder_middle = 1500  # yaw left (-) and yaw right (+)

# define engines off PPM value for throttle
throttle_off = 1000

# count frames without detecting the drone
framesWithoutDrone = 0

# count loops
loopsCount = 0

# specify the USB port for the Arduino
usb_port = '/dev/cu.usbmodem1421'
# usb_port = '/dev/cu.usbmodem1421'

# open serial connection with Arduino
# baudrate of 115200
arduino = serial.Serial(usb_port, 115200, timeout=.01)

# wait a bit for the connection to settle
time.sleep(2)

# tell the drone to calibrate the gyroscope
# left stick to bottom left
command = "%i,%i,%i,%i" % (throttle_off, 1000, 1000, rudder_middle)
command = command + "\n"
print("Starting calibration.")
arduino.write(command.encode())
# after 1.5 second, left stick goes back to neutral position
time.sleep(1.5)
command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
command = command + "\n"
arduino.write(command.encode())

# start the capture (on camera channel 0) thread
cap = WebcamVideoStream(src=0).start()

# wait one second for everything to settle before reading first frame
time.sleep(1)

# create a video of the tracking
timestamp = "{:%Y_%m_%d_%H_%M}".format(datetime.now())
fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
out = cv2.VideoWriter('recordings/video' + timestamp + '.mov', fourcc, 30, (612, 425), True)

# start FPS counter
fps = FPS()
fps.start()

# record time
timerStart = datetime.now()

while True:
    # capture a frame
    frame = cap.read()

    # undistort and crop the frame
    # cv2.undistort() is slow so we use a remapping
    # undistorted = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    undistorted = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
    x, y, w, h = roi
    cropped = undistorted[y:y + h, x:x + w]

    # blur (optional)
    # blurred = cv2.GaussianBlur(cropped, (3, 3), 0)

    # RGB to gray
    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

    # drone detected or not
    droneDetected = False

    # detect the ArUco markers in the frame
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # if markers are detected
    if ids is not None:
        # go through the ids
        for i in range(len(ids)):
            # if the drone marker is detected, get its world coordinates and orientation
            if ids[i] == idDroneMarker:

                droneDetected = True

                framesWithoutDrone = 0

                # estimate the drone marker pose wrt to the camera
                # dist = None because we already give the function an undistorted image
                rvec, tvec = aruco.estimatePoseSingleMarkers([corners[i]], markerLength, mtx, None)

                # draw the drone marker
                withMarkers = aruco.drawDetectedMarkers(cropped, [corners[i]], ids[i], (0, 255, 0))
                # draw its axis
                withMarkers = aruco.drawAxis(withMarkers, mtx, None, rvec, tvec, markerLength*2)

                # get the coordinates and orientation of the drone
                xDrone, yDrone, zDrone, angleDrone = get_coordinates(R_origin, tvec_origin, rvec, tvec)

                # the drone was detected, no need to go further
                break

        # if markers are detected, but the drone was not detected among the markers
        if not droneDetected:
            framesWithoutDrone += 1
            # draw the undistorted and cropped frame, without marker and axis
            # assume same coordinates and orientation as in last frame
            withMarkers = cropped
            # same position and orientation

    # if no markers are detected
    else:
        framesWithoutDrone += 1
        # draw the undistorted and cropped frame, without marker and axis
        # assume same coordinates and orientation as in last frame
        withMarkers = cropped

    # print coordinates and orientation
    print("[DRONE] X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xDrone, yDrone, zDrone, angleDrone))

    # display the frame
    # cv2.imshow('frame', withMarkers)

    # write in the video
    out.write(withMarkers)

    # wait 1 ms for a key to be pressed
    key = cv2.waitKey(20)

    # abort if we lost the drone for about 1.5 secs
    if framesWithoutDrone >= 45:
        k = 0
        print("Lost the drone! Abort!")
        while (k < 10):
            command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
            command = command + "\n"
            arduino.write(command.encode())
            time.sleep(0.1)
            k += 1
        break

    # record the position and orientation
    xRecord.append(xDrone)
    yRecord.append(yDrone)
    zRecord.append(zDrone)
    angleRecord.append(angleDrone)

    # filter x
    xFiltered = lfilter(b, a, xRecord)
    xDroneFiltered = xFiltered[-1]
    xFilteredRecord.append(xDroneFiltered)

    # filter y
    yFiltered = lfilter(b, a, yRecord)
    yDroneFiltered = yFiltered[-1]
    yFilteredRecord.append(yDroneFiltered)

    # filter z
    zFiltered = lfilter(b, a, zRecord)
    zDroneFiltered = zFiltered[-1]
    zFilteredRecord.append(zDroneFiltered)

    # filter angle
    angleFiltered = lfilter(b, a, angleRecord)
    angleDroneFiltered = angleFiltered[-1]
    angleFilteredRecord.append(angleDroneFiltered)

    # implement a PID controller

    # store previous errors in position (cm) and angle (degrees) (to compute variation of error)
    xError_old = xError
    yError_old = yError
    zError_old = zError
    angleError_old = angleError

    # compute errors in position (cm) and angle (degrees) (P)
    # compute errors wrt filtered measurements
    if ii == len(xTarget):
        ii = 0

    xError = xTarget[ii]-xDroneFiltered
    yError = yTarget[ii]-yDroneFiltered
    zError = zTarget[ii]-zDroneFiltered
    angleError = angleTarget[ii]-angleDroneFiltered

    ii += 1

    # compute integral (sum) of errors (I)
    # should design an anti windup for z
    xErrorI += xError
    yErrorI += yError
    zErrorI += zError
    angleErrorI += angleError

    # compute derivative (variation) of errors (D)
    xErrorD = xError-xError_old
    yErrorD = yError-yError_old
    zErrorD = zError-zError_old
    angleErrorD = angleError-angleError_old

    # compute commands
    xCommand = KPx*xError + KIx*xErrorI + KDx*xErrorD
    yCommand = KPy*yError + KIy*yErrorI + KDy*yErrorD
    zCommand = KPz*zError + KIz*zErrorI + KDz*zErrorD
    angleCommand = KPangle*angleError + KIangle*angleErrorI + KDangle*angleErrorD

    # print the X, Y, Z, angle commands
    print("[COMMANDS]: X={:.1f} Y={:.1f} Z={:.1f} angle={:.1f}".format(xCommand, yCommand, zCommand, angleCommand))

    # throttle command is zCommand
    # commands are relative to the middle PPM values
    throttleCommand = throttle_middle + zCommand

    # angleDrone to radians for projection
    angleDroneRad = angleDrone * np.pi/180

    # project xCommand and yCommand on the axis of the drone
    # commands are relative to the middle PPM values
    elevatorCommand = elevator_middle + -np.sin(angleDroneRad)*xCommand + np.cos(angleDroneRad)*yCommand
    aileronCommand = aileron_middle + np.cos(angleDroneRad)*xCommand + np.sin(angleDroneRad)*yCommand

    # rudder command is angleCommand
    # commands are relative to the middle PPM values
    rudderCommand = rudder_middle + angleCommand

    # round and clamp the commands to [1000, 2000] us (limits for PPM values)
    throttleCommand = round(clamp(throttleCommand, 1000, 2000))
    aileronCommand = round(clamp(aileronCommand, 1000, 2000))
    elevatorCommand = round(clamp(elevatorCommand, 1000, 2000))
    rudderCommand = round(clamp(rudderCommand, 1000, 2000))

    # create the command to send to Arduino
    command = "%i,%i,%i,%i" % (throttleCommand, aileronCommand, elevatorCommand, rudderCommand)

    # print the projected commands
    print("[COMMANDS]: T={:.0f} A={:.0f} E={:.0f} R={:.0f}".format(throttleCommand, aileronCommand, elevatorCommand, rudderCommand))

    # send to Arduino via serial port
    command = command + "\n"
    arduino.write(command.encode())

    # record everything
    timerStop = datetime.now() - timerStart
    timeRecord.append(timerStop.total_seconds())

    xErrorRecord.append(xError)
    yErrorRecord.append(yError)
    zErrorRecord.append(zError)
    angleErrorRecord.append(angleError)

    elevatorRecord.append(elevatorCommand)
    aileronRecord.append(aileronCommand)
    throttleRecord.append(throttleCommand)
    rudderRecord.append(rudderCommand)

    # if ESC is pressed, stop the program
    if key == 27:
        print("Exit!")
        break

    loopsCount += 1

    # update the FPS counter
    fps.update()

fps.stop()
print("[INFO] Elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] Approximate FPS: {:.2f}".format(fps.fps()))

time.sleep(0.5)
# send a neutral command to the drone (throttle off)
i = 0
while(i < 10):
    command = "%i,%i,%i,%i" % (throttle_off, aileron_middle, elevator_middle, rudder_middle)
    command = command + "\n"
    arduino.write(command.encode())
    time.sleep(0.1)
    i += 1

# release capture and close all the windows
cap.stop()

# release video
out.release()
cv2.destroyAllWindows()

# close the connection and reopen it
arduino.close()
arduino = serial.Serial(usb_port, 115200, timeout=.01)
arduino.close()

# save the log in Matlab format
scipy.io.savemat('recordings/recording' + timestamp + '.mat', mdict={'time': timeRecord, 'x': xRecord, 'xFiltered': xFilteredRecord, 'y': yRecord, 'yFiltered': yFilteredRecord, 'z': zRecord, 'zFiltered': zFilteredRecord, 'angle': angleRecord, 'angleFiltered': angleFilteredRecord, 'xError': xErrorRecord, 'yError': yErrorRecord, 'zError': zErrorRecord, 'angleError': angleErrorRecord, 'aileron': aileronRecord, 'elevator': elevatorRecord, 'throttle': throttleRecord, 'rudder': rudderRecord})
