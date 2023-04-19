import numpy as np
import cv2 as cv
import glob
import ur_control.src.ur_control as urc
from realsense import realsense_depth as rd
import os
import threading
import time

# Create UR control object
ur = urc.urControl("172.31.1.200", 0.3, 0.1, 'rs')
dir_path = os.path.realpath(os.path.dirname(__file__))

# Initialize the realsense object
rs = rd.DepthCamera(f"{dir_path}/logs/captures/rs")
rsstream_start_thread = threading.Thread(target=rs.stream)
rsstream_start_thread.start()

# Move to home pose where the calibration object needs to be place
ur.move_home()
input("Place the ChAruCo board under the drone. Then press Enter.")

all_captures = []

stations = 21

for i in range(stations):
    ur.move_target('sys')
    time.sleep(0.5)
    rs.save_frame(0,i+1)
    time.sleep(0.2)
all_captures.append(rs.get_caplist())
rs.release(rsstream_start_thread)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

for fname in all_captures[0]:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (18,28), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (18,28), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
cv.destroyAllWindows()