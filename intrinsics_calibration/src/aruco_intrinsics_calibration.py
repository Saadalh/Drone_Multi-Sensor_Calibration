import numpy as np
import cv2
import glob
from cv2 import aruco
import os

# Define the size of the ArUco markers (in this case, 6x6 bits and 100 pixels)
marker_size = 100

# Define the number of rows and columns in the gridboard
gridboard_rows = 5
gridboard_cols = 7

# Define the dictionary of ArUco markers to be used (in this case, the standard DICT_6X6_250 dictionary)
dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters()

# Initialize arrays to store the observed 2D points and corresponding 3D points
obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane
image_size = None # Determined in runtime

# Define the grid of ArUco markers to be used for calibration
gridboard = aruco.GridBoard((gridboard_rows, gridboard_cols), marker_size, 1, dictionary)

# Capture a set of calibration images of the calibration target from different viewpoints
calibration_images = glob.glob("/home/ubuntu/Python/Drone_Multi-Sensor_Calibration/intrinsics_calibration/aruco_captures/*.jpg")

for image_file in calibration_images:
    # Load the image and convert to grayscale
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect the ArUco markers in the image
    detector = aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(gray)
    
    # If all the markers are detected, compute the 3D and 2D points
    if len(corners) == gridboard_rows * gridboard_cols:
        objp = gridboard.getObjPoints()
        img_points.append(corners)
        obj_points.append(objp)

        # Draw the detected markers on the calibration image and display it
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        img = cv2.aruco.drawDetectedMarkers(color, corners, borderColor = (0,255,0))

        # If our image size is unknown, set it now
        if not image_size:
            image_size = gray.shape[::-1]

        cv2.imshow('calibration image', img)
        cv2.waitKey(10)

# Perform camera calibration using the observed 2D and corresponding 3D points
print(img_points[0])
print("--------------------")
print(corners[0])
retval, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(objp, corners, image_size, None, None)

# Print the intrinsic camera parameters
print('Camera matrix:\n', camera_matrix)
print('Distortion coefficients:', distortion_coefficients)

# Save the camera matrix and distortion coefficients to a file
np.savez('../aruco_calibration_values.npz', mtx=camera_matrix, dist=distortion_coefficients)