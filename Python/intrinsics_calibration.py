import numpy as np
import cv2
import glob
import cv2.aruco as aruco
import os

# Define the size of the ArUco markers (in this case, 6x6 bits and 100 pixels)
marker_size = 100

# Define the dictionary of ArUco markers to be used (in this case, the standard DICT_6X6_250 dictionary)
dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Initialize arrays to store the observed 2D points and corresponding 3D points
obj_points = []  # 3D points in real world space
img_points = []  # 2D points in image plane

# Define the grid of ArUco markers to be used for calibration
gridboard = aruco.GridBoard_create(5, 7, marker_size, 1, dictionary)

# Capture a set of calibration images of the calibration target from different viewpoints
calibration_images = glob.glob('calibration_images/*.jpg')

for image_file in calibration_images:
    # Load the image and convert to grayscale
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Detect the ArUco markers in the image
    corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    
    # If all the markers are detected, compute the 3D and 2D points
    if len(corners) == 35:
        _, rvecs, tvecs = aruco.estimatePoseBoard(corners, ids, gridboard, camera_matrix, distortion_coefficients)
        img_points.append(corners)
        obj_points.append(gridboard.objPoints)
    else:
        # Remove the image file if it does not have the expected number of markers
        os.remove(image_file)

# Perform camera calibration using the observed 2D and corresponding 3D points
retval, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# Print the intrinsic camera parameters
print('Camera matrix:\n', camera_matrix)
print('Distortion coefficients:', distortion_coefficients)
