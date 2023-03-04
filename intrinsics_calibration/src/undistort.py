import cv2 as cv
import numpy as np
import argparse

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", required=True, help="path to the distorted input image.")
args = vars(ap.parse_args())

# Load the image
img = cv.imread(args["input"])

# Load the npz file
npzfile = np.load('/home/ubuntu/Python/Drone_Multi-Sensor_Calibration/calibration_values.npz')

# Extract the matrix values
intrinsic_matrix = npzfile['mtx']
distortion_coeffs = npzfile['dist']

print("mtx:")
print(intrinsic_matrix)
print("dist:")
print(distortion_coeffs)

# Load the intrinsic calibration matrix
intrinsic_matrix = np.array([[intrinsic_matrix[0][0], 0, intrinsic_matrix[0][2]],
                             [0, intrinsic_matrix[1][1], intrinsic_matrix[1][2]],
                             [0, 0, 1]])

# Load the distortion coefficients
distortion_coeffs = np.array([distortion_coeffs[0][0], distortion_coeffs[0][1], distortion_coeffs[0][2], distortion_coeffs[0][3], distortion_coeffs[0][4]])

# Undistort the image
undistorted_img = cv.undistort(img, intrinsic_matrix, distortion_coeffs)

# Display the undistorted image
cv.imshow('Distorted Image', img)
cv.waitKey(0)
cv.imshow('Undistorted Image', undistorted_img)
cv.waitKey(0)
cv.destroyAllWindows
exit()