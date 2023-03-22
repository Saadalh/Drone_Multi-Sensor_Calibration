import argparse, os
import numpy as np
import cv2 as cv

dir_path = os.path.realpath(os.path.dirname(__file__))

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", required=True, help="path to the distorted input image.")
args = vars(ap.parse_args())

# Load the npz file
npzfile = np.load(f'{dir_path}/../../logs/charuco_calibration_values.npz')

# Extract the matrix values
intrinsic_matrix = npzfile['mtx']
distortion_coeffs = npzfile['dist']

# Load the image
img = cv.imread(args["input"])
h,  w = img.shape[:2]
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, (w,h), 1, (w,h))

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

# crop the image
x, y, w, h = roi
undistorted_img = undistorted_img[y:y+h, x:x+w]

# Display the undistorted image
cv.imshow('Distorted Image', img)
cv.waitKey(0)
cv.imshow('Undistorted Image', undistorted_img)
cv.waitKey(0)
cv.destroyAllWindows
exit()