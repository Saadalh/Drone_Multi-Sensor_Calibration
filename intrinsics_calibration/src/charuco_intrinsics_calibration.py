import numpy as np
import cv2
from cv2 import aruco
import glob
import os


# ChAruco board variables
CHARUCOBOARD_ROWCOUNT = 7
CHARUCOBOARD_COLCOUNT = 5 
squareLength = 0.04
squareLength_mm = 40
markerLength = 0.031
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
ARUCO_PARAM =  cv2.aruco.DetectorParameters()
chessboardSize = (6,4)
frameSize = (640,480)

# Create constants to be passed into OpenCV and Aruco methods
CHARUCO_BOARD = aruco.CharucoBoard(
        (CHARUCOBOARD_COLCOUNT,CHARUCOBOARD_ROWCOUNT),
        squareLength,
        markerLength,
        ARUCO_DICT)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * squareLength_mm # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

objp = objp * squareLength_mm

# Create the arrays and variables we'll use to store info like corners and IDs from images processed
imgpoints = [] # Corners discovered in all images processed (3d point in real world space)
objpoints = [] # 2d point in image plane
ids_all = [] # Aruco ids corresponding to corners discovered
image_size = None # Determined at runtime
dir_path = os.path.realpath(os.path.dirname(__file__))


# This requires a set of images or a video taken with the camera you want to calibrate
# I'm using a set of images taken with the camera with the naming convention:
# 'camera-pic-of-charucoboard-<NUMBER>.jpg'
# All images used should be the same size, which if taken with the same camera shouldn't be a problem
images = glob.glob(f'{dir_path}/../charuco_captures/*.jpg')

# Loop through images glob'ed
for iname in images:
    # Open the image
    img = cv2.imread(iname)
    # Grayscale the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find aruco markers in the query image
    detector = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAM)
    marker_corners, marker_ids, _ = detector.detectMarkers(gray)

    # Outline the aruco markers found in our query image
    img = cv2.aruco.drawDetectedMarkers(img, marker_corners, borderColor = (0,255,0))

    # Define ChArUco board object
    CHARUCO_DETECTOR = cv2.aruco.CharucoDetector(CHARUCO_BOARD)

    # Get charuco corners and ids from detected aruco markers
    charuco_corners, charuco_ids, marker_corners, marker_ids = CHARUCO_DETECTOR.detectBoard(image=gray)

    # If a Charuco board was found, let's collect image/corner points
    # Requiring at least 20 squares
    if len(charuco_ids) > 20:
        # Add these corners and ids to our calibration arrays
        imgpoints.append(charuco_corners)
        ids_all.append(charuco_ids)
        objpoints.append(objp)
        
        # Draw the Charuco board we've detected to show our calibrator the board was properly detected
        img = aruco.drawDetectedCornersCharuco(
                image=img,
                charucoCorners=charuco_corners,
                charucoIds=charuco_ids)
       
        # If our image size is unknown, set it now
        if not image_size:
            image_size = gray.shape[::-1]
    
        # Reproportion the image, maxing width or height at 1000
        proportion = max(img.shape) / 1000.0
        img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
        # Pause to display each image, waiting for key press
        cv2.imshow(iname, img)
        cv2.waitKey(10)
    else:
        print("Not able to detect a charuco board in image: {}".format(iname))
    print("one is done")

# Destroy any open CV windows
cv2.destroyAllWindows()

# Make sure at least one image was found
if len(images) < 1:
    # Calibration failed because there were no images, warn the user
    print("Calibration was unsuccessful. No images of charucoboards were found. Add images of charucoboards and use or alter the naming conventions used in this file.")
    # Exit for failure
    exit()

# Make sure we were able to calibrate on at least one charucoboard by checking
# if we ever determined the image size
if not image_size:
    # Calibration failed because we didn't see any charucoboards of the PatternSize used
    print("Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
    # Exit for failure
    exit()

# Now that we've seen all of our images, perform the camera calibration
# based on the set of points we've discovered
calibration, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)
#calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
#       charucoCorners=imgpoints,
#        charucoIds=ids_all,
#        board=CHARUCO_BOARD,
#        imageSize=image_size,
#        cameraMatrix=None,
#        distCoeffs=None)
    
# Print matrix and distortion coefficient to the console
print(cameraMatrix)
print(distCoeffs)
    
# Save values to be used where matrix+dist is required, for instance for posture estimation
# I save files in a pickle file, but you can use yaml or whatever works for you
# Save the camera matrix and distortion coefficients to a file
np.savez(f'{dir_path}/../charuco_calibration_values.npz', mtx=cameraMatrix, dist=distCoeffs)
    
# Print to console our success
print(f'Calibration Successful. Calibration calues saved in: {dir_path}/../charuco_calibration_values.npz')