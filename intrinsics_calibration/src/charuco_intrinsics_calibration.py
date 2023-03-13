import numpy as np
import cv2
from cv2 import aruco
import glob
import os

class charuco:
    def __init__(self, x_count, y_count, square_len, marker_len):
        # ChAruco board variables
        self.CHESSBOARD_X_COUNT = x_count
        self.CHESSBOARD_Y_COUNT = y_count
        self.squareLength = square_len
        self.markerLength = marker_len
        self.ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Create constants to be passed into OpenCV and Aruco methods
        self.CHARUCO_BOARD = aruco.CharucoBoard.create(
                self.CHESSBOARD_X_COUNT,
                self.CHESSBOARD_Y_COUNT,
                self.squareLength,
                self.markerLength,
                self.ARUCO_DICT)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = self.CHARUCO_BOARD.objPoints
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # All images used should be the same size, which if taken with the same camera shouldn't be a problem
        self.dir_path = os.path.realpath(os.path.dirname(__file__))
        self.images = glob.glob(f'{self.dir_path}/../charuco_captures/*.jpg')

    def intrinsicsCalibration(self):
        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        corners_all = [] # Corners discovered in all images processed (3d point in real world space)
        self.objpoints = [] # 2d point in image plane
        ids_all = [] # Aruco ids corresponding to corners discovered
        image_size = None # Determined at runtime
        
        # Loop through images glob'ed
        for im in self.images:
            # Open the image
            img = cv2.imread(im)
            # Grayscale the image
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Detect marker corners
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, self.ARUCO_DICT)

            # Outline the aruco markers found in our query image
            img = aruco.drawDetectedMarkers(img, marker_corners, borderColor = (0,255,0))

            # Get charuco corners and ids from detected aruco markers
            _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, self.CHARUCO_BOARD)

            # If a Charuco board was found, let's collect image/corner points
            # Requiring at least 20 squares
            if len(charuco_ids) > 20:
                # Add these corners and ids to our calibration arrays
                corners2 = cv2.cornerSubPix(gray, charuco_corners, (11,11), (-1,-1), self.criteria)
                corners_all.append(corners2)
                ids_all.append(charuco_ids)
                self.objpoints.append(self.objp)
                
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
                #cv2.imshow(im, img)
                #cv2.waitKey(200)
                #cv2.destroyWindow(im)
                #cv2.waitKey(10)
            else:
                print("Not able to detect a charuco board in image: {}".format(im))
            print("one is done")

        # Destroy any open CV windows
        cv2.destroyAllWindows()

        # Make sure at least one image was found
        if len(self.images) < 1:
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
        calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(corners_all, ids_all, self.CHARUCO_BOARD, image_size, None, None)
            
        # Print matrix and distortion coefficient to the console
        print(cameraMatrix)
        print(distCoeffs)
            
        # Save values to be used where matrix+dist is required, for instance for posture estimation
        # I save files in a pickle file, but you can use yaml or whatever works for you
        # Save the camera matrix and distortion coefficients to a file
        np.savez(f'{self.dir_path}/../charuco_calibration_values.npz', mtx=cameraMatrix, dist=distCoeffs)
            
        # Print to console our success
        print(f'Calibration Successful. Calibration values saved in: {self.dir_path}/../charuco_calibration_values.npz')

        return cameraMatrix, distCoeffs

    def poseEstimation(self, cameraMatrix, distCoeffs):
        
        for im in self.images:
            # Open the image
            img = cv2.imread(im)
            # Detect marker corners
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(img, self.ARUCO_DICT)
            if len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(img, marker_corners, marker_ids)
                _, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(marker_corners, marker_ids, img, self.CHARUCO_BOARD)
                
                if len(charuco_ids) > 0:
                    color = (255, 0, 0)
                    cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids, color)
                    rvec = np.empty([3,1])
                    tvec = np.empty([3,1])
                    retval,rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, self.CHARUCO_BOARD, cameraMatrix, distCoeffs, None, None)

                    if retval == True:
                        cv2.drawFrameAxes(img, cameraMatrix, distCoeffs, rvec, tvec, 0.1)

            print(f"retval: {retval}")
            print(f"rvec: {rvec}")
            print(f"tvec: {tvec}")
            cv2.imshow("out", img)
            cv2.waitKey(10)
        return tvec, rvec

if __name__ == "__main__":
    calib_obj = charuco(7, 5, 0.04, 0.031)
    camMatrix, distCoef = calib_obj.intrinsicsCalibration()
    tvec, rvec = calib_obj.poseEstimation(camMatrix, distCoef)

