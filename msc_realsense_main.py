from intrinsics_calibration.src import charuco_intrinsics_calibration as charuco
import ur_control.src.ur_control as urc
from realsense import realsense_depth as rd
from averages import averages as avg
import cv2 as cv
import threading
import argparse
import time
import csv
import os

if __name__ == "__main__":

    # Args for setting IP/port of AI-deck. Default settings are for when
    # AI-deck is in AP mode.
    parser = argparse.ArgumentParser(description='Hand-Eye calibration of a ur gripper to a webcam example')
    parser.add_argument("-r", default="172.31.1.200", metavar="robotip", help="Robot IP")
    parser.add_argument("-s", type=int, default='21', metavar="stations", help="Number of stations in each repetition")
    parser.add_argument("-e", type=int, default='1', metavar="repetitions", help="Number of repetition")
    parser.add_argument("-v", type=float, default='0.3', metavar="velocity", help="Velocity of the UR")
    parser.add_argument("-a", type=float, default='0.1', metavar="acceleration", help="Acceleration of the UR")
    args = parser.parse_args()

    # Define robot-related parameters
    rob_ip = args.r
    v = args.v
    a = args.a

    # Create UR control object
    ur = urc.urControl(rob_ip, v, a)
    dir_path = os.path.realpath(os.path.dirname(__file__))

    # Initialize the realsense object
    rs = rd.DepthCamera(f"{dir_path}/logs/captures")
    rsstream_start_thread = threading.Thread(target=rs.stream)
    rsstream_start_thread.start()

    # Move to home pose where the calibration object needs to be place
    ur.move_home()
    input("Place the ChAruCo board under the drone. Then press Enter.")

    ur_poses = [] # List of TCP poses lists
    all_captures = []

    repetitions = args.e # Number of repetitions of the same path
    stations = args.s # Number of stations of capture, imu, and aruco pose data collection

    for x in range(repetitions):
        ur.move_home()
        captures = []
        for i in range(stations):
            ur_pose = [] # saves the TCP pose
            # No need for IMU values when moving from home to first capture pose
            if i == 0:
                if x == 0:
                    ur.move_target('sys') # moves to a random pose
                else:
                    ur.move_repeat() # mimics movement from the original repetition  
                rob_pose = ur.read_pose() # reads the robot pose (list of 6)
                time.sleep(0.5)
                rs.save_frame(x, i)
                time.sleep(0.2)
                ur_poses.append(rob_pose) # append the robot pose
            else:
                # Move to a random position
                if x == 0:
                    ur.move_target('sys')
                else:
                    ur.move_repeat()
                time.sleep(0.5)
                rs.save_frame(x, i)
                time.sleep(0.2)
                rob_pose = ur.read_pose()
                ur_poses.append(rob_pose)
        all_captures.append(rs.get)

    rs.release()
    rsstream_start_thread.join()

    # Average the robot poses
    avg_ur_poses = avg.poses_average(ur_poses, repetitions) # returns a list of {stations} averaged ur poses (tx,ty,tz,rx,ry,rz)
    # Save the average robot poses (m, radian)
    with open(f"{dir_path}/logs/robot_poses.csv", "w", newline="") as f:
        posewriter = csv.writer(f)
        posewriter.writerows(avg_ur_poses) 
        print("robot_poses.csv is created")

    charuco_poses = [] # saves charuco poses of each repetition
    for i in range(repetitions):
        # Create ChAruCo board object, calibrate the camera intrinsics, and estimate ChAruCo poses for each repetition
        if repetitions == 1:
            charucoObj = charuco.charuco(5, 3, 0.055, 0.043, f'{dir_path}/logs/', captures)
        else:
            charucoObj = charuco.charuco(5, 3, 0.055, 0.043, f'{dir_path}/logs/', captures[i])
        camMat, distCoef = charucoObj.intrinsicsCalibration()
        charucoObj.poseEstimation(camMat, distCoef, charuco_poses) # Outputs a 3x1 translation and a 3x1 rotation (Rodrigues) of the calib object wrt the camera CS
    # Average the charuco poses
    avg_charuco_poses = avg.poses_average(charuco_poses, repetitions)
    # Save the ChAruCo poses (m, radian)
    with open(f"{dir_path}/logs/charuco_poses.csv", "w", newline="") as f:
        posewriter = csv.writer(f)
        posewriter.writerows(avg_charuco_poses)
        print("charuco_poses.csv is created")

    # Split all logs into rotation and translation np.arrays
    # Split the robot poses
    ur_tvecs, ur_rvecs = avg.split_poses(avg_ur_poses)
    # Split the charuco poses
    charuco_tvecs, charuco_rvecs = avg.split_poses(avg_charuco_poses)

    # Perform the hand-eye calibration to get X. (Camera to TCP)
    r_X, t_X = cv.calibrateHandEye(ur_rvecs, ur_tvecs, charuco_rvecs, charuco_tvecs, method=cv.CALIB_HAND_EYE_TSAI)
    print(f"translation cam2tcp, X, matrix: \n{t_X}")
    print(f"----------------------------------")
    print(f"rotation cam2tcp, X, matrix: \n{r_X}")

    # Save the X hand-eye calibration matrix
    with open(f"{dir_path}/logs/camera2tcp_calibMat.txt", "w") as f:
        f.write("Translation:\n")
        f.write(str(t_X))
        f.write("\n")
        f.write("Rotation:\n")
        f.write(str(r_X))