from intrinsics_calibration.src import charuco_intrinsics_calibration as charuco
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import crazyflie.src.asynch_imu_log as imu 
from cflib.crazyflie.log import LogConfig
import ur_control.src.ur_control as urc
from cflib.crazyflie import Crazyflie
import crazyflie.src.imu_trans2pose as t2p
import crazyflie.src.capture as cap
from cflib.utils import uri_helper
import cflib.crtp
import cv2 as cv
import threading
import argparse
import logging
import glob
import time
import csv
import os

if __name__ == "__main__":

    # Args for setting IP/port of AI-deck. Default settings are for when
    # AI-deck is in AP mode.
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="dip", help="AI-deck IP")
    parser.add_argument("-r", default="172.31.1.200", metavar="rip", help="Robot IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    parser.add_argument("-u", type=str, default='radio://0/100/2M/E7E7E7E701', metavar="uri", help="Radio-AP URI")
    parser.add_argument('--unsave', action='store_false', help="Dont save streamed images")
    args = parser.parse_args()

    # Define robot-related parameters
    rob_ip = args.r
    v = 0.30
    a = 0.1

    # Create UR control object
    ur = urc.urControl(rob_ip, v, a)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # cfRadio, cfWiFi connection, and Path variables
    deck_port = args.p
    deck_ip = args.n
    dir_path = os.path.realpath(os.path.dirname(__file__))
    logfile = f"{dir_path}/logs/imu_log.txt"
    uri_add = args.u
    uri = uri_helper.uri_from_env(default=uri_add)

    # Connect to crazyflie and initialize the client socket
    cfCam = cap.Camera(deck_ip, deck_port, f"{dir_path}/logs/captures")
    stream_start_thread = threading.Thread(target=cfCam.start_stream)
    stream_stop_thread = threading.Thread(target=cfCam.stop_stream)

    # Initialize log parameters
    logging.basicConfig(level=logging.ERROR)
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    # Move to home pose where the calibration object needs to be place
    ur.move_home()
    input("Place the ChAruCo board under the drone. Then press Enter.")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logger = imu.logging(logfile, scf, lg_stab)
        logger.start_async_log() # start IMU logging
        stream_start_thread.start() # start camera streaming

        imu_timestamps = [] # List of target IMU timestamps 

        capture_timestamps = [] # List of target capture timestamps 

        ur_poses = [] # List of TCP poses
        tcp_rvecs = [] # List of TCP rot vectors
        tcp_tvecs = [] # List of TCP trans vectors
        ur_pose_labels = ["Translation Vector", "Rotation Vector"]
        ur_poses.append(ur_pose_labels)

        stations = 10 # Number of stations of capture, imu, and aruco pose data collection
        for i in range(stations):
            # Timestamp array to save the start and end imu_timestamps in it
            imu_timestamp = []
            capture_timestamp = []
            ur_pose = []
            # No need for IMU values when moving from home to first capture pose
            if i == 0:
                ur.move_target() # move to random pose  
                robrvec, robtvec = ur.read_pose() # read the robot pose
                time.sleep(0.5)
                capture_timestamp.append(time.time()) # get the capture timestamp
                time.sleep(0.2)
                capture_timestamps.append(capture_timestamp) # append the capture timestamp
                ur_pose.append(robtvec) # create the pose pair
                ur_pose.append(robrvec)
                tcp_rvecs.append(robrvec) # append rotation part
                tcp_tvecs.append(robtvec) # append translation part
                ur_poses.append(ur_pose) # append the pose pair
            else:
                # Move to a random position
                imu_timestamp.append(time.time()) # get the first imu timestamp
                ur.move_target()
                imu_timestamp.append(time.time()) # get the second imu timestamp
                time.sleep(0.5)
                capture_timestamp.append(time.time())
                time.sleep(0.2)
                robrvec, robtvec = ur.read_pose() # results in two np.array(1x3)
                imu_timestamps.append(imu_timestamp) # append the imu pair
                capture_timestamps.append(capture_timestamp) 
                ur_pose.append(robtvec)
                ur_pose.append(robrvec)
                tcp_rvecs.append(robrvec)
                tcp_tvecs.append(robtvec)
                ur_poses.append(ur_pose)

        imu_dict_list = logger.stop_async_log() # Each element is a dictionary of the 6 imu values
        stream_stop_thread.start()
        stream_stop_thread.join()
        stream_start_thread.join()

    # Get the wanted IMU poses
    stations_imu = [] # a list for each station transformation
    for imutsp in imu_timestamps:
        imutsp_list = [] # a list for each pose pair
        for imuts in imutsp:
            print(imuts)
            timediff = 100
            dummy_imu_dict = {} # a dictionary for each pose
            for dict in imu_dict_list:
                if abs(dict["timestamp"]-imuts) < timediff:
                    timediff = abs(dict["timestamp"]-imuts)
                    dummy_imu_dict = dict
            imutsp_list.append(dummy_imu_dict)
        stations_imu.append(imutsp_list)
    # Create a pose reference and apply the imu transformations
    imu_poses = t2p.imu_trans2pose(stations_imu)

    # Save the target IMU poses (m, degree)
    with open(f"{dir_path}/logs/imu_poses.csv", "w", newline="") as f:
        imuwriter = csv.writer(f)
        imuwriter.writerows(imu_poses)

    # Save the robot poses (m, radian)
    with open(f"{dir_path}/logs/robot_poses.csv", "w", newline="") as f:
        posewriter = csv.writer(f)
        posewriter.writerows(ur_poses)

    # Get the file names of the needed captures
    all_captures = glob.glob(f'{dir_path}/logs/captures/*.jpg')
    capture_files = []
    for ts in capture_timestamps:
        timediff = 10
        for scap in all_captures:
            caphead, captail = os.path.split(scap)
            uscount = 0
            dcount = 0
            capts = ""
            capnum = ""
            for c in captail:
                if c == ".":
                    dcount += 1
                if uscount == 2 and dcount < 2:
                    capts += c 
                if uscount == 1 and dcount == 0:
                    capnum += c
                if c == "_":
                    uscount += 1
            if abs(float(capts)-ts[0]) < timediff:
                timediff = abs(float(capts)-ts[0])
                wantedcapts = capts
                wantedcapnum = capnum
        capture_files.append(f"{dir_path}/logs/captures/img_{wantedcapnum}{wantedcapts}.jpg")

    # Delete all unwanted capture files
    for cfile in all_captures:
        if cfile not in capture_files:
            os.remove(cfile)

    # Create ChAruCo board object, calibrate the camera intrinsics, and estimate ChAruCo poses
    charucoObj = charuco.charuco(5, 3, 55, 43, f"{dir_path}/logs/captures")
    camMat, distCoef = charucoObj.intrinsicsCalibration()
    charucoPoses, charuco_rvecs, charuco_tvecs = charucoObj.poseEstimation(camMat, distCoef) # Outputs a 3x1 translation and a 3x1 rotation (Rodrigues) of the calib object wrt the camera CS

    # Save the ChAruCo poses (m, radian)
    with open(f"{dir_path}/logs/charuco_poses.csv", "w", newline="") as f:
        posewriter = csv.writer(f)
        posewriter.writerows(charucoPoses)

    # Perform the hand-eye calibration to get X. (Camera to TCP)
    r_X, t_X = cv.calibrateHandEye(tcp_rvecs, tcp_tvecs, charuco_rvecs, charuco_tvecs, method=cv.CALIB_HAND_EYE_TSAI)
    print(f"translation cam2grip, X, matrix: {t_X}")
    print(f"##################################")
    print(f"rotation cam2grip, X, matrix: {r_X}")
    print(f"##################################")
    print(f"type of tcprvec: {type(tcp_rvecs[0])}")
    print(f"type of charucorvec: {type(charuco_rvecs[0])}")

    # Save the X hand-eye calibration matrix
    with open(f"{dir_path}/logs/gripper2camera_calibMat.txt", "w") as f:
        f.write("Translation:\n")
        f.write(str(t_X))
        f.write("\n")
        f.write("Rotation:\n")
        f.write(str(r_X))
