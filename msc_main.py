from intrinsics_calibration.src import charuco_intrinsics_calibration as charuco
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import crazyflie.src.asynch_imu_log as imu 
from cflib.crazyflie.log import LogConfig
import ur_control.src.ur_control as urc
from cflib.crazyflie import Crazyflie
import crazyflie.src.capture as cap
from cflib.utils import uri_helper
import cflib.crtp
import threading
import argparse
import logging
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

    # Radio, WiFi connection, and Path variables
    deck_port = args.p
    deck_ip = args.n
    dir_path = os.path.realpath(os.path.dirname(__file__))
    logfile = f"{dir_path}/logs/imu_log.txt"
    uri_add = args.u
    uri = uri_helper.uri_from_env(default=uri_add)

    # Connect to crazyflie and initialize the client socket
    cfCam = cap.Camera(deck_ip, deck_port)
    stream_start_thread = threading.Thread(target=cfCam.start_stream, args=( 0, f"{dir_path}/logs/captures"))
    stream_stop_thread = threading.Thread(target=cfCam.stop_stream)

    # Initialize log parameters
    logging.basicConfig(level=logging.ERROR)
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')

    # Move to home pose where the calibration object needs to be place
    ur.move_home()
    input("Place the ChAruCo board under the drone. Then press Enter.")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logger = imu.logging(logfile, scf, lg_stab)
        logger.start_async_log()
        stream_start_thread.start()

        stations = 25 # Number of stations of capture, imu, and aruco pose data collection
        imu_timestamps = [] # List of target IMU timestamps 
        capture_timestamps = [] # List of target capture timestamps 
        ur_poses = [] # List of target robot poses

        for i in range(0, stations+1):
            # No need for IMU values when moving from home to first capture pose
            if i == 0:
                ur.move_random()
                ur_poses.append(ur.read_pose())
                capture_timestamps.append(time.time())
                #cfCam.capture()
                time.sleep(1)
            # Timestamp array to save the start and end imu_timestamps in it
            imu_timestamp = []
            # Move to a random position
            imu_timestamp.append(time.time())
            ur.move_random()
            imu_timestamp.append(time.time())
            capture_timestamps.append(time.time())
            #cfCam.capture()
            time.sleep(1)
            ur_poses.append(ur.read_pose())
            imu_timestamps.append(imu_timestamp)

        logger.stop_async_log()
        stream_stop_thread.start()
        stream_stop_thread.join()
        stream_start_thread.join()

        # Save the target imu timestamps
        with open(f"{dir_path}/logs/imu_timestamps.csv", "w", newline="") as f:
            imuwriter = csv.writer(f)
            imuwriter.writerows(imu_timestamps)

        # Save the camera capture timestamps
        with open(f"{dir_path}/logs/capture_timestamps.csv", "w", newline="") as f:
            imuwriter = csv.writer(f)
            imuwriter.writerows(capture_timestamps)
        
        # Save the robot poses
        with open(f"{dir_path}/logs/robot_poses.csv", "w", newline="") as f:
            posewriter = csv.writer(f)
            posewriter.writerows(ur_poses)

    # Create ChAruCo board object, calibrate the camera intrinsics, and estimate ChAruCo poses
    charucoObj = charuco.charuco(3, 3, 0.063, 0.049, f"{dir_path}/logs/captures")
    camMat, distCoef = charucoObj.intrinsicsCalibration()
    charucoPoses = charucoObj.poseEstimation(camMat, distCoef)

    # Save the ChAruCo poses
    with open(f"{dir_path}/logs/charuco_poses.csv", "w", newline="") as f:
        posewriter = csv.writer(f)
        posewriter.writerows(charucoPoses)