from scipy.spatial.transform import Rotation as R
import charuco_intrinsics_calibration as calib
from robodk import robomath
import numpy as np
import cv2 as cv
import threading
import robolink   
import random
import scipy
import time
import math
import os  

def add_noise(t, r, tnm, tns, rnm, rns):
    translation_noise_mean = tnm  # Mean of translation noise
    translation_noise_stddev = tns  # Standard deviation of translation noise

    rotation_noise_mean = rnm  # Mean of rotation noise in degrees
    rotation_noise_stddev = rns  # Standard deviation of rotation noise in degrees
    
    nt = []
    nr = []

    for i in range(len(t)):
        nti = []
        nri = []

        # Add noise to translation vector
        translation_noise = [random.gauss(translation_noise_mean, translation_noise_stddev)]
        nti.append((t[i][0] + translation_noise)[0])
        translation_noise = [random.gauss(translation_noise_mean, translation_noise_stddev)]
        nti.append((t[i][1] + translation_noise)[0])
        translation_noise = [random.gauss(translation_noise_mean, translation_noise_stddev)]
        nti.append((t[i][2] + translation_noise)[0])

        # Add noise to rotation vector
        rotation_noise = [random.gauss(rotation_noise_mean, rotation_noise_stddev)]
        nri.append((r[i][0] + rotation_noise)[0])
        rotation_noise = [random.gauss(rotation_noise_mean, rotation_noise_stddev)]
        nri.append((r[i][1] + rotation_noise)[0])
        rotation_noise = [random.gauss(rotation_noise_mean, rotation_noise_stddev)]
        nri.append((r[i][2] + rotation_noise)[0])

        nt.append(np.array(nti))
        nr.append(np.array(nri))

    return nt, nr

# Splits the poses to lists for each repetition
def split_poses(poses, in_m=False):
    tvecs = []
    rvecs = []
    if in_m == True:
        for pose in poses:
            tvec = np.array([pose[0]/1000, pose[1]/1000, pose[2]/1000])
            rvec = np.array([pose[3], pose[4], pose[5]])
            #reuler = R.from_euler('xyz', [pose[3], pose[4], pose[5]], degrees=True)
            #rvec = np.array(reuler.as_matrix())
            tvecs.append(tvec)
            rvecs.append(rvec)    
    else:
        for pose in poses:
            tvec = np.array([pose[0], pose[1], pose[2]])
            rvec = np.array([pose[3], pose[4], pose[5]])
            #reuler = R.from_euler('xyz', [pose[3], pose[4], pose[5]], degrees=True)
            #rvec = np.array(reuler.as_matrix())
            tvecs.append(tvec)
            rvecs.append(rvec)   
    return tvecs, rvecs

def euler_2_rod(res):
    rrs = []
    for re in res:
        rotobj = R.from_euler('xyz', re, degrees=True)
        rrs.append(cv.Rodrigues(rotobj.as_matrix())[0])
    return rrs

def matrix_2_rotvec(rm):
    rotobj = R.from_matrix(rm)
    rotvec = np.array(rotobj.as_rotvec())
    return rotvec

def matrix_2_euler(rm):
    rotobj = R.from_matrix(rm)
    re = np.array(rotobj.as_euler('xyz', degrees=True))
    return re

# Import the robot components:
RDK = robolink.Robolink()
robot = RDK.Item('UR5e', robolink.ITEM_TYPE_ROBOT)
initial_imu_frame = RDK.Item('initial_imu_frame', robolink.ITEM_TYPE_FRAME)
imu_frame = RDK.Item('cf_imu_frame', robolink.ITEM_TYPE_FRAME)
camera_frame = RDK.Item('cf_camera_frame', robolink.ITEM_TYPE_FRAME)
charuco_frame = RDK.Item('charuco_frame', robolink.ITEM_TYPE_FRAME)
tcp_frame = RDK.Item('tcp_frame', robolink.ITEM_TYPE_FRAME)
base_frame = RDK.Item('UR5e Base', robolink.ITEM_TYPE_FRAME)

# Set the tool frames and directory path
robot.setPoseFrame(base_frame)
robot.setPoseTool(robot.PoseTool())
robot.setSpeed(300)
dir_path = os.path.realpath(os.path.dirname(__file__))

# Move to the 24 targets and capture images:
tcp_poses = []
imu_poses = []
init_imu_poses = []
charuco_poses = []
#captures = []

home = RDK.Item('home')
robot.MoveL(home)
time.sleep(0.2)

for x in range(3):
    for i in range(8):
        target = (RDK.Item(f'loop{x+1} {i+1}')) # import targets
        if i+1 < 7:
            robot.MoveL(target) # moveL to the targets
        elif i+1 > 6:
            robot.MoveJ(target) # moveJ to the targets
        time.sleep(0.1)
        tcp_poses.append(robomath.Pose_2_Fanuc(tcp_frame.PoseWrt(base_frame))) # get the tcp pose
        init_imu_poses.append(robomath.Pose_2_Fanuc(initial_imu_frame.PoseWrt(imu_frame))) # get the intiial imu frame pose
        imu_poses.append(robomath.Pose_2_Fanuc(imu_frame.PoseWrt(initial_imu_frame))) # get the imu pose
        charuco_poses.append(robomath.Pose_2_Fanuc(charuco_frame.PoseWrt(camera_frame))) # get the charuco pose

# Split the rotation and translation parts of each pose (xyz eurler representation)
t_tcp, re_tcp = split_poses(tcp_poses)
t_imu, re_imu = split_poses(imu_poses)
t_init_imu, re_init_imu = split_poses(init_imu_poses)
t_charuco, re_charuco = split_poses(charuco_poses)

# Convert to OpenCV Rodigues representation
rr_tcp = euler_2_rod(re_tcp)
rr_imu = euler_2_rod(re_imu)
rr_init_imu = euler_2_rod(re_init_imu)
rr_charuco = euler_2_rod(re_charuco)

# X: IMU to Camera / Y: TCP to IMU / Z: TCP to Camera
# Perform hand-eye calibration to find the ground-truth X, Y, and Z loops
rm_X, t_X = cv.calibrateHandEye(rr_imu, t_imu, rr_charuco, t_charuco, method=cv.CALIB_HAND_EYE_PARK)
rm_Y, t_Y = cv.calibrateHandEye(rr_tcp, t_tcp, rr_init_imu, t_init_imu, method=cv.CALIB_HAND_EYE_PARK)
rm_Z, t_Z = cv.calibrateHandEye(rr_tcp, t_tcp, rr_charuco, t_charuco, method=cv.CALIB_HAND_EYE_PARK)

# Convert to euler angles
re_X = matrix_2_euler(rm_X)
re_Y = matrix_2_euler(rm_Y)
re_Z = matrix_2_euler(rm_Z)

# Create files to save values
ext_X_t_error_path = f"{dir_path}/values/ext_X_t_error.txt"
ext_X_t_error_file = open(ext_X_t_error_path, 'a')
ext_X_t_error_file.write("NEW DATASET: \n\n")
ext_Y_t_error_path = f"{dir_path}/values/ext_Y_t_error.txt"
ext_Y_t_error_file = open(ext_Y_t_error_path, 'a')
ext_Y_t_error_file.write("NEW DATASET: \n\n")
ext_Z_t_error_path = f"{dir_path}/values/ext_Z_t_error.txt"
ext_Z_t_error_file = open(ext_Z_t_error_path, 'a')
ext_Z_t_error_file.write("NEW DATASET: \n\n")

ext_X_roll_error_path = f"{dir_path}/values/ext_X_roll_error.txt"
ext_X_roll_error_file = open(ext_X_roll_error_path, 'a')
ext_X_roll_error_file.write("NEW DATASET: \n\n")
ext_X_pitch_error_path = f"{dir_path}/values/ext_X_pitch_error.txt"
ext_X_pitch_error_file = open(ext_X_pitch_error_path, 'a')
ext_X_pitch_error_file.write("NEW DATASET: \n\n")
ext_X_yaw_error_path = f"{dir_path}/values/ext_X_yaw_error.txt"
ext_X_yaw_error_file = open(ext_X_yaw_error_path, 'a')
ext_X_yaw_error_file.write("NEW DATASET: \n\n")

ext_Y_roll_error_path = f"{dir_path}/values/ext_Y_roll_error.txt"
ext_Y_roll_error_file = open(ext_Y_roll_error_path, 'a')
ext_Y_roll_error_file.write("NEW DATASET: \n\n")
ext_Y_pitch_error_path = f"{dir_path}/values/ext_Y_pitch_error.txt"
ext_Y_pitch_error_file = open(ext_Y_pitch_error_path, 'a')
ext_Y_pitch_error_file.write("NEW DATASET: \n\n")
ext_Y_yaw_error_path = f"{dir_path}/values/ext_Y_yaw_error.txt"
ext_Y_yaw_error_file = open(ext_Y_yaw_error_path, 'a')
ext_Y_yaw_error_file.write("NEW DATASET: \n\n")

ext_Z_roll_error_path = f"{dir_path}/values/ext_Z_roll_error.txt"
ext_Z_roll_error_file = open(ext_Z_roll_error_path, 'a')
ext_Z_roll_error_file.write("NEW DATASET: \n\n")
ext_Z_pitch_error_path = f"{dir_path}/values/ext_Z_pitch_error.txt"
ext_Z_pitch_error_file = open(ext_Z_pitch_error_path, 'a')
ext_Z_pitch_error_file.write("NEW DATASET: \n\n")
ext_Z_yaw_error_path = f"{dir_path}/values/ext_Z_yaw_error.txt"
ext_Z_yaw_error_file = open(ext_Z_yaw_error_path, 'a')
ext_Z_yaw_error_file.write("NEW DATASET: \n\n")

int_t_error_path = f"{dir_path}/values/int_t_error.txt"
int_t_error_file = open(int_t_error_path, 'a')
int_t_error_file.write("NEW DATASET: \n\n")

int_roll_error_path = f"{dir_path}/values/int_roll_error.txt"
int_roll_error_file = open(int_roll_error_path, 'a')
int_roll_error_file.write("NEW DATASET: \n\n")

int_pitch_error_path = f"{dir_path}/values/int_pitch_error.txt"
int_pitch_error_file = open(int_pitch_error_path, 'a')
int_pitch_error_file.write("NEW DATASET: \n\n")

int_yaw_error_path = f"{dir_path}/values/int_yaw_error.txt"
int_yaw_error_file = open(int_yaw_error_path, 'a')
int_yaw_error_file.write("NEW DATASET (Level 3): \n\n")

for i in range(100):
    # Add noise to the dataset
    nt_tcp, nre_tcp = add_noise(t_tcp, re_tcp, 0, 0.0, 0, 0)
    nt_imu, nre_imu = add_noise(t_imu, re_imu, 0, 10, 0, 5)
    nt_init_imu, nre_init_imu = add_noise(t_init_imu, re_init_imu, 0, 10, 0, 5)
    nt_charuco, nre_charuco = add_noise(t_charuco, re_charuco, 0, 0, 0, 0)

    # Convert to OpenCV Rodigues representation
    nrr_tcp = euler_2_rod(nre_tcp)
    nrr_imu = euler_2_rod(nre_imu)
    nrr_init_imu = euler_2_rod(nre_init_imu)
    nrr_charuco = euler_2_rod(nre_charuco)

    # Perform hand-eye calibration to find the noisey X, Y, and Z loops
    nrm_X, nt_X = cv.calibrateHandEye(nrr_imu, nt_imu, nrr_charuco, nt_charuco, method=cv.CALIB_HAND_EYE_PARK)
    nrm_Y, nt_Y = cv.calibrateHandEye(nrr_tcp, nt_tcp, nrr_init_imu, nt_init_imu, method=cv.CALIB_HAND_EYE_PARK)
    nrm_Z, nt_Z = cv.calibrateHandEye(nrr_tcp, nt_tcp, nrr_charuco, nt_charuco, method=cv.CALIB_HAND_EYE_PARK)

    # Convert to euler angles
    nre_X = matrix_2_euler(nrm_X)
    nre_Y = matrix_2_euler(nrm_Y)
    nre_Z = matrix_2_euler(nrm_Z)

    # Calculate the rotation error to the ground-truth
    gtr_X_error_vec = abs(re_X - nre_X)
    ext_X_roll_error_file.write(str(gtr_X_error_vec[0]) + "\n")
    ext_X_pitch_error_file.write(str(gtr_X_error_vec[1]) + "\n")
    ext_X_yaw_error_file.write(str(gtr_X_error_vec[2]) + "\n")
    
    gtr_Y_error_vec = abs(re_Y - nre_Y)
    ext_Y_roll_error_file.write(str(gtr_Y_error_vec[0]) + "\n")
    ext_Y_pitch_error_file.write(str(gtr_Y_error_vec[1]) + "\n")
    ext_Y_yaw_error_file.write(str(gtr_Y_error_vec[2]) + "\n")
    
    gtr_Z_error_vec = abs(re_Z - nre_Z)
    ext_Z_roll_error_file.write(str(gtr_Z_error_vec[0]) + "\n")
    ext_Z_pitch_error_file.write(str(gtr_Z_error_vec[1]) + "\n")
    ext_Z_yaw_error_file.write(str(gtr_Z_error_vec[2]) + "\n")
    # Calculate the translation error to the ground-truth
    gtt_X_error_vec = abs(t_X - nt_X)
    gtt_X_error =  np.linalg.norm(gtt_X_error_vec)
    ext_X_t_error_file.write(str(gtt_X_error) + "\n")
    
    gtt_Y_error_vec = abs(t_Y - nt_Y)
    gtt_Y_error =  np.linalg.norm(gtt_Y_error_vec)
    ext_Y_t_error_file.write(str(gtt_Y_error) + "\n")

    gtt_Z_error_vec = abs(t_Z - nt_Z)
    gtt_Z_error =  np.linalg.norm(gtt_Z_error_vec)
    ext_Z_t_error_file.write(str(gtt_Z_error) + "\n")

    # Calculate the closed-loop rotation error
    nrm_error = abs(np.dot(np.transpose(nrm_Z), np.dot(nrm_X, nrm_Y)) - np.eye(3))
    matobj = R.from_matrix(nrm_error)
    nre_error = matobj.as_euler('xyz', degrees=True)
    int_roll_error_file.write(str(nre_error[0]) + "\n")
    int_pitch_error_file.write(str(nre_error[1]) + "\n")
    int_yaw_error_file.write(str(nre_error[2]) + "\n")
    # Calculate the closed-loop translation error
    nt_error_vector = abs(nt_Z - (nt_X + nt_Y))
    nt_error = np.linalg.norm(nt_error_vector)
    int_t_error_file.write(str(nt_error) + "\n")
