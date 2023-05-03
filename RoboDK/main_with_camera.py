import robolink   
from robodk import robomath
import time
import os  
import random
import charuco_intrinsics_calibration as calib
import threading
import numpy as np
import cv2 as cv
import scipy
import math
from scipy.spatial.transform import Rotation as R

class Camera:
    def __init__(self, cam):
        self.cap_frame = None
        self.camobj = cam

    def stream(self):
        self.camobj.setParam('Open', 1)
        print("Streaming LIVE!")
        while self.camobj.setParam('isOpen') == '1':
            # Get camera frames
            bytes_img = RDK.Cam2D_Snapshot('', self.camobj)
            if isinstance(bytes_img, bytes) and bytes_img != b'':
                nparr = np.frombuffer(bytes_img, np.uint8)
                self.cap_frame = cv.imdecode(nparr, cv.IMREAD_COLOR)
            if self.cap_frame is None:
                break

    def save_capture(self, x, i):
        print("Captured frame!")
        cv.imwrite(f"{dir_path}/captures/cap_{x+1}{i+1}.jpg", self.cap_frame)

    def stop(self):
        RDK.Cam2D_Close(self.camobj)

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
cam = RDK.Item('cf_camera_rgb_324x244', robolink.ITEM_TYPE_CAMERA)
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

# Create streaming thread
camObj = Camera(cam)
stream_thread = threading.Thread(target=camObj.stream)
stream_thread.start() 

# Move to the 24 targets and capture images:
tcp_poses = []
imu_poses = []
init_imu_poses = []
charuco_poses = []
captures = []

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
        camObj.save_capture(x, i)
        captures.append(f"{dir_path}/captures/cap_{x+1}{i+1}.jpg") # capture frame
        time.sleep(0.2)
# End streaming
camObj.stop()
stream_thread.join()
print("Stream broken.")

# Calibrate the camera and estimate charuco poses
charuco_poses = []
charuco = calib.charuco(3, 5, 0.055, 0.043, f"{dir_path}/captures", captures)
cameraMatrix, distCoeffs = charuco.intrinsicsCalibration()
charuco.poseEstimation(cameraMatrix, distCoeffs, charuco_poses) # results in poses of the caruco board wrt the camera cs

# Split the rotation and translation parts of each pose (xyz eurler representation)
t_tcp, re_tcp = split_poses(tcp_poses)
t_imu, re_imu = split_poses(imu_poses)
t_init_imu, re_init_imu = split_poses(init_imu_poses)
t_charuco, re_charuco = split_poses(charuco_poses)

# Add noise to the dataset
nt_tcp, nre_tcp = add_noise(t_tcp, re_tcp, 0, 0.1, 0, 1)
nt_imu, nre_imu = add_noise(t_imu, re_imu, 0, 0.1, 0, 1)
nt_init_imu, nre_init_imu = add_noise(t_init_imu, re_init_imu, 0, 1, 0, 1)
nt_charuco, nre_charuco = add_noise(t_charuco, re_charuco, 0, 1, 0, 1)

# Convert to OpenCV Rodigues representation
rr_tcp = euler_2_rod(re_tcp)
rr_imu = euler_2_rod(re_imu)
rr_init_imu = euler_2_rod(re_init_imu)
rr_charuco = euler_2_rod(re_charuco)

nrr_tcp = euler_2_rod(nre_tcp)
nrr_imu = euler_2_rod(nre_imu)
nrr_init_imu = euler_2_rod(nre_init_imu)
nrr_charuco = euler_2_rod(nre_charuco)

# X: IMU to Camera / Y: TCP to IMU / Z: TCP to Camera
# Perform hand-eye calibration to find the ground-truth X, Y, and Z loops
rm_X, t_X = cv.calibrateHandEye(rr_imu, t_imu, rr_charuco, t_charuco, method=cv.CALIB_HAND_EYE_PARK)
rm_Y, t_Y = cv.calibrateHandEye(rr_tcp, t_tcp, rr_init_imu, t_init_imu, method=cv.CALIB_HAND_EYE_PARK)
rm_Z, t_Z = cv.calibrateHandEye(rr_tcp, t_tcp, rr_charuco, t_charuco, method=cv.CALIB_HAND_EYE_PARK)

rotvec_X = matrix_2_rotvec(rm_X)
rotvec_Y = matrix_2_rotvec(rm_Y)
rotvec_Z = matrix_2_rotvec(rm_Z)

re_X = matrix_2_euler(rm_X)
re_Y = matrix_2_euler(rm_Y)
re_Z = matrix_2_euler(rm_Z)

# Perform hand-eye calibration to find the noisey X, Y, and Z loops
nrm_X, nt_X = cv.calibrateHandEye(nrr_imu, nt_imu, nrr_charuco, nt_charuco, method=cv.CALIB_HAND_EYE_PARK)
nrm_Y, nt_Y = cv.calibrateHandEye(nrr_tcp, nt_tcp, nrr_init_imu, nt_init_imu, method=cv.CALIB_HAND_EYE_PARK)
nrm_Z, nt_Z = cv.calibrateHandEye(nrr_tcp, nt_tcp, nrr_charuco, nt_charuco, method=cv.CALIB_HAND_EYE_PARK)

nrotvec_X = matrix_2_rotvec(nrm_X)
nrotvec_Y = matrix_2_rotvec(nrm_Y)
nrotvec_Z = matrix_2_rotvec(nrm_Z)

nre_X = matrix_2_euler(nrm_X)
nre_Y = matrix_2_euler(nrm_Y)
nre_Z = matrix_2_euler(nrm_Z)

# Print out the calibration matrices
#print(f"The X hand-eye calibration matrix from IMU to Camera is:\nTranslation:\n{t_X}\nRotation:\n{rm_X}")
#print("---------------------------------------------------------------------")
#print(f"The Y hand-eye calibration matrix from TCP to IMU is:\nTranslation:\n{t_Y}\nRotation:\n{rm_Y}")
#print("---------------------------------------------------------------------")
#print(f"The Z hand-eye calibration matrix from TCP to Camera is:\nTranslation:\n{t_Z}\nRotation:\n{rm_Z}")

# Calculate the closed-loop rotation error
nr_error = abs(np.dot(np.transpose(nrm_Z), np.dot(nrm_X, nrm_Y)) - np.eye(3))
matobj = R.from_matrix(nr_error)
print("###########################")
print(f"Closed-Loop Rotation error (in Euler representation): {matobj.as_euler('xyz', degrees=True)}")

# Calculate the closed-loop translation error
nt_error = abs(nt_Z - (nt_X + nt_Y))
print("##########################")
print(f"Closed_Loop Translation error: {nt_error}")

#test = np.arccos((np.dot(rotvec_X, nrotvec_X))/(np.linalg.norm(rotvec_X)*np.linalg.norm(nrotvec_X)))
test = nre_X - re_X
print(f"test: {test}")
# Calculate the rotation error to the ground-truth for each calibration matrix
#error_angle_X = rotvec_X.inv().compose(nrotvec_X).magnitude()
#error_angle_Y = rotvec_Y.inv().compose(nrotvec_Y).magnitude()
#error_angle_Z = rotvec_Z.inv().compose(nrotvec_Z).magnitude()

# Calculate the translation error to the ground-truth
gtr_X_error_vec = t_X - nt_X
gtr_X_error =  np.linalg.norm(gtr_X_error_vec)

gtr_Y_error_vec = t_Y - nt_Y
gtr_Y_error =  np.linalg.norm(gtr_Y_error_vec)

gtr_Z_error_vec = t_Z - nt_Z
gtr_Z_error =  np.linalg.norm(gtr_Z_error_vec)