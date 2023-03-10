import numpy as np
import random
import time
import scipy
import rtde_control as rtdec
import rtde_receive as rtder

def target_generator():
    # Generate a random position based on the specified ranges
    ranpos_x = round(random.uniform(-0.359, 0.220), 4)
    ranpos_y = round(random.uniform(-0.740, -0.320), 4)
    ranpos_z = round(random.uniform(0.203, 0.315), 4)

    return ranpos_x, ranpos_y, ranpos_z

# Creating controller and receiver objects 
rtde_c = rtdec.RTDEControlInterface("172.31.1.200")
rtde_r = rtder.RTDEReceiveInterface("172.31.1.200")

# Define the reference orientation for the TCP
v = 0.12
a = 0.1
r = 0.05

# Get current pose in x,y,z,rx,ry,rz (rotation vector representation)
# Improve: move the robot to the point where the object needs to be place instead
base_target_pose = [0, -0.6, 0.008, 3.14, 0, 0]

# Fix orientation of the base-target pose
rtde_c.moveL(base_target_pose, v, a, False)
time.sleep(0.2)
input("Place the center of the calibration object directly under the drone. Then press Enter to continue.")

# Generate random capture position
cap_pos_x, cap_pos_y, cap_pos_z = target_generator()

# Calculate the vector from the TCP to the target
cap_target_vec = np.array([base_target_pose[0]-cap_pos_x, -(base_target_pose[1]-cap_pos_y), -(base_target_pose[2]-cap_pos_z)])
cap_target_vec = np.reshape(cap_target_vec, (1, -1))

# Define the reference vector and calculate the required rotation values
ref_vec = np.array([0, 0, 0.01])
ref_vec = np.reshape(ref_vec, (1, -1))
cap_target_rotobj = scipy.spatial.transform.Rotation.align_vectors(cap_target_vec, ref_vec)
cap_target_rotvec = cap_target_rotobj[0].as_rotvec()

# Move the TCP to the capture position
cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, 3.14, 0, 0]
rtde_c.moveJ_IK(cap_target_pose, v, a, False)
time.sleep(0.2)
print(f"The cap_target_pose is: {cap_target_pose}.")

# Get the pose of the TCP to point at the capture pose relative to base
cap_target_pose_change = [0, 0, 0, cap_target_rotvec[0], cap_target_rotvec[1], cap_target_rotvec[2]]
cap_target_pose = rtde_c.poseTrans(cap_target_pose, cap_target_pose_change)
# Orient the TCP to point at the target
input(f"The cap_target_pose is: {cap_target_pose}. \nMake sure it is correct, and then press Enter.")
rtde_c.moveL(cap_target_pose, v, a, False)
time.sleep(0.2)