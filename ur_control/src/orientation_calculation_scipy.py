import numpy as np
import time
import scipy
import rtde_control as rtdec
import rtde_receive as rtder

# Creating controller and receiver objects 
rtde_c = rtdec.RTDEControlInterface("172.31.1.200")
rtde_r = rtder.RTDEReceiveInterface("172.31.1.200")

# Define the reference orientation for the TCP
v = 0.12
a = 0.1
r = 0.05

# Get current pose in x,y,z,rx,ry,rz (rotation vector representation)
# Improve: move the robot to the point where the object needs to be place instead
base_target_pose = rtde_r.getActualTCPPose()

# Fix orientation of the base-target pose
base_target_pose[3:6] = [3.14, 0, 0]
rtde_c.moveJ_IK()
print(f"Target pose in Base CS: {base_target_pose}")

# Generate random capture position
cap_pos_x = -0.100 # random in the range of (-359, 220)
cap_pos_y = -0.500 # random in the range of (-740, -320)
cap_pos_z = 0.350 # random in the range of (203, 315)

# Calculate and normalize the vector from the TCP to the target
cap_target_vec = np.array([base_target_pose[0]-cap_pos_x, -(base_target_pose[1]-cap_pos_y), -(base_target_pose[2]-cap_pos_z)])
print(f"Target position in TCP CS: {cap_target_vec}")

# Move the TCP to the capture position
cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, 3.14, 0, 0]
rtde_c.moveJ_IK(cap_target_pose, v, a, False)
time.sleep(0.2)

# Define the reference vector
ref_vec = np.array([0, 0, 0.01])
ref_vec = np.reshape(ref_vec, (1, -1))

# Calculate the required rotation matrix
cap_target_rotobj = scipy.spatial.transform.Rotation.align_vectors(ref_vec, cap_target_vec)
cap_target_rotvec = cap_target_rotobj[0].as_matrix()
print(f"Rotation matrix: {cap_target_rotvec}")


# Orient the TCP to point at the target point
cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, cap_target_rotvec[0], cap_target_rotvec[1], cap_target_rotvec[2]]
#rtde_c.moveJ_IK(cap_target_pose, v, a, False)
#time.sleep(0.2)