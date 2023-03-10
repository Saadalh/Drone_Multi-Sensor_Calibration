import rtde_control as rtdec
import rtde_receive as rtder
import numpy as np
import time

# Creating controller and receiver objects 
rtde_c = rtdec.RTDEControlInterface("172.31.1.200")
rtde_r = rtder.RTDEReceiveInterface("172.31.1.200")

# Define the reference orientation for the TCP
v = 0.12
a = 0.1
r = 0.05

# Generate random capture position
cap_pos_x = -0.100 # random in the range of (-359, 220)
cap_pos_y = -0.500 # random in the range of (-740, -320)
cap_pos_z = 0.350 # random in the range of (203, 315)

# Move the TCP to the capture position
cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, 3.14, 0, 0]
rtde_c.moveJ_IK(cap_target_pose, v, a, False)
time.sleep(0.2)

# Get the current pose in the base CS
actual_pose = rtde_r.getActualTCPPose()

# Define the pose change wrt actual_pose
relative_pose_change = [0, 0, 0, 0, 0.5, 0]

# Get the new pose wrt base CS, this should shift the y-position with -0.1
new_pose = rtde_c.poseTrans(actual_pose, relative_pose_change)

# Move to the new pose
#rtde_c.moveJ_IK(new_pose, v, a, False)







# If this works, then get the cap_pose, get the required rotation to point at the object, get new pose using posetrans, and move the tcp there. 