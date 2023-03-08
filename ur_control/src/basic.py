import rtde_control as rtdec
import rtde_receive as rtder
import time

# Creating controller and receiver objects 
rtde_c = rtdec.RTDEControlInterface("172.31.1.200")
rtde_r = rtder.RTDEReceiveInterface("172.31.1.200")
init_q = rtde_r.getActualQ()

# Getting TCP pose
target = rtde_r.getTargetTCPPose()
print(target)
print(type(target))

# Creating joint-space target point in the robot base
new_q = init_q[:]
new_q[5] += 0.2

# Creating tool-space target point in the robot base
new_target = target[:]
new_target[2] += 0.08

# Move synchronously to new joint values in joint space.
#rtde_c.moveJ(new_q, 1.05, 1.4, False)
#time.sleep(0.2)

# Move synchronously to original joint values in joint space
#rtde_c.moveJ(init_q, 1.05, 1.4, False)
#time.sleep(0.2)

# Move synchronously to new position in joint space.
#rtde_c.moveJ_IK(new_target, 1.05, 1.4, False)
#time.sleep(0.2)

# Move synchronously to new position in joint space.
#rtde_c.moveJ_IK(target, 1.05, 1.4, False)
#time.sleep(0.2)
