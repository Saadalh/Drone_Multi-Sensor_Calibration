import numpy as np
import random
import time
import scipy
import rtde_control as rtdec
import rtde_receive as rtder

class urControl:

    def __init__(self, ip, v, a):
        # Creating controller and receiver objects 
        self.rtde_c = rtdec.RTDEControlInterface(ip)
        self.rtde_r = rtder.RTDEReceiveInterface(ip)

        # Define the reference orientation for the TCP
        self.v = v
        self.a = a

    def read_pose(self):
        # Read current pose of the TCP
        return self.rtde_r.getActualTCPPose()

    def target_generator(self):
        # Generate a random position based on the specified ranges
        ranpos_x = round(random.uniform(-0.100, 0.100), 4)
        ranpos_y = round(random.uniform(-0.500, -0.400), 4)
        ranpos_z = round(random.uniform(0.203, 0.315), 4)

        return ranpos_x, ranpos_y, ranpos_z

    def move_home(self):
        # Move the TCP to the point where the calibration object needs to be place
        self.base_target_pose = [0.00745, -0.595, 0.146, 3.14, 0, 0]
        self.rtde_c.moveJ_IK(self.base_target_pose, self.v, self.a, False)
        time.sleep(0.2)

    def move_random(self):
        # Generate random capture position
        cap_pos_x, cap_pos_y, cap_pos_z = self.target_generator()

        # Calculate the vector from the TCP to the target
        cap_target_vec = np.array([self.base_target_pose[0]-cap_pos_x, -(self.base_target_pose[1]-cap_pos_y), -(0.0065-cap_pos_z)])
        cap_target_vec = np.reshape(cap_target_vec, (1, -1))

        # Define the reference vector and calculate the required rotation values
        ref_vec = np.array([0, 0, 0.01])
        ref_vec = np.reshape(ref_vec, (1, -1))
        cap_target_rotobj = scipy.spatial.transform.Rotation.align_vectors(cap_target_vec, ref_vec)
        cap_target_rotvec = cap_target_rotobj[0].as_rotvec()

        # Move the TCP to the capture position
        cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, 3.14, 0, 0]
        self.rtde_c.moveJ_IK(cap_target_pose, self.v, self.a, False)
        time.sleep(0.2)

        # Get the pose of the TCP to point at the capture pose relative to base
        yDist = cap_pos_y - self.base_target_pose[1]
        xAng = np.arctan((yDist/cap_pos_z))
        xDist = cap_pos_x - self.base_target_pose[0]
        yAng = np.arctan((xDist/cap_pos_z))

        cap_target_rotobj_hm = scipy.spatial.transform.Rotation.from_euler('xy', [xAng, yAng])
        cap_target_rotvec_hm = cap_target_rotobj_hm.as_rotvec()

        cap_target_pose_change = [0, 0, 0, cap_target_rotvec[0], cap_target_rotvec[1], 0]
        #cap_target_pose_change = [0, 0, 0, cap_target_rotvec_hm[0], cap_target_rotvec_hm[1], 0]
        cap_target_pose = self.rtde_c.poseTrans(cap_target_pose, cap_target_pose_change)
        # Orient the TCP to point at the target
        self.rtde_c.moveJ_IK(cap_target_pose, self.v, self.a, False)
        time.sleep(0.2)

if __name__ == "__main__":
    ip = "172.31.1.200"
    v = 0.12
    a = 0.1

    urc = urControl(ip, v, a)
    urc.move_home()
    urc.move_random()
    urc.move_random()
    urc.move_home()