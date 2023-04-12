import rtde_control as rtdec
import rtde_receive as rtder
import numpy as np
import random
import math
import time

class urControl:

    base_target_pose = [-0.338, -0.096, 0.025, 0, -3.14, 0] # the home pose, can be changed depending on the current setup
    system_target_poses = []

    def __init__(self, ip, v, a):
        # Creating controller and receiver objects 
        self.rtde_c = rtdec.RTDEControlInterface(ip)
        self.rtde_r = rtder.RTDEReceiveInterface(ip)

        # Define the reference orientation for the TCP
        self.v = v
        self.a = a
        self.poses = []
        self.repeat_counter = 0
        self.cspose = 0

        for i in range(3):
            self.system_target_poses.append([self.base_target_pose[0], self.base_target_pose[1]+0.211, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]-0.158, self.base_target_pose[1]+0.158, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]-0.151, self.base_target_pose[1], 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]-0.158, self.base_target_pose[1]-0.158, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0], self.base_target_pose[1]-0.211, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]+0.158, self.base_target_pose[1]-0.158, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]+0.151, self.base_target_pose[1], 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])
            self.system_target_poses.append([self.base_target_pose[0]+0.158, self.base_target_pose[1]+0.158, 0.446+(0.002*i), self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]])

    def read_pose(self):
        # Read current pose of the TCP
        pose = self.rtde_r.getActualTCPPose()
        return pose

    def random_pose_generator(self):
        # Generate a random position based on the specified ranges. Ranges can be changed based on the current setup
        ranpos_x = round(random.uniform(-0.500, -0.300), 4)
        ranpos_y = round(random.uniform(-0.350, 0), 4)
        ranpos_z = round(random.uniform(0.447, 0.450), 4)

        return ranpos_x, ranpos_y, ranpos_z

    def move_home(self):
        # Move the TCP to the point where the calibration object needs to be place
        self.rtde_c.moveJ_IK(self.base_target_pose, self.v, self.a, False)
        time.sleep(0.5)

    def move_target(self, type):

        # Generate random capture position
        if type == 'random':
            cap_pos_x, cap_pos_y, cap_pos_z = self.random_pose_generator()
        elif type == 'sys':
            cap_pos_x, cap_pos_y, cap_pos_z = self.sys_pose_generator()

        # Calculate the vector from the TCP to the target
        #cap_target_vec = np.array([self.base_target_pose[0]-cap_pos_x, -(self.base_target_pose[1]-cap_pos_y), -(0.0065-cap_pos_z)])
        #cap_target_vec = np.reshape(cap_target_vec, (1, -1))

        # Define the reference pose
        cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, self.base_target_pose[3], self.base_target_pose[4], self.base_target_pose[5]]

        # Get the orientation of the TCP to point at the capture pose relative to base
        yDist = cap_pos_y - self.base_target_pose[1] # tcp to the left of the object
        if yDist > 0:
            xAng = np.arctan((abs(yDist)/cap_pos_z))
        else:
            xAng = -(np.arctan((abs(yDist)/cap_pos_z)))

        xDist = cap_pos_x - self.base_target_pose[0] # tcp is behind the object
        if xDist > 0:
            yAng = (np.arctan((abs(xDist)/cap_pos_z)))
        else:
            yAng = -(np.arctan((abs(xDist)/cap_pos_z)))

        cap_target_pose_change = [0, 0, 0, xAng, yAng, 0]
        cap_target_pose = self.rtde_c.poseTrans(cap_target_pose, cap_target_pose_change)

        # Save the pose for regeneration
        self.poses.append(cap_target_pose)

        # Move the TCP to the target pose
        self.rtde_c.moveJ_IK(cap_target_pose, self.v, self.a, False)
        time.sleep(0.1)

    def sys_pose_generator(self):
        print(f"pose #{self.cspose+1}")
        x = self.system_target_poses[self.cspose][0]
        y = self.system_target_poses[self.cspose][1]
        z = self.system_target_poses[self.cspose][2]
        self.cspose += 1
        return x, y, z

    def move_repeat(self):
        self.rtde_c.moveJ_IK(self.poses[self.repeat_counter], self.v, self.a, False)
        self.repeat_counter += 1
        if self.repeat_counter == (len(self.poses)-1):
            self.repeat_counter = 0 

if __name__ == "__main__":
    ip = "172.31.1.200"
    v = 0.12
    a = 0.1

    urc = urControl(ip, v, a)
    #print(urc.read_pose())
    urc.move_home()
    #urc.move_target()
    #urc.move_target()
    #urc.move_home()