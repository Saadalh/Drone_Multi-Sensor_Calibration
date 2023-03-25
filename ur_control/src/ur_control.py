import rtde_control as rtdec
import rtde_receive as rtder
import numpy as np
import random
import scipy
import math
import time

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
        pose = self.rtde_r.getActualTCPPose()
        rvec = np.array([pose[3],pose[4],pose[5]])
        tvec = np.array([pose[0],pose[2],pose[3]])
        return rvec, tvec

    def random_pose_generator(self):
        # Generate a random position based on the specified ranges
        ranpos_x = round(random.uniform(-0.200, 0.200), 4)
        ranpos_y = round(random.uniform(-0.580, -0.250), 4)
        ranpos_z = round(random.uniform(0.350, 0.415), 4)

        return ranpos_x, ranpos_y, ranpos_z
    
    def systematic_pose_generator(pose, numbers, layers, factor, increase):

        poses = np.empty((layers, numbers, 6))

        # Define variabels
        trans = 80 # distance between the levels -> change the distances for your usecase

        rot = 10 # angle for the tilt of the camera -> change the tilt for your usecase

        for n in range(layers):
            multiplierT = factor + (increase*n)

            multiplierR = factor + (increase*(0.25*n))

            for i in range(numbers):

                ########

                # Program it as a 4x4 Matrix and use the translation vectors to rotate the robot arm

                # Defeinieren eines Roatationsvektor und eines Translationsvektor

                ########

                #print("n: " + str(n))

                poses[n][i][0] = pose[0]+(multiplierT*trans*(math.sin(math.radians((360/numbers)*i))))

                poses[n][i][1] = pose[1]+ trans*(n+1) #mutliplierT

                poses[n][i][2] = pose[2]+(multiplierT*trans*(math.cos(math.radians((360/numbers)*i))))

                poses[n][i][3] = pose[3]+math.sin(math.radians(rot*multiplierR))*(math.cos(math.radians((360/numbers)*i)))

                poses[n][i][4] = pose[4]+math.sin(math.radians(rot*multiplierR))*-(math.sin(math.radians((360/numbers)*i)))#+(rot*cos((360/numbers)*i))

                poses[n][i][5] = pose[5]

        #print("Positionen: "+ str(poses))

        return poses

    def move_home(self):
        # Move the TCP to the point where the calibration object needs to be place
        self.base_target_pose = [0.00745, -0.440, 0.146, 3.14, 0, 0]
        self.rtde_c.moveJ_IK(self.base_target_pose, self.v, self.a, False)
        time.sleep(0.5)

    def move_target(self):
        # Generate random capture position
        cap_pos_x, cap_pos_y, cap_pos_z = self.random_pose_generator()

        # Calculate the vector from the TCP to the target
        cap_target_vec = np.array([self.base_target_pose[0]-cap_pos_x, -(self.base_target_pose[1]-cap_pos_y), -(0.0065-cap_pos_z)])
        cap_target_vec = np.reshape(cap_target_vec, (1, -1))

        # Define the reference vector and calculate the required rotation values
        ref_vec = np.array([0, 0, 0.01])
        ref_vec = np.reshape(ref_vec, (1, -1))
        #cap_target_rotobj = scipy.spatial.transform.Rotation.align_vectors(cap_target_vec, ref_vec)
        #cap_target_rotvec = cap_target_rotobj[0].as_euler('xy')
        #print(f"Cap-Target-Rot: {cap_target_rotvec}")

        # Move the TCP to the capture position
        cap_target_pose = [cap_pos_x, cap_pos_y, cap_pos_z, 3.14, 0, 0]
        #self.rtde_c.moveJ_IK(cap_target_pose, self.v, self.a, False)
        #time.sleep(0.1)

        # Get the pose of the TCP to point at the capture pose relative to base
        yDist = cap_pos_y - self.base_target_pose[1]
        if yDist > 0:
            xAng = -(np.arctan((abs(yDist)/cap_pos_z)))
        else:
            xAng = np.arctan((abs(yDist)/cap_pos_z))

        xDist = cap_pos_x - self.base_target_pose[0]
        if xDist > 0:
            yAng = -np.arctan((abs(xDist)/cap_pos_z))
        else:
            yAng = (np.arctan((abs(xDist)/cap_pos_z)))

        cap_target_rotobj_hm = scipy.spatial.transform.Rotation.from_euler('xy', [xAng, yAng])
        cap_target_rotvec_hm = cap_target_rotobj_hm.as_rotvec()

        #cap_target_pose_change = [0, 0, 0, cap_target_rotvec[0], cap_target_rotvec[1], 0]
        cap_target_pose_change = [0, 0, 0, xAng, yAng, 0]
        cap_target_pose = self.rtde_c.poseTrans(cap_target_pose, cap_target_pose_change)
        # Orient the TCP to point at the target
        self.rtde_c.moveJ_IK(cap_target_pose, self.v, self.a, False)
        time.sleep(0.1)

if __name__ == "__main__":
    ip = "172.31.1.200"
    v = 0.12
    a = 0.1

    urc = urControl(ip, v, a)
    urc.move_home()
    urc.move_target()
    urc.move_target()
    urc.move_home()