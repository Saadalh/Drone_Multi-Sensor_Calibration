from scipy.spatial.transform import Rotation as R
import numpy as np
from cv2 import Rodrigues
import math
import copy
import glob
import os
import re

# Averages the poses of all repetitions
def poses_average(ur_poses, repetitions): # len(urposes) =  6 , rep = 2, parts = 3, len(seglist) = 2, len(seglist[0]) = 3
    if repetitions != 1:
        seg_list = []
        parts = int(len(ur_poses)/repetitions)
        for i in range(repetitions):
            newlist = ur_poses[0:(parts)]
            seg_list.append(newlist)
            del ur_poses[0:(parts)]
        tot_avg_ur = []
        for x in range(parts):
            avg_ur = []
            tx_tot = 0
            ty_tot = 0
            tz_tot = 0
            rx_tot = 0
            ry_tot = 0
            rz_tot = 0
            for i in range(repetitions):
                tx_tot += seg_list[i][x][0] 
                ty_tot += seg_list[i][x][1] 
                tz_tot += seg_list[i][x][2] 
                rx_tot += seg_list[i][x][3] 
                ry_tot += seg_list[i][x][4] 
                rz_tot += seg_list[i][x][5]

            avg_ur.append(tx_tot/len(seg_list)) 
            avg_ur.append(ty_tot/len(seg_list)) 
            avg_ur.append(tz_tot/len(seg_list)) 
            avg_ur.append(rx_tot/len(seg_list)) 
            avg_ur.append(ry_tot/len(seg_list)) 
            avg_ur.append(rz_tot/len(seg_list)) 
            tot_avg_ur.append(avg_ur)
        
        return tot_avg_ur
    else:
        return ur_poses

# Picks the needed IMU poses from the log file based on the timestamps list
def imu_poses_picker(imu_timestamps, imu_dict_list):
    # Get the wanted IMU poses
    picked_imu_posepairs = [] # a list for each station transformation
    for imutsp in imu_timestamps:
        imutsp_list = [] # a list for each pose pair
        for imuts in imutsp:
            timediff = 100
            dummy_imu_dict = {} # a dictionary for each pose
            for dict in imu_dict_list:
                if abs(dict["timestamp"]-imuts) < timediff:
                    timediff = abs(dict["timestamp"]-imuts)
                    dummy_imu_dict = dict
            imutsp_list.append(dummy_imu_dict)
        picked_imu_posepairs.append(imutsp_list)

    return picked_imu_posepairs

# Transforms the imu pose-pairs to individual poses
def imu_pairs2pose(posepairs, repetitions, stations):
    poses = []
    transformations = []
    for pair in posepairs:
        trans = []

        trans.append(pair[1]["stateEstimate.x"] - pair[0]["stateEstimate.x"])
        trans.append(pair[1]["stateEstimate.y"] - pair[0]["stateEstimate.y"])
        trans.append(pair[1]["stateEstimate.z"] - pair[0]["stateEstimate.z"])

        r0 = R.from_euler('xyz', [pair[0]["stateEstimate.roll"], pair[0]["stateEstimate.pitch"], pair[0]["stateEstimate.yaw"]], degrees=True)
        r1 = R.from_euler('xyz', [pair[1]["stateEstimate.roll"], pair[1]["stateEstimate.pitch"], pair[1]["stateEstimate.yaw"]], degrees=True)
        r01 = np.matmul(np.linalg.inv(r0.as_matrix()), r1.as_matrix())
        r01 = R.from_matrix(r01.tolist())
        trans.append(r01) # results in a transformation in the following format: (tx,ty,tz,3x3rotmat)
        
        transformations.append(trans)

    irot = R.from_matrix([[1,0,0],[0,1,0],[0,0,1]])
    ipose = [0, 0, 0, irot]
    cpose = copy.deepcopy(ipose)
    npose = copy.deepcopy(ipose)
    npose.append(0)
    npose.append(0)

    for trans in transformations:
        if (len(poses)%stations==0):
            npose[0] = 0
            npose[1] = 0
            npose[2] = 0
            npose[3] = 0
            npose[4] = 0
            npose[5] = 0
            poses.append(copy.deepcopy(npose))
        npose[0] = cpose[0] + trans[0]
        npose[1] = cpose[1] + trans[1]
        npose[2] = cpose[2] + trans[2]

        crot = R.from_matrix(np.matmul(cpose[3].as_matrix(), trans[3].as_matrix())) 

        npose[3] = crot.as_euler('xyz')[0]
        npose[4] = crot.as_euler('xyz')[1]
        npose[5] = crot.as_euler('xyz')[2]

        poses.append(copy.deepcopy(npose))

        cpose = copy.deepcopy(npose)        

        cpose.pop(-1)
        cpose.pop(-1)
        cpose[3] = crot

    return poses

# Pick the needed, and deletes the unwanted captures from the specified directory based on the timestamps list
def captures_picker(dir_path, capture_timestamps):
    # Get the file names of the needed captures
    all_captures = glob.glob(f'{dir_path}/*.jpg')
    capture_files = []
    for ts in capture_timestamps:
        timediff = 10
        for scap in all_captures:
            caphead, captail = os.path.split(scap)
            uscount = 0
            dcount = 0
            capts = ""
            capnum = ""
            for c in captail:
                if c == ".":
                    dcount += 1
                if uscount == 2 and dcount < 2:
                    capts += c 
                if uscount == 1 and dcount == 0:
                    capnum += c
                if c == "_":
                    uscount += 1
            if abs(float(capts)-ts) < timediff:
                timediff = abs(float(capts)-ts)
                wantedcapts = capts
                wantedcapnum = capnum
        capture_files.append(f"{dir_path}/img_{wantedcapnum}{wantedcapts}.jpg")

    # Delete all unwanted capture files
    for cfile in all_captures:
        if cfile not in capture_files:
            os.remove(cfile)

    return capture_files

# sort_captures() support-fucntion to extract the number to-be-compared in the capture file name
def get_capture_number(filename):
    # Extracts the capture number from the file name
    # Assumes the file name format is '/path/to/file/img_{capture_number}_timestamp.jpg'
    match = re.search(r'img_(\d+)_\d+\.\d+\.jpg', filename)
    if match:
        return int(match.group(1))
    else:
        return -1
    
# Sort the captures based on their names in ascending order
def sort_captures(captures, repetitions):
    caps = captures
    if repetitions != 1:
        sorted_splitted_captures = []
        sorted_captures = sorted(caps, key=get_capture_number)
        elements = int(len(sorted_captures)/repetitions)
        for i in range(repetitions):
            sorted_splitted_captures.append(sorted_captures[0:elements])
            del sorted_captures[0:elements]
        return sorted_splitted_captures
    else:
        return caps

# Split the poses to lists for each repetition  
def split_poses(poses):
    tvecs = []
    rvecs = []
    for pose in poses:
        tvec = np.array([pose[0], pose[1], pose[2]])
        rotvec = R.from_rotvec([pose[3], pose[4], pose[5]])
        rodvec = Rodrigues(rotvec.as_matrix())
        tvecs.append(tvec)
        rvecs.append(rodvec[0])    
    return tvecs, rvecs