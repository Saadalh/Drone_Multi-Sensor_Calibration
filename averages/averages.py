import numpy as np
import copy
import glob
import os
import re

def poses_average(ur_poses, repetitions):
    if repetitions != 1:
        seg_list = []
        parts = int(len(ur_poses)/repetitions)
        for i in range(repetitions):
            newlist = ur_poses[0:parts]
            seg_list.append(newlist)
            del ur_poses[0:2]
            
        tot_avg_ur = []
        for x in range(len(seg_list[0])):
            avg_ur = []
            tx_tot = 0
            ty_tot = 0
            tz_tot = 0
            rx_tot = 0
            ry_tot = 0
            rz_tot = 0
            for i in range(len(seg_list)):
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

def imu_pairs2pose(posepairs):
    poses = []
    transformations = []
    for pair in posepairs:
        trans = []
        trans.append(pair[1]["stateEstimate.x"] - pair[0]["stateEstimate.x"])
        trans.append(pair[1]["stateEstimate.y"] - pair[0]["stateEstimate.y"])
        trans.append(pair[1]["stateEstimate.z"] - pair[0]["stateEstimate.z"])
        trans.append(pair[1]["stateEstimate.roll"] - pair[0]["stateEstimate.roll"])
        trans.append(pair[1]["stateEstimate.pitch"] - pair[0]["stateEstimate.pitch"])
        trans.append(pair[1]["stateEstimate.yaw"] - pair[0]["stateEstimate.yaw"])
        transformations.append(trans)
    print(transformations)

    ivec = [0, 0, 0, 0, 0, 0]
    poses.append(ivec)
    cpose = copy.deepcopy(ivec)

    for trans in transformations:
        cpose[0] = cpose[0] + trans[0]
        cpose[1] = cpose[1] + trans[1]
        cpose[2] = cpose[2] + trans[2]
        cpose[3] = cpose[3] + trans[3]
        cpose[4] = cpose[4] + trans[4]
        cpose[5] = cpose[5] + trans[5]
        poses.append(cpose)
        cpose = copy.deepcopy(cpose)

    return poses

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
            if abs(float(capts)-ts[0]) < timediff:
                timediff = abs(float(capts)-ts[0])
                wantedcapts = capts
                wantedcapnum = capnum
        capture_files.append(f"{dir_path}/logs/captures/img_{wantedcapnum}{wantedcapts}.jpg")

    # Delete all unwanted capture files
    for cfile in all_captures:
        if cfile not in capture_files:
            os.remove(cfile)

    return capture_files

def get_capture_number(filename):
    # Extracts the capture number from the file name
    # Assumes the file name format is '/path/to/file/img_{capture_number}_timestamp.jpg'
    match = re.search(r'img_(\d+)_\d+\.\d+\.jpg', filename)
    if match:
        return int(match.group(1))
    else:
        return -1
    
def sort_captures(captures, repetitions):
    if repetitions != 1:
        sorted_splitted_captures = []
        sorted_captures = sorted(captures, key=get_capture_number)
        elements = int(len(sorted_captures)/repetitions)
        for i in range(repetitions):
            sorted_splitted_captures.append(sorted_captures[0:elements])
            del sorted_captures[0:elements]
        return sorted_splitted_captures
    else:
        return captures
    
def split_poses(poses):
    tvecs = []
    rvecs = []
    for pose in range(len(poses)):
        tvec = np.array([pose[0], pose[1], pose[2]])
        rvec = np.array([pose[3], pose[4], pose[5]])
        tvecs.append(tvec)
        rvecs.append(rvec)
    
    return tvecs, rvecs