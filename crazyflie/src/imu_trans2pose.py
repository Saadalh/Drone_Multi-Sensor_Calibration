import copy

def imu_trans2pose():
    pointpairs = [[{"stateEstimate.x": 0, "stateEstimate.y": 0, "stateEstimate.z": 0, "stateEstimate.roll": 0, "stateEstimate.pitch": 0, "stateEstimate.yaw": 0}, {"stateEstimate.x": 10, "stateEstimate.y": 5, "stateEstimate.z": 10, "stateEstimate.roll": 0.1, "stateEstimate.pitch": 0.2, "stateEstimate.yaw": 0.1}],
                  [{"stateEstimate.x": 0, "stateEstimate.y": 0, "stateEstimate.z": 0, "stateEstimate.roll": 0, "stateEstimate.pitch": 0, "stateEstimate.yaw": 0}, {"stateEstimate.x": 5, "stateEstimate.y": 10, "stateEstimate.z": 5, "stateEstimate.roll": 0.2, "stateEstimate.pitch": 0.1, "stateEstimate.yaw": 0.2}]]
    rvec = []
    tvec = []
    pose = []
    poses = []
    transformations = []
    for pair in pointpairs:
        trans = []
        trans.append(pair[1]["stateEstimate.x"] - pair[0]["stateEstimate.x"])
        trans.append(pair[1]["stateEstimate.y"] - pair[0]["stateEstimate.y"])
        trans.append(pair[1]["stateEstimate.z"] - pair[0]["stateEstimate.z"])
        trans.append(pair[1]["stateEstimate.roll"] - pair[0]["stateEstimate.roll"])
        trans.append(pair[1]["stateEstimate.pitch"] - pair[0]["stateEstimate.pitch"])
        trans.append(pair[1]["stateEstimate.yaw"] - pair[0]["stateEstimate.yaw"])
        transformations.append(trans)
    print(transformations)

    rvec = [0, 0, 0]
    tvec = [0, 0, 0]
    pose.append(tvec)
    pose.append(rvec)
    poses.append(pose)
    cpose = copy.deepcopy(pose)

    for trans in transformations:
        cpose[0][0] = cpose[0][0] + trans[0]
        cpose[0][1] = cpose[0][1] + trans[1]
        cpose[0][2] = cpose[0][2] + trans[2]
        cpose[1][0] = cpose[1][0] + trans[3]
        cpose[1][1] = cpose[1][1] + trans[4]
        cpose[1][2] = cpose[1][2] + trans[5]
        poses.append(cpose)
        cpose = copy.deepcopy(cpose)

    return poses

pps = imu_trans2pose()
print(pps)