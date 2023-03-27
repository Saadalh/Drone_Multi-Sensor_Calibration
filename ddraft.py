ur_poses = [ [[1, 1, 1, 1, 1, 1] , [2, 2, 2, 2, 2, 2]
            ,[3, 3, 3, 3, 3, 3]] , [[3, 3, 3, 3, 3, 3] , 
             [8, 8, 8, 8, 8, 8] , [3, 3, 3, 3, 3, 3]] ]

tot_avg_ur = []
for x in range(len(ur_poses[0])):
    avg_ur = []
    tx_tot = 0
    ty_tot = 0
    tz_tot = 0
    rx_tot = 0
    ry_tot = 0
    rz_tot = 0
    for i in range(len(ur_poses)):
        tx_tot += ur_poses[i][x][0] 
        ty_tot += ur_poses[i][x][1] 
        tz_tot += ur_poses[i][x][2] 
        rx_tot += ur_poses[i][x][3] 
        ry_tot += ur_poses[i][x][4] 
        rz_tot += ur_poses[i][x][5]

    avg_ur.append(tx_tot/len(ur_poses)) 
    avg_ur.append(ty_tot/len(ur_poses)) 
    avg_ur.append(tz_tot/len(ur_poses)) 
    avg_ur.append(rx_tot/len(ur_poses)) 
    avg_ur.append(ry_tot/len(ur_poses)) 
    avg_ur.append(rz_tot/len(ur_poses)) 
    tot_avg_ur.append(avg_ur)

imu_poses = [ [[1,1,1],[2,2,2]] , [[3,3,3],[4,4,4]], [[10,10,10],[4,4,4]], 
              [[5,5,5],[6,6,6]] , [[7,7,7],[8,8,8]], [[6,6,6],[12,12,12]] ]
# Define the number of repetitions
repetitions = 2

num_chunks = len(imu_poses) // repetitions
remainder = len(imu_poses) % repetitions
chunks = [imu_poses[i*(num_chunks):(i+1)*(num_chunks)] for i in range(repetitions)]
if remainder:
    chunks[-1] += imu_poses[-remainder:]

averaged_poses = []
for i in range(num_chunks):
    mean_pose = [sum([chunk[i][j] for chunk in chunks], []) for j in range(len(imu_poses[0]))]
    mean_pose = [sum(mean_pose[j][k] for j in range(len(mean_pose))) / len(mean_pose) for k in range(len(mean_pose[0]))]
    averaged_poses.append(mean_pose)
    
print(tot_avg_ur)