def ur_pose_average(ur_poses, repetitions):
    if repetitions != 1:
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
        
        return tot_avg_ur
    else:
        return ur_poses