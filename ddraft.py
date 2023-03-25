   

 # Get the wanted IMU logs
stations_imu = []
for imutsp in imu_timestamps:
        imutsp_list = []
        for imuts in imutsp:
                print(imuts)
                timediff = 100
                dummy_imu_dict = {}
                for dict in imu_dict_list:
                        if abs(dict["timestamp"]-imuts) < timediff:
                                timediff = abs(dict["timestamp"]-imuts)
                                dummy_imu_dict = dict
                        imutsp_list.append(dummy_imu_dict)
        stations_imu.append(imutsp_list)