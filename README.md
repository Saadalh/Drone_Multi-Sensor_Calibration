# Hand-Eye-Calibration
This repository includes the code used to calibrate the IMU of a CrazyFlie drone, the mounted Himax camera, and the TCP of a UR5e to one another.

To re-simulate this experiment, please clone the directory fully without commiting any rename commands.
Following these steps, the hand-eye calibration could be executed:
1) Print a charuco board with 5 rows and 7 columns, 6x6 aruco dictionary, 55mm square length, and 43mm aruco-marker length. 
   The board can be downloaded from calibrate.io
  
2) Follow the steps on the CrazyFlie to enable the WiFi networking, radio logging, and install the necessary python libraries.

3) Install the Universal Robots RTDE python library to enable communication with the UR.

4) Turn on the CrazyFlie, and connect to its WiFi network.

5) Attach the drone the the UR via a Robotiq HandE gripper using the provided interface. Or print a custom interface that fits the availabe gripper.

4) Run the msc_main.py script with the following parameters: 
   -n: Specifies the CrazyFlie AI-deck IP address, default=192.168.4.1
   -i: Specifies the UR robot IP address, default=172.31.1.200
   -p: Specifies the port of the CrazyFlie AI-Deck, default=5000
   -s: Specifies the number of stations each repetition has, default=20
   -r: Specifies the number of repetitions of the generated stations, default=1 (no repetition)
   -v: Specifies the velocity of the robot arm, default=0.3
   -a: Specifies the acceleration of the robot arm, default=0.1
   -u: Specifies the CrazyFlie radio-AP URI, default=radio://0/100/2M/E7E7E7E701

The script should create/overwrite the following files:
- Camera captures saved in "logs/captures".
- "logs/robot_poses.csv": stores the UR TCP poses of each station.
- "logs/imu_poses.csv": stores the IMU poses of each station.
- "logs/charuco_poses.csv": stores the charuco poses of each station.
- "logs/camera2tcp_calibMat.txt": stores the calibration matrix between the drone camera and the UR TCP.
- "logs/imu2camera_calibMat.txt": stores the calibration matrix between the drone IMU and the drone camera.
- "logs/imu2tcp_calibMat.txt": stores the calibration martrix between the drone IMU and the UR TCP.

All the poses are saved in the format of (tx, ty, tz, rx, ry, rz).
