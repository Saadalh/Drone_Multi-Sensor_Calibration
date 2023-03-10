import ur_control.src.ur_control as urc
import crazyflie.src.capture as cap
import crazyflie.src.asynch_imu_log as imu
import cflib.crtp
from cflib.utils import uri_helper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
import logging
from cflib.crazyflie.log import LogConfig
import time
import os
import csv
import argparse

if __name__ == "__main__":

    # Args for setting IP/port of AI-deck. Default settings are for when
    # AI-deck is in AP mode.
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="dip", help="AI-deck IP")
    parser.add_argument("-r", default="172.31.1.200", metavar="rip", help="Robot IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    parser.add_argument("-u", type=str, default='radio://0/100/2M/E7E7E7E701', metavar="uri", help="Radio-AP URI")
    parser.add_argument('--unsave', action='store_false', help="Dont save streamed images")
    args = parser.parse_args()

    # Define robot-related parameters
    rob_ip = args.r
    v = 0.15
    a = 0.1

    # Create UR control object
    ur = urc.urControl(rob_ip, v, a)

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Radio, WiFi connection, and Path variables
    deck_port = args.p
    deck_ip = args.n
    dir_path = os.path.realpath(os.path.dirname(__file__))
    logfile = f"{dir_path}/imu_log.txt"
    uri_add = args.u
    uri = uri_helper.uri_from_env(default=uri_add)

    # Connect to crazyflie and initialize the client socket
    cs = cap.connect_wifi(deck_ip, deck_port)
    count = 0

    # Initialize log parameters
    logging.basicConfig(level=logging.ERROR)
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')

    # Move to home pose where the calibration object needs to be place
    ur.move_home()
    input("Place the ChAruCo board under the drone. Then press Enter.")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logger = imu.logging(logfile, scf, lg_stab)
        logger.start_async_log()

        stations = 7 # Number of stations of capture, imu, and aruco pose data collection
        timestamps = []
        for i in range(0, stations):
            # No need for IMU values when moving from home to first capture pose
            if i == 0:
                ur.move_random()
                cap.capture(time.time(), count, cs, dir_path)
            # Timestamp array to save the start and end timestamps in it
            timestamp = []
            # Move to a random position
            timestamp.append(time.time())
            ur.move_random()
            timestamp.append(time.time())
            cap.capture(time.time(), count, cs, dir_path)
            timestamps.append(timestamp)

        logger.stop_async_log()

        with open("imu_timestamps.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writer(timestamps)





