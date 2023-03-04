import capture
import logging
import argparse
import asynch_imu_log as log
import time
import os
import socket
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
import cflib.crtp

# Args for setting IP/port of AI-deck. Default settings are for when
# AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
parser.add_argument("-u", default='radio://0/100/2M/E7E7E7E701', metavar="uri", help="Radio URI")
args = parser.parse_args()

# Radio, WiFi connection, and Path variables
uri = uri_helper.uri_from_env(default=args.u)
logging.basicConfig(level=logging.ERROR)
deck_port = args.p
deck_ip = args.n
dir_path = os.path.realpath(os.path.dirname(__file__))
logfile = f"{dir_path}/../imu_logs/imu_log.txt"

if __name__ == '__main__':
    # Initialize the low-level cflib drivers
    cflib.crtp.init_drivers()

    scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))
    client_socket = capture.connect_wifi(deck_ip, deck_port)

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        logger = log.logging(logfile, scf, lg_stab)
        logger.start_async_log()

        for i in range(0,3):
            print(f"Robot is moving to position {i+1}...")
            time.sleep(2)
            print(f"Robot is at position {i+1}, capturing picture...")
            time.sleep(1)
            start = time.time()
            cap_num = capture.capture(start, i, client_socket)
        
        logger.stop_async_log()

    
    