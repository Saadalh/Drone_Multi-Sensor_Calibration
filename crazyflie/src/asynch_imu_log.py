# import standard libraries
import logging
import time
import os

# import required cflib connection-libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# import required cflib logging-libraries
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

uri = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')
dir_path = os.path.realpath(os.path.dirname(__file__))
logfile = f"{dir_path}/../imu_logs/imu_log.txt"
logging.basicConfig(level=logging.ERROR)

class logging:

    def __init__(self, logfile, scf, logconf):
        self.fstream = open(logfile, "w")
        self.cf = scf.cf
        self.logconf = logconf
        self.cf.log.add_config(self.logconf)
        self.logconf.data_received_cb.add_callback(self.log_stab_callback)

    def log_stab_callback(self, timestamp, data, logconf):
        line = f"[{time.time()}][{timestamp}][{logconf.name}]: {data}\n"
        #print('[%d][%d][%s]: %s' % (time.time(), timestamp, logconf.name, data))
        self.fstream.write(line)

    def start_async_log(self):
        self.logconf.start()

    def stop_async_log(self):
        self.logconf.stop()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
    lg_stab.add_variable('stateEstimateZ.z', 'int16_t')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        logger = logging(logfile, scf, lg_stab)
        logger.start_async_log()
        time.sleep(5)
        logger.stop_async_log()

