"""
import pyfirmata
import time

board = pyfirmata.Arduino('COM14')

while True:
    board.digital[13].write(1)
    time.sleep(0.1)
    board.digital[13].write(0)
    time.sleep(0.1)
"""
import time
import board
import busio
import adafruit_lsm6ds.lsm6ds33
from pyfirmata import Arduino, util

# set up the serial connection and the Firmata protocol
board = Arduino('COM14', baudrate=9600)
it = util.Iterator(board)
it.start()

# create the I2C object and the LSM6DS33 sensor object
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm6ds.lsm6ds33.LSM6DS33(i2c)

# main loop
while True:
    x, y, z = sensor.acceleration
    print(f"Acceleration: ({x:.2f}, {y:.2f}, {z:.2f}) m/s^2")
    time.sleep(0.1)

