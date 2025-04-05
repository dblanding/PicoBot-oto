# main.py
"""
MicroPython code for PicoBot project
* Raspberry Pi Pico mounted on differential drive robot
* 56:1 gear motors
* 2 BLE UART Friend modules
* Tele-op driving commands come in on uart0
* Robot data sent out to laptop on uart1
* 2 VL53L0X tof distance sensors on I2C0: Left, Right
* 1 VL53L0X tof distance sensor on I2C1: Forward
* SparkFun OTOS on I2C1 reports pose (x, y, heading)
    * initially (0, 0, 0)
    * x & y units: meters
    * heading units: radians, Zero along X-axis, pos CCW, neg CW
    * yaw_rate (rad/sec) also used
* distance sensor units: mm (gets converted to meters in mapper)
"""

import asyncio
import gc
import json
from machine import I2C, Pin, UART
from math import pi, sqrt
import motors
from parameters import JS_GAIN, MIN_DIST, ANGLE_TOL, SWATH_PITCH
from parameters import P_TURN_GAIN, D_TURN_GAIN, MAX_ANG_SPD
import qwiic_i2c
import qwiic_otos
import struct
import time
import VL53L0X
import arena

D_GAIN = 0.286  # Gain of Derivative feedback term

# set up uart0 for communication with BLE UART friend
print("setting up uart0 for accepting tele-op joystick commands")
uart0 = UART(0, 9600)
uart0.init(bits=8, parity=None, stop=1, timeout=10)

# set up uart1 for communication with BLE UART friend
print("Setting up uart1 for sending robot data to laptop")
uart1 = UART(1, 9600)
uart1.init(tx=Pin(4), rx=Pin(5), bits=8, parity=None, stop=1, timeout=10)

# setup onboard LED
led = machine.Pin("LED", machine.Pin.OUT)

# Initialize I2C1 using qwiic library
i2c1 = qwiic_i2c.get_i2c_driver(sda=14, scl=15, freq=100000)

# set up OTOS on i2c1
myOtos = qwiic_otos.QwiicOTOS(23, i2c1)

# set up multiplexer on i2c0
i2c0 = I2C(0, sda=Pin(12), scl=Pin(13))
print("Device Addresses found on I2C0: ",i2c0.scan())

# set up forward looking dist sensor on i2c1
print("Device Addresses found on I2C1: ",i2c1.scan())
i2c1 = I2C(1, sda=Pin(14), scl=Pin(15), freq=100000, timeout=200000 )
tof1 = VL53L0X.VL53L0X(i2c1)
tof1.start()

# Initialize I2C1 using qwiic library
my_bus = qwiic_i2c.get_i2c_driver(sda=14, scl=15, freq=100000)

# Set up and initialize Optical Tracking Odomety Sensor (OTOS) device
myOtos = qwiic_otos.QwiicOTOS(23, my_bus)
print("\nSetting up OTOS\n")

# Perform the self test
result = myOtos.selfTest()
    
# Check if the self test passed
if(result == True):
    print("Self test passed!")
else:
    print("Self test failed!")

# Check if it's connected
if not myOtos.is_connected():
    print("The OTOS isn't connected", file=sys.stderr)

myOtos.begin()

print("Ensure the OTOS is flat and stationary during calibration!")
for i in range(5, 0, -1):
    print("Calibrating in %d seconds..." % i)
    time.sleep(1)

print("Calibrating IMU...")

# Calibrate the IMU, which removes the accelerometer and gyroscope offsets
myOtos.calibrateImu()

# Account for OTOS location w/r/t robot center
offset = qwiic_otos.Pose2D(1.25, 0, 0)
myOtos.setOffset(offset)

# Set units for linear and angular measurements.
# If not set, the default is inches and degrees.
# Note that this setting is not stored in the sensor.
# it's part of the library, so you need to set it.
myOtos.setLinearUnit(myOtos.kLinearUnitMeters)
myOtos.setAngularUnit(myOtos.kAngularUnitRadians)

# Reset the tracking algorithm - this resets the position to the origin,
# but can also be used to recover from some rare tracking errors
myOtos.resetTracking()

print("OTOS initialized")

def get_dist(channel):
    """
    return dist (mm) from tof sensor on mux
        channel = b'\x02'  # left sensor on ch 1
        channel = b'\x04'  # right sensor on ch 2
    see: https://github.com/mcauser/micropython-tca9548a
    """
    i2c0.writeto(0x70, channel)
    tof = VL53L0X.VL53L0X(i2c0)
    tof.start()
    dist = tof.read()
    tof.stop()
    return dist

def get_pose():
    pose = myOtos.getPosition()
    return (pose.x, pose.y, pose.h)

def bot_moving(pose0, pose1):
    """Detect if bot has moved more than MIN_DIST from pose0 to pose1"""
    x0, y0, h0 = pose0
    x1, y1, h1 = pose1
    d = sqrt((x0 - x1)**2 + (y0 - y1)**2)
    return d > MIN_DIST

def check_pose(prev_pose):
    curr_pose = get_pose()
    is_moving = bot_moving(prev_pose, curr_pose)
    return (is_moving, curr_pose)

def get_yaw_rate():
    vel = myOtos.getVelocity()
    return vel.h

if __name__ == "__main__":
    while True:
        x, y, h = get_pose()
        print(h*180/pi)
        time.sleep(1)
    sleep
    