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
from math import pi, sqrt, atan2, sin, cos
import motors
from parameters import JS_GAIN, MIN_DIST, ANGLE_TOL, SWATH_PITCH
from parameters import P_TURN_GAIN, D_TURN_GAIN, MAX_ANG_SPD
import qwiic_i2c
import qwiic_otos
import struct
import time
import VL53L0X

D_GAIN = 0.5  # Gain of Derivative feedback term

waypoints_file = "waypoints.txt"

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
tof_1 = VL53L0X.VL53L0X(i2c1)
tof_1.start()

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

def read_waypoints(wp_file):
    """Load waypoints from file"""
    waypoints = []
    with open(wp_file) as f:
        lines = f.readlines()
        for line in lines:
            if ',' in line:
                str_x, str_y = line.split(',')
                wp = float(str_x), float(str_y)
                waypoints.append(wp)
    return waypoints

def set_up_snsr(sensor):
    """Return tof"""
    i2c0.writeto(0x70, sensor)
    tof = VL53L0X.VL53L0X(i2c0)
    tof.start()
    return tof

# set up 2 sensors on mux
# see: https://github.com/mcauser/micropython-tca9548a
ch1 = b'\x02'  # left sensor on ch 1
tof1 = set_up_snsr(ch1)
ch2 = b'\x04'  # right sensor on ch 2
tof2 = set_up_snsr(ch2)

def get_dist(tof, channel):
    i2c0.writeto(0x70, channel)
    dist = tof.read()
    tof.stop()
    i2c0.writeto(0x70, b'\x00')  # clear channels
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

def send_json(data):
    uart1.write((json.dumps(data) + "\n").encode())

def read_json():
    try:
        data = uart1.readline()
        decoded = data.decode()
        return json.loads(decoded)
    except (UnicodeError, ValueError):
        print("Invalid data")
        return None

# geometry helper functions
def p2r(r, theta):
    """Convert polar coords to rectangular"""
    x = cos(theta) * r
    y = sin(theta) * r
    return (x, y)

def r2p(x, y):
    """Convert rectangular coords to polar"""
    r = sqrt(x*x + y*y)
    theta = atan2(y, x)
    return (r, theta)

def rel_polar_coords_to_pt(curr_pose, point):
    """Based on current pose, return relative
    polar coords dist (m), angle (rad) to goal point.
    """
    # current pose
    x0, y0, a0 = curr_pose

    # coords of goal point
    x1, y1 = point

    # Relative coords to goal point
    x = x1 - x0
    y = y1 - y0

    # Convert rectangular coords to polar
    r, theta = r2p(x, y)

    # Relative angle to goal point
    rel_angle = theta - a0

    # ensure angle is between -pi/2 and +pi/2
    if rel_angle < -pi:
        rel_angle += 2 * pi
    elif rel_angle > pi:
        rel_angle -= 2 * pi

    return (r, rel_angle)

# read waypoints file
waypoints = read_waypoints(waypoints_file)

class Robot():
    def __init__(self):

        # set up some starting values
        self.lin_spd = 0.7  # nominal drive speed
        self.ang_spd = 0  # prev value ang_spd only when stuck
        self.run = True
        self.mode = 'W'  # 'T' for tele-op, 'W' for auto (waypoints) 
        self.errors = []
        self.prev_pose = (0, 0, 0)
        self.wp_idx = 0  # index of first waypoint

    def auto(self):
        """Set mode to drive auto follow waypoints."""
        self.mode = 'W'

    def tele(self):
        """Set mode to drive by tele-operation."""
        self.mode = 'T'

    def turn(self, goal_angle, gz, yaw):
        """
        Return ang_spd needed to drive motors when
        turning in place to goal_angle (radians).
        self.ang_spd is used to remember prev ang_spd when stuck
        """
        # calculate ang_spd to steer to goal_angle
        yaw_err = yaw - goal_angle
        p = -(yaw_err * P_TURN_GAIN)  # proportional term
        d = -(gz * D_TURN_GAIN)  # derivative term
        ang_spd = p + d

        # limit value of ang_spd
        if ang_spd < -MAX_ANG_SPD:
            ang_spd = -MAX_ANG_SPD
        if ang_spd > MAX_ANG_SPD:
            ang_spd = MAX_ANG_SPD

        # reset self.ang_spd to zero if not stuck
        if abs(gz) > 0.01:
            self.ang_spd = 0

        # check if turn is complete
        if abs(gz) < 0.01 and abs(yaw_err) < ANGLE_TOL:
            ang_spd = 0

        # give an extra boost if needed to overcome static friction
        elif abs(gz) < 0.01 and abs(yaw_err) < 3 * ANGLE_TOL:
            if not self.ang_spd:  # not previously boosted
                ang_spd *= 1.5
                self.ang_spd = ang_spd
            else:  # boost it further
                ang_spd = self.ang_spd * 1.5
                self.ang_spd = 0

        return ang_spd

    def stop(self):
        self.run = False
        motors.drive_motors(0, 0)

    async def main(self):
        try:
            while self.run:
                # read distances from VCSEL sensors
                dist_L = get_dist(tof1, ch1)
                dist_R = get_dist(tof2, ch2)
                dist_F = tof_1.read()

                # check current pose
                is_moving, pose = check_pose(self.prev_pose)
                self.prev_pose = pose
                
                # get yaw (radians) and yaw rate (rad/sec)
                _, _, yaw = pose
                gz = get_yaw_rate()

                # Drive in tele-op mode
                if self.mode == 'T':
                    if uart0.any() > 0:
                        try:
                            # get Bluetooth command
                            bytestring = uart0.readline()
                            data_type = bytestring[:2].decode()
                            bin_value = bytestring[2:14]
                            if data_type == '!A':  # accelerometer data
                                x, y, z = struct.unpack('3f', bin_value)
                                self.lin_spd = y * JS_GAIN
                                self.ang_spd = -x * JS_GAIN
                                print(self.lin_spd, self.ang_spd)
                                
                        except Exception as e:
                            self.errors.append(e)

                        # send commands to motors
                        motors.drive_motors(self.lin_spd, self.ang_spd)

                # Drive autonomously to sequence of waypoints
                elif self.mode == 'W':

                    wp = waypoints[self.wp_idx]
                    goal_dist, goal_angle = rel_polar_coords_to_pt(pose, wp)
                    if goal_dist > 0.2:
                        # drive to waypoint, steering to goal_angle
                        d = -(gz * D_GAIN)  # derivative term
                        ang_spd = goal_angle + d
                        motors.drive_motors(self.lin_spd, ang_spd)
                    else:
                        # arrived at waypoint
                        self.wp_idx += 1
                        if self.wp_idx == len(waypoints):
                            # No more waypoints
                            self.stop()

                # If robot is moving, send robot data to laptop
                if is_moving:
                    if self.mode == 'W':
                        send_json({
                            "pose": list(pose),
                            "dist_L": dist_L,
                            "dist_R": dist_R,
                            "dist_F": dist_F,
                            "errors": self.errors,
                            "goal_dist": goal_dist,
                            "goal_angle": goal_angle,
                            })
                    elif self.mode == 'T':
                        send_json({
                            "pose": list(pose),
                            "dist_L": dist_L,
                            "dist_R": dist_R,
                            "dist_F": dist_F,
                            "errors": self.errors,
                            })

                led.toggle()
                await asyncio.sleep(0.1)

        finally:
            motors.drive_motors(0, 0)


async def command_handler(robot):
    print("Starting handler")
    robot_task = None
    # robot_task = asyncio.create_task(robot.main())
    while True:
        if uart1.any():
            request = read_json()
            if not request:
                continue
            print("Received: ", request)
            if request["command"] == "arena":
                print("Received request for waypoints")
                print(waypoints)
                send_json({"waypoints": waypoints})
            elif request["command"] == "start":
                if not robot_task:
                    robot_task = asyncio.create_task(robot.main())
            elif request["command"] == "stop":
                robot.stop()
            elif request["command"] == "auto":
                robot.auto()
            elif request["command"] == "tele":
                robot.tele()

        await asyncio.sleep(0.1)


robot = Robot()
asyncio.run(command_handler(robot))
