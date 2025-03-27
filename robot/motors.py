# motors.py

from machine import Pin, PWM
from parameters import FULL_SPD, TURN_SPD

# setup pins connected to L298N Motor Drive Controller Board
ena = PWM(Pin(21))
in1 = Pin(20, Pin.OUT, value=0)
in2 = Pin(19, Pin.OUT, value=0)
in3 = Pin(18, Pin.OUT, value=0)
in4 = Pin(17, Pin.OUT, value=0)
enb = PWM(Pin(16))

ena.freq(1_000)
enb.freq(1_000)

def set_mtr_dirs(a_mode, b_mode):
    """Set motor direction pins for both motors.
    options are: 'FWD', 'REV', 'OFF'."""

    if a_mode == 'FWD':
        in1.value(0)
        in2.value(1)
    elif a_mode == 'REV':
        in1.value(1)
        in2.value(0)
    else:  # Parked
        in1.value(0)
        in2.value(0)

    if b_mode == 'FWD':
        in3.value(0)
        in4.value(1)
    elif b_mode == 'REV':
        in3.value(1)
        in4.value(0)
    else:  # Parked
        in3.value(0)
        in4.value(0)

def set_mtr_spds(a_PWM_val, b_PWM_val):
    """set speeds for both a and b motors
    allowable values are u16 integers (< 65_536)"""
    a_val = int(a_PWM_val)
    if a_val > 65_530:
        a_val = 65_530
    b_val  = int(b_PWM_val)
    if b_val > 65_530:
        b_val = 65_530
    ena.duty_u16(a_val)
    enb.duty_u16(b_val)

def move_stop():
    set_mtr_dirs('OFF', 'OFF')
    set_mtr_spds(0, 0)

# Stop the robot NOW
move_stop()

def drive_motors(lin_spd, ang_spd):
    """
    Based on robot's desired motion in 2 DOF:
    linear speed: lin_spd (in range -1 to +1)
    angular speed: ang_spd (in range -1 to +1)
    Calculate both motor speeds and
    drive motors accordingly.
    """
    
    # linear components
    a_lin_spd = int(FULL_SPD * lin_spd)
    b_lin_spd = int(FULL_SPD * lin_spd)
    
    # turning components
    a_ang_spd = int(TURN_SPD * ang_spd)
    b_ang_spd = int(TURN_SPD * ang_spd)
    
    # superimpose components
    a_spd = a_lin_spd - a_ang_spd
    b_spd = b_lin_spd + b_ang_spd
    
    # determine direction of each motor
    if a_spd > 0:
        a_dir = 'FWD'
    elif a_spd < 0:
        a_dir = 'REV'
    else:
        a_dir = 'OFF'

    if b_spd > 0:
        b_dir = 'FWD'
    elif b_spd < 0:
        b_dir = 'REV'
    else:
        b_dir = 'OFF'

    # set motor direction pins
    set_mtr_dirs(a_dir, b_dir)
    
    # set motor speeds
    set_mtr_spds(abs(a_spd), abs(b_spd))