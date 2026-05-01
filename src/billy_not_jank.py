#region VEXcode Generated Robot Configuration
from vex import *
import urandom # type: ignore
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
# AI Vision Color Descriptions
# AI Vision Code Descriptions
fruit_vision = AiVision(Ports.PORT16)
april_vision = AiVision(Ports.PORT14)
imu = Inertial(Ports.PORT6)
claw = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
elevate_a = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
elevate_b = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
elevator = MotorGroup(elevate_a, elevate_b)
left_motor_1 = Motor(Ports.PORT21, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_motor_1 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
button = Bumper(brain.three_wire_port.a)
line_tracker_h = Line(brain.three_wire_port.h)
line_tracker_f = Line(brain.three_wire_port.f)


# wait for rotation sensor to fully initialize
wait(30, MSEC)

# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()

def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs
controller_1_right_shoulder_control_motors_stopped = True

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            # check the buttonR1/buttonR2 status
            # to control motor_8
            if controller_1.buttonY.pressing():
                claw.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonX.pressing():
                claw.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            elif not controller_1_right_shoulder_control_motors_stopped:
                claw.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_right_shoulder_control_motors_stopped = True
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
remote_control_code_enabled = True
rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      Billy NOT Jank
#	Author:       Conner, Parker, Prince, Izzy
#	Created:      05/01/2026
#	Description:  Billy NOT Jank attempts to harvest
#                 and deliver fruit for RBE 1001
# 
# ------------------------------------------

# Library imports
from vex import *

IDLE = 0
RAMP_DRIVE = 1
SEARCHING = 2
APPROACHING = 3
HARVESTING = 4
DELIVERING = 5
HARVEST_RESET = 6
DOING_YA_MOM = 67

ROBOT_STATE = IDLE

left_motor_1.set_stopping(HOLD)
left_motor_2.set_stopping(HOLD)
right_motor_1.set_stopping(HOLD)
right_motor_2.set_stopping(HOLD)

imu.calibrate()
wait(2, SECONDS)
imu.set_rotation(0, DEGREES)

GEAR_RATIO = 5/3
WHEEL_DIAM = 10.4775
CIRCUMFERENCE = math.pi * WHEEL_DIAM
WHEEL_TRACK = 22.86

