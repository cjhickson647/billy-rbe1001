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
ai_vision_18 = AiVision(Ports.PORT18)
inertial_5 = Inertial(Ports.PORT5)
motor_8 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
left_motor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
bumper_g = Bumper(brain.three_wire_port.g)
line_tracker_a = Line(brain.three_wire_port.a)
range_finder_c = Sonar(brain.three_wire_port.c)


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
            if controller_1.buttonR1.pressing():
                motor_8.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonR2.pressing():
                motor_8.spin(REVERSE)
                controller_1_right_shoulder_control_motors_stopped = False
            elif not controller_1_right_shoulder_control_motors_stopped:
                motor_8.stop()
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
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *

# Begin project code
# define the states
IDLE = 0
DRIVING_FWD = 1
DRIVING_BKWD = 2

# start out in the idle state
current_state = IDLE

# Bumper
## TODO: Add a Bumper with the Device Manager

# Reflectance
## TODO: Add a reflectance sensor (Linetracker) with the Device Manager

# Rangefinder
## TODO: Add an ultrasonic rangefinder (Rangefinder) with the Device Manager

def drive_for(direction, turns, speed):
    # L = motor rotation turns / (5) * pi * 4.125
    # speed = 10cm * 60 / pi * 4.125) ( 5)
    # 1 meter
    left_motor.set_velocity(speed, RPM);
    left_motor.spin_for(direction, turns, TURNS, wait = False)

    right_motor.set_velocity(speed, RPM);
    right_motor.spin_for(direction, turns, TURNS, wait = False)

"""
Pro-tip: print out state _transistions_.
"""
GEAR_RATIO = 5.0
WHEEL_DIAM = 10.4775 # cm
CIRCUMFERENCE = math.pi * WHEEL_DIAM
WHEEL_TRACK = 31.75
target_turn_degrees = 90.0
inertial_5.calibrate()

# Targets
target_dist = 104 # 1 meter in cm, 56 for dead reckoning
target_speed_cms = 20.0 # cm/s

# Conversions
# Rotations = (Distance / Circumference) * Gear Ratio
motor_rotations = (target_dist / CIRCUMFERENCE) * GEAR_RATIO

# Velocity = (Speed / Circumference) * 60 seconds * Gear Ratio
motor_rpm = (target_speed_cms / CIRCUMFERENCE) * 60 * GEAR_RATIO

# Calculation
# (Wheel Track / Wheel Diam) * (Turn Degrees / 360) * Gear Ratio
turn_rotations = (WHEEL_TRACK / WHEEL_DIAM) * (target_turn_degrees / 360.0) * GEAR_RATIO
# left_motor.set_velocity(30, RPM); #OG 30
# right_motor.set_velocity(30, RPM);

def handleRight1Button():
    left_motor.spin_for(FORWARD, turn_rotations, TURNS, wait=False) 
    right_motor.spin_for(REVERSE, turn_rotations, TURNS, wait=False)

def handleLeft1Button():
    global current_state
    print('Left 1 Button Pressed')

    if(current_state == IDLE):
        print('IDLE -> FORWARD')
        current_state = DRIVING_FWD

        # Note how we set the motor to drive here, just once. 
        # No need to call over and over and over in a loop.
        # Also, note that we call the non-blocking version so we can
        # return to the main loop.

        ## TODO: You'll need to update the speed and number of turns
         # L = motor rotation turns / (5) * pi * 4.125
         # speed = 10cm * 60 / pi * 4.125) ( 5)
        # 1 meter
        drive_for(FORWARD, motor_rotations, motor_rpm)
        #drive_for(FORWARD, 5, 60)
        
    else: # in any other state, the button acts as a kill switch
        print(' -> IDLE')
        current_state = IDLE
        left_motor.stop()
        right_motor.stop()

"""
Pro-tip: print out state _transistions_.
"""
def handleBumperG():
    global current_state
    if bumper_g.pressing() == True:
        drive_for(REVERSE, motor_rotations, motor_rpm)
    else: 
        pass
    ## Todo: Add code to handle the bumper being presses


# Here, we give an example of a proper event checker. It checks for the _event_ 
# of stopping (not just if the robot is stopped).
wasMoving = False
def checkMotionComplete():
    global wasMoving

    retVal = False

    isMoving = left_motor.is_spinning() or right_motor.is_spinning()

    if(wasMoving and not isMoving):
        retVal = True

    wasMoving = isMoving
    return retVal

# Then we declare a handler for the completion of the motion.
def handleMotionComplete():
    global current_state

    if(current_state == DRIVING_FWD):
        print('FORWARD -> BACKWARD')
        current_state = DRIVING_BKWD

         ## TODO: You'll need to update the speed and number of turns       
        #drive_for(REVERSE, 5, 60)
        #drive_for(REVERSE, 500/(math.pi*10.4775), (10 * 60 / math.pi * 10.4775) * 5)
    
    elif(current_state == DRIVING_BKWD):
        print('BACKWARD -> IDLE')
        current_state = IDLE

    else:
        print('E-stop') # Should print when button is used as E-stop


## TODO: Add a checker for the reflectance sensor
## See checkMotionComplete() for a good example

## TODO: Add a handler for when the reflectance sensor triggers
def HandleReflectanceA():
    # brain.screen.print(line_tracker_a.reflectivity(PERCENT))
    if line_tracker_a.reflectivity(PERCENT) > 50:
        drive_for(REVERSE, motor_rotations, motor_rpm)
    else:
        pass

def UltrasonicSensorCD():
    brain.screen.print(range_finder_c.distance(MM))
    wait(1,SECONDS)
    brain.screen.clear_row(1)
    brain.screen.set_cursor(1, 1)


#Dead Reckoning:
def dead_Reckoning():
    motor_8.set_stopping(HOLD)
    motor_8.set_velocity(67, PERCENT)
    motor_8.set_position(0,DEGREES)
    drive_for(REVERSE, motor_rotations, motor_rpm)
    motor_8.spin_to_position(990, DEGREES, wait=False)
    wait(6,SECONDS)
    #if current_state == IDLE:
    motor_8.spin_to_position(770, DEGREES)


def sonar():
    inertial_5.set_heading(0, DEGREES)
    inertial_5.set_rotation(0,DEGREES)
    target_heading = 0.0
    speed = 50

    motor_8.set_stopping(HOLD)
    motor_8.set_velocity(speed, PERCENT)
    left_motor.set_velocity(speed, PERCENT)
    right_motor.set_velocity(speed, PERCENT)
    motor_8.set_position(0,DEGREES)
    motor_8.spin_to_position(990, DEGREES, wait=False)
    

    while range_finder_c.distance(MM) > 75:
        # current_heading = inertial_5.rotation(DEGREES)
        # brain.screen.print(current_heading)
        # error = target_heading - current_heading

        # kP = 0.2
        # correction = error * kP
        
        # left_motor.set_velocity(speed - correction, PERCENT)
        # right_motor.set_velocity(speed + correction, PERCENT)


        left_motor.spin(REVERSE)
        right_motor.spin(REVERSE)
        
        wait(20,MSEC)
        brain.screen.clear_row(1)
        brain.screen.set_cursor(1, 1)

    left_motor.stop()
    right_motor.stop()
    motor_8.spin_to_position(770, DEGREES)
    controller_1.screen.print(motor_8.torque(TorqueUnits.NM))

"""
The line below makes use of VEX's built-in event management. Basically, we set up a "callback", 
basically, a function that gets called whenever the button is pressed (there's a corresponding
one for released). Whenever the button is pressed, the handleButton function will get called,
_without you having to do anything else_.

"""
controller_1.buttonL1.pressed(handleLeft1Button)
controller_1.buttonR1.pressed(handleRight1Button)
controller_1.buttonA.pressed(dead_Reckoning)

## TODO: Add event callback for bumper
"""
Note that the main loop only checks for the completed motion. The button press is handled by 
the VEX event system.
"""

# --------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------

## A basic line following program that uses checker/handlers
## See lecture notes for the state diagram

ROBOT_IDLE = 0
ROBOT_LINING = 1

robotState = ROBOT_IDLE

# Controller
controller = Controller()

left_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)

left_sensor = Line(brain.three_wire_port.b)
right_sensor = Line(brain.three_wire_port.a)

Kp = 0.3 ## TODO: Pick a Kp to start; then adjust to get good performance

## Line timer handler. Note that we check the state and act accordingly
def handleLineTimer():
    if(robotState == ROBOT_LINING):
        right_reflectivity = right_sensor.reflectivity()
        left_reflectivity = left_sensor.reflectivity()

        print(left_reflectivity, right_reflectivity)

        # TODO: Define the error
        line_error = 0

        # TODO: Calculate the effort from the error
        turning_effort = 0
        
        # TODO: Find the base speed to go 20 cm/sec
        # We'll add and subtract from the wheels to keep the average speed the same
        base_speed = 0
        
        # TODO: Control the motor speeds as a combination of base_speed and turning effort
        # Depending on your definition of error, you will need +/- for each term
        left_motor.spin(FORWARD, base_speed + turning_effort, RPM)
        right_motor.spin(FORWARD, base_speed - turning_effort, RPM)

    ## Don't forget to restart the timer!
    lineTimer.event(handleLineTimer, 50)

## The line timer will tell us when to correct the heading
lineTimer = Timer()

## This uses the VEX event machinery, 'automatic' checker-handler
## It has the same functionality as "if check timer expired -> handle timer expired"
## Maybe adust the timer interval
lineTimer.event(handleLineTimer, 50)

## Button handler. Note that we check the state and then act accordingly
def handleLeft1Button_2():
    global robotState
    print("Button L1")
    if(robotState == ROBOT_IDLE):
        robotState = ROBOT_LINING        
    elif(robotState == ROBOT_LINING):
        robotState = ROBOT_IDLE    
        left_motor.stop()
        right_motor.stop()    

## Same as "if check button press -> handle button press"
controller.buttonL1.pressed(handleLeft1Button_2)

## Everything is event-driven through the event library...no code in the main loop!
while True:
    pass

# --------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------

# The main loop
while True:
    pass
    # if(checkMotionComplete()): handleMotionComplete()
    # controller_1.screen.print(motor_8.torque(TorqueUnits.NM))
    # controller_1.screen.clear_row(1)
    # controller_1.screen.set_cursor(1,1)
    # wait(0.2,SECONDS)
    
    #controller_1.screen.print(motor_8.torque(TorqueUnits.NM))
    #controller_1.screen.clear_row(1)
    #controller_1.screen.set_cursor(1,1)
    #wait(0.2,SECONDS)
    #handleBumperG()
    #HandleReflectanceA()
    #wait(1,SECONDS)
    #UltrasonicSensorCD()
    
    # Lab 2 ------------------------------------------------------------------
    # ------------------------------------------------------------------------
    
