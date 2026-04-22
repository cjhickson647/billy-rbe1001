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
ai_vision_16 = AiVision(Ports.PORT16)
inertial_5 = Inertial(Ports.PORT5)
claw = Motor(Ports.PORT19, GearSetting.RATIO_18_1, True)
left_motor_motor_a = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
left_motor_motor_b = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
elevator = MotorGroup(left_motor_motor_a, left_motor_motor_b)
left_motor_1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_motor_1 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
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
left_motor_1.set_stopping(HOLD)
left_motor_2.set_stopping(HOLD)
right_motor_1.set_stopping(HOLD)
right_motor_2.set_stopping(HOLD)

# Reflectance
## TODO: Add a reflectance sensor (Linetracker) with the Device Manager

# Rangefinder
## TODO: Add an ultrasonic rangefinder (Rangefinder) with the Device Manager
inertial_5.calibrate()
wait(2, SECONDS)
inertial_5.set_rotation(0, DEGREES)

"""
Pro-tip: print out state _transistions_.
"""
GEAR_RATIO = 5/3
WHEEL_DIAM = 10.4775 # cm
CIRCUMFERENCE = math.pi * WHEEL_DIAM
WHEEL_TRACK = 22.86
target_turn_degrees = 90.0
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

# def handleRight1Button():
#     left_motor.spin_for(FORWARD, turn_rotations, TURNS, wait=False) 
#     right_motor.spin_for(REVERSE, turn_rotations, TURNS, wait=False)

# def handleLeft1Button():
#     global current_state
#     print('Left 1 Button Pressed')

#     if(current_state == IDLE):
#         print('IDLE -> FORWARD')
#         current_state = DRIVING_FWD

#         # Note how we set the motor to drive here, just once. 
#         # No need to call over and over and over in a loop.
#         # Also, note that we call the non-blocking version so we can
#         # return to the main loop.

#         ## TODO: You'll need to update the speed and number of turns
#          # L = motor rotation turns / (5) * pi * 4.125
#          # speed = 10cm * 60 / pi * 4.125) ( 5)
#         # 1 meter
#         drive_for(FORWARD, motor_rotations, motor_rpm)
#         #drive_for(FORWARD, 5, 60)
        
#     else: # in any other state, the button acts as a kill switch
#         print(' -> IDLE')
#         current_state = IDLE
#         left_motor.stop()
#         right_motor.stop()

"""
Pro-tip: print out state _transistions_.
"""

# Here, we give an example of a proper event checker. It checks for the _event_ 
# of stopping (not just if the robot is stopped).
wasMoving = False
# def checkMotionComplete():
#     global wasMoving

#     retVal = False

#     isMoving = left_motor.is_spinning() or right_motor.is_spinning()

#     if(wasMoving and not isMoving):
#         retVal = True

#     wasMoving = isMoving
#     return retVal

# # Then we declare a handler for the completion of the motion.
# def handleMotionComplete():
#     global current_state

#     if(current_state == DRIVING_FWD):
#         print('FORWARD -> BACKWARD')
#         current_state = DRIVING_BKWD

#          ## TODO: You'll need to update the speed and number of turns       
#         #drive_for(REVERSE, 5, 60)
#         #drive_for(REVERSE, 500/(math.pi*10.4775), (10 * 60 / math.pi * 10.4775) * 5)
    
#     elif(current_state == DRIVING_BKWD):
#         print('BACKWARD -> IDLE')
#         current_state = IDLE

#     else:
#         print('E-stop') # Should print when button is used as E-stop

"""
The line below makes use of VEX's built-in event management. Basically, we set up a "callback", 
basically, a function that gets called whenever the button is pressed (there's a corresponding
one for released). Whenever the button is pressed, the handleButton function will get called,
_without you having to do anything else_.

"""

#controller_1.buttonR1.pressed(handleRight1Button)

## TODO: Add event callback for bumper
"""
Note that the main loop only checks for the completed motion. The button press is handled by 
the VEX event system.
"""
timer = Timer()
def pidDrive(distance):
    timer.clear()
    # set gain constants
    kP = 5
    kI = 0.00
    kD = 31.425
    kP_steer = 1.74; #1.2157

    # reset motors
    left_motor_1.set_position(0, DEGREES)
    left_motor_2.set_position(0, DEGREES)
    right_motor_1.set_position(0, DEGREES)
    right_motor_1.set_position(0, DEGREES)

    # convert distance in cm to degrees to be rotated
    motor_rotations = (distance / CIRCUMFERENCE) * GEAR_RATIO
    desiredDistance = motor_rotations * 360

    # set initial values for each of the terms
    currentHeading = inertial_5.rotation(DEGREES)
    CM_PER_DEGREE = CIRCUMFERENCE / (360 * GEAR_RATIO)
    error_cm = 0
    error = 0
    integral = 0
    derivative = 0
    prevError = 0

    motorPower = 0
    prevMotorPower = 0

    # main PID loop
    while True:
        # get current distance
        print("{:.2f}, {:.2f}".format(timer.time(SECONDS), error_cm))
        currentDistance = (left_motor_1.position() + left_motor_2.position() + right_motor_1.position() + right_motor_2.position()) / 4
        cdCM = ((left_motor_1.position(DEGREES) * CM_PER_DEGREE) + (left_motor_2.position(DEGREES) * CM_PER_DEGREE) + (right_motor_1.position(DEGREES) * CM_PER_DEGREE) + (right_motor_2.position(DEGREES) * CM_PER_DEGREE)) / 4
        error_cm = distance - cdCM
        # calculate error and add accumulated error
        error = desiredDistance - currentDistance
        if (error < 200 and error > -200):
            integral += error
        
        # calculate derivative term and motor power
        derivative = error - prevError;
        motorPower = (kP * error) + (kI * integral) + (kD * derivative)

        # calculate heading correction
        currentAngle = inertial_5.rotation(DEGREES)
        headingError = currentHeading - currentAngle
        headingCorrect = headingError * kP_steer

        # normalize motor power between 1 and -1
        if (motorPower > 1):
            motorPower = 1
        if (motorPower < -1):
            motorPower = -1

        # add slew to prevent jerky, rapid acceleration
        slewRate = 0.1
        if (motorPower > prevMotorPower + slewRate):
            motorPower = prevMotorPower + slewRate
        if (motorPower < prevMotorPower - slewRate):
            motorPower = prevMotorPower - slewRate

        # add heading correction to motor power
        left_raw = (11 * motorPower) - headingCorrect
        right_raw = (11 * motorPower) + headingCorrect
        
        # if the resulting motor power is above the motor limit, scale it down
        max_raw = max(abs(left_raw), abs(right_raw))
        if max_raw > 11:
            left_calc = (left_raw / max_raw) * 11
            right_calc = (right_raw / max_raw) * 11
        else:
            left_calc = left_raw
            right_calc = right_raw

        # spin motors using the calulated motor power
        left_motor_1.spin(FORWARD, left_calc, VOLT);
        left_motor_2.spin(FORWARD, left_calc, VOLT)
        right_motor_1.spin(FORWARD, right_calc, VOLT);
        right_motor_2.spin(FORWARD, right_calc, VOLT)

        # if the error is minimal, we have reached the target, exit
        if (error > -10 and error < 10 and error - prevError > -10 and error - prevError < 10):
            break;
        
        # update variables for next loop
        prevMotorPower = motorPower
        prevError = error
        wait(20, MSEC)
    
    # stop motors
    left_motor_1.stop()
    left_motor_2.stop()
    right_motor_1.stop()
    right_motor_2.stop()


def pidTurn(degrees):
    
    # set gain constants
    kP = 1.7
    kI = 0
    kD = 10

    # reset motors
    left_motor_1.set_position(0, DEGREES)
    left_motor_2.set_position(0, DEGREES)
    right_motor_1.set_position(0, DEGREES)
    right_motor_2.set_position(0, DEGREES)

    # set initial values for each of the terms
    error = 0
    integral = 0
    derivative = 0
    prevError = 0

    motorPower = 0
    prevMotorPower = 0

    while True:
        # get current rotation position
        brain.screen.print("3")
        currentRotation = inertial_5.rotation(DEGREES)
         # calculate error and add accumulated error
        print("{:.2f}, {:.2f}".format(timer.time(SECONDS), error))
        error = degrees - currentRotation
        if (error < 200 and error > -200):
            integral += error
        
        # calculate derivative term and motor power
        derivative = error - prevError;
        motorPower = (kP * error) + (kI * integral) + (kD * derivative)

        # normalize motor power between 1 and -1
        if (motorPower > 1):
            motorPower = 1
        if (motorPower < -1):
            motorPower = -1

        # add slew to prevent jerky, rapid acceleration
        slewRate = 0.1
        if (motorPower > prevMotorPower + slewRate):
            motorPower = prevMotorPower + slewRate
        if (motorPower < prevMotorPower - slewRate):
            motorPower = prevMotorPower - slewRate

        # spin motors using the calulated motor power
        # calculatedPower = motorPower * 11
        # if abs(error) > 1.0: # Only apply if we aren't in the deadband
        #     if calculatedPower > 0 and calculatedPower < 1.4:
        #         calculatedPower = 1.4 # The Voltage Floor

        left_motor_1.spin(FORWARD, -11 * motorPower, VOLT);
        left_motor_2.spin(FORWARD, -11 * motorPower, VOLT)
        right_motor_1.spin(FORWARD, 11 * motorPower, VOLT);
        right_motor_2.spin(FORWARD, 11 * motorPower, VOLT)

        # if the error is minimal, we have reached the target, exit
        if (error > -1 and error < 1 and error - prevError > -0.5 and error - prevError < 0.5):
            break;
        
        # update variables for next loop
        prevMotorPower = motorPower
        prevError = error
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(inertial_5.rotation(DEGREES))
        wait(20, MSEC)

    left_motor_1.stop()
    left_motor_2.stop()
    right_motor_1.stop()
    right_motor_2.stop()
    brain.screen.clear_screen()
        

controller_1.buttonL1.pressed(lambda: pidDrive(150))
controller_1.buttonR1.pressed(lambda: pidTurn(90))
controller_1.buttonR2.pressed(lambda: pidTurn(0))

# pidDrive(292)
# wait(20,MSEC)
# pidTurn(90)
# wait(20,MSEC)
# pidDrive(295)
# wait(20,MSEC)
# pidTurn(0)
# wait(20,MSEC)
# pidDrive(169.5)
# wait(20,MSEC)
# pidTurn(-90)
# wait(20,MSEC)
# pidDrive(283.7)
# wait(20,MSEC)
# pidTurn(0)
# wait(20,MSEC)
# pidDrive(-440)

# while True:
#     controller_1.screen.print(inertial_5.rotation(DEGREES))
#     controller_1.screen.clear_screen()
#     controller_1.screen.set_cursor(1,1)
    

# --------------------------------------------------------------------------------------------------------------------------
# --------------------------------------------------------------------------------------------------------------------------

## Everything is event-driven through the event library...no code in the main loop!