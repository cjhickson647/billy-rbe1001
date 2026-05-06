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
# fruit__apricot = Colordesc(1, 233, 77, 81, 10, 0.19)
# fruit__apricot = Colordesc(1, 233, 112, 103, 15.67, 0.21)
fruit__apricot = Colordesc(1, 228, 64, 65, 15, 0.21)
# fruit__lime = Colordesc(2, 91, 255, 144, 9, 0.21)
fruit__lime = Colordesc(2, 53, 251, 108, 15, 0.23)
# fruit__grape = Colordesc(3, 146, 89, 195, 10, 0.2)
fruit__grape = Colordesc(3, 165, 119, 202, 40, 0.21)
fruit__tree = Colordesc(4, 121, 163, 160, 40, 0.1)
fruit_vision = AiVision(Ports.PORT16, fruit__apricot, fruit__lime, fruit__grape, fruit__tree)
april_vision = AiVision(Ports.PORT14, AiVision.ALL_TAGS)
imu = Inertial(Ports.PORT6)
claw = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
elevate_a = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
elevate_b = Motor(Ports.PORT18, GearSetting.RATIO_18_1, True)
elevator = MotorGroup(elevate_a, elevate_b)
flap = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
left_motor_1 = Motor(Ports.PORT21, GearSetting.RATIO_18_1, False)
left_motor_2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_motor_1 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_motor_2 = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
light = Motor(Ports.PORT7, GearSetting.RATIO_18_1, True)
button = Bumper(brain.three_wire_port.a)
line_tracker_left = Line(brain.three_wire_port.h)
line_tracker_right = Line(brain.three_wire_port.f)
ultrasonic = Sonar(brain.three_wire_port.c)
left_drive_smart = MotorGroup(left_motor_1, left_motor_2)
right_drive_smart = MotorGroup(right_motor_1, right_motor_2)
drivetrain = DriveTrain(left_drive_smart, right_drive_smart, 319.19, 295, 40, MM, 1.6666666666666667)


# wait for rotation sensor to fully initialize
wait(30, MSEC)

# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

# define variables used for controlling motors based on controller inputs

# define a task that will handle monitoring inputs from controller_1
controller_1_left_shoulder_control_motors_stopped = True
controller_1_right_shoulder_control_motors_stopped = True
drivetrain_l_needs_to_be_stopped_controller_1 = False
drivetrain_r_needs_to_be_stopped_controller_1 = False

remote_control_code_enabled = False

# define a task that will handle monitoring inputs from controller_1
def rc_auto_loop_function_controller_1():
    global drivetrain_l_needs_to_be_stopped_controller_1, drivetrain_r_needs_to_be_stopped_controller_1, controller_1_left_shoulder_control_motors_stopped, controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        if remote_control_code_enabled:
            
            # calculate the drivetrain motor velocities from the controller joystick axies
            # left = axis3 + axis1
            # right = axis3 - axis1
            drivetrain_left_side_speed = controller_1.axis3.position() + controller_1.axis1.position()
            drivetrain_right_side_speed = controller_1.axis3.position() - controller_1.axis1.position()
            
            # check if the value is inside of the deadband range
            if drivetrain_left_side_speed < 5 and drivetrain_left_side_speed > -5:
                # check if the left motor has already been stopped
                if drivetrain_l_needs_to_be_stopped_controller_1:
                    # stop the left drive motor
                    left_drive_smart.stop()
                    # tell the code that the left motor has been stopped
                    drivetrain_l_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the left motor next
                # time the input is in the deadband range
                drivetrain_l_needs_to_be_stopped_controller_1 = True
            # check if the value is inside of the deadband range
            if drivetrain_right_side_speed < 5 and drivetrain_right_side_speed > -5:
                # check if the right motor has already been stopped
                if drivetrain_r_needs_to_be_stopped_controller_1:
                    # stop the right drive motor
                    right_drive_smart.stop()
                    # tell the code that the right motor has been stopped
                    drivetrain_r_needs_to_be_stopped_controller_1 = False
            else:
                # reset the toggle so that the deadband code knows to stop the right motor next
                # time the input is in the deadband range
                drivetrain_r_needs_to_be_stopped_controller_1 = True
            
            # only tell the left drive motor to spin if the values are not in the deadband range
            if drivetrain_l_needs_to_be_stopped_controller_1:
                left_drive_smart.set_velocity(drivetrain_left_side_speed, PERCENT)
                left_drive_smart.spin(FORWARD)
            # only tell the right drive motor to spin if the values are not in the deadband range
            if drivetrain_r_needs_to_be_stopped_controller_1:
                right_drive_smart.set_velocity(drivetrain_right_side_speed, PERCENT)
                right_drive_smart.spin(FORWARD)
            # check the buttonL1/buttonL2 status
            # to control elevator
            if controller_1.buttonL1.pressing():
                elevator.spin(FORWARD)
                controller_1_left_shoulder_control_motors_stopped = False
            elif controller_1.buttonL2.pressing():
                elevator.spin(REVERSE)
                controller_1_left_shoulder_control_motors_stopped = False
            elif not controller_1_left_shoulder_control_motors_stopped:
                elevator.stop()
                # set the toggle so that we don't constantly tell the motor to stop when
                # the buttons are released
                controller_1_left_shoulder_control_motors_stopped = True
            # check the buttonR1/buttonR2 status
            # to control claw
            if controller_1.buttonR1.pressing():
                claw.spin(FORWARD)
                controller_1_right_shoulder_control_motors_stopped = False
            elif controller_1.buttonR2.pressing():
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

rc_auto_loop_thread_controller_1 = Thread(rc_auto_loop_function_controller_1)

buttonRightWasPressed = False
buttonYWasPressed = False
buttonXWasPressed = False
buttonAWasPressed = False
auton = True
def rc_auto_loop_function_controller_2():
    global auton
    global ROBERT
    global currentRobert
    global buttonRightWasPressed
    global buttonAWasPressed
    global buttonXWasPressed
    global buttonYWasPressed
    global intersection
    global controller_1_right_shoulder_control_motors_stopped, remote_control_code_enabled
    # process the controller input every 20 milliseconds
    # update the motors based on the input values
    while True:
        # check the buttonR1/buttonR2 status
        # to control motor_8
        if controller_1.buttonY.pressing():
            if not buttonYWasPressed:
            # claw.spin(FORWARD)
                # ROBERT = -2
                intersection = 1
                buttonYWasPressed = True
        else:
            buttonYWasPressed = False

        if controller_1.buttonX.pressing():
            if not buttonXWasPressed:
            # claw.spin(FORWARD)
                # ROBERT = -2
                intersection = 2
                buttonXWasPressed = True
        else:
            buttonXWasPressed = False

        if controller_1.buttonA.pressing():
            if not buttonAWasPressed:
            # claw.spin(FORWARD)
                # ROBERT = -2
                intersection = 3
                buttonAWasPressed = True
        else:
            buttonAWasPressed = False

        if controller_1.buttonRight.pressing():
            if not buttonRightWasPressed:
                if remote_control_code_enabled == True:
                    auton = True
                    remote_control_code_enabled = False
                    ROBERT = currentRobert
                    buttonRightWasPressed = True
                elif remote_control_code_enabled == False:
                    auton = False
                    remote_control_code_enabled = True
                    buttonRightWasPressed = True
        else:
            buttonRightWasPressed = False

        # elif not controller_1.buttonRight.pressing():
        #     buttonRightWasPressed = False
        # elif not controller_1.buttonY.pressing():
        #     buttonYWasPressed = False
        # elif controller_1.buttonLeft.pressing():
        #     # set the toggle so that we don't constantly tell the motor to stop when
        #     # the buttons are released
        #     pass
            # controller_1_right_shoulder_control_motors_stopped = True
        # wait before repeating the process
        wait(20, MSEC)

# define variable for remote controller enable/disable
rc_auto_loop_thread_controller_2 = Thread(rc_auto_loop_function_controller_2)

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      Billy NOT Jank
#	Author:       Conner, Parker, Prince, Izzy
#	Created:      05/06/2026
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
AVOID_DANGER = 5
FIND_WALL = 6
PANIC = 7
DELIVERING = 8
FIND_LINE = 9
LINE_FOLLOWING = 10
DEPOSIT_RESET = 11
FRUIT_NAVIGATION = 12
AUTO_RESET = 67
DEBUG = 420

ROBOT_STATE = IDLE
LAST_STATE = -1

# this might be confusing, but we use the variable ROBERT
# to control how we execute different steps within our state machine
ROBERT = 0
currentRobert = ROBERT
is_turning = False
driveCount = 0 # drive count is how much we drive to the fruit in FRUIT_NAVIGATION
fruit_count = 0
distance = 0
turning = False
heading = 0
distance = 0
desiredDistance = 0
desiredAngle = 0
currentFruit = 0

ROBOT_STATE = IDLE # initial starting state
count = 0 # count is what affects our swing turn decrements in FRUIT_NAVIGATION

# second_count is what determines our how far we drive before swinging for each consecutive fruit
second_count = 0 
intersection = 0 # intersection is what the robot responds to when it's at an intersection while line following
swingHeading = 0 # swing heading determines how much we swing less after each consecutive fruit

left_motor_1.set_stopping(HOLD)
left_motor_2.set_stopping(HOLD)
right_motor_1.set_stopping(HOLD)
right_motor_2.set_stopping(HOLD)
left_motor_1.set_velocity(42)
left_motor_2.set_velocity(42)
right_motor_1.set_velocity(42)
right_motor_2.set_velocity(42)
claw.set_position(0, DEGREES)
flap.set_position(0, DEGREES)
flap.set_velocity(100)

imu.calibrate()
wait(2, SECONDS)
imu.set_rotation(0, DEGREES)

timer = Timer()
GEAR_RATIO = 5/3
WHEEL_DIAM = 10.4775
CIRCUMFERENCE = math.pi * WHEEL_DIAM
turn_thread = None
CAMERA_RATIO = 0.0052806653
FRUIT_HEIGHT = 7.62
CAMERA_DIFF = -6.7
TAG_HEIGHT = 24
# 37 cm
# 39 pixels wide
# 7.62 cm tall


# PIDDrive(distance, heading)
# Takes a distance you want to drive and a boolean
# to make the robot move a certain distance using PID
# and the motor encoder position, with the option
# to heading correct to any fruit
class PIDDrive:
    def __init__(self, distance, heading):
        # initialize everything, set 0s
        timer.clear()
        # set gain constants
        self.kP, self.kI, self.kD = 5, 0.03, 85 #31.425
        self.kP_steer = 0.1
        self.slewRate = 0.05
        self.heading = heading
        
        # reset motors
        left_motor_1.set_position(0, DEGREES)
        left_motor_2.set_position(0, DEGREES)
        right_motor_1.set_position(0, DEGREES)
        right_motor_2.set_position(0, DEGREES)

        # convert distance in cm to degrees to be rotated
        motor_rotations = (distance / CIRCUMFERENCE) * GEAR_RATIO
        self.desiredDistance = motor_rotations * 360
        
        # set initial values for each of the terms
        self.currentHeading = imu.rotation(DEGREES)
        self.fruitError = 0
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prevError = 0
        self.motorPower = 0
        self.prevMotorPower = 0
        self.completed = False

    def update(self):
        # calculation logic
        global currentFruit
        if self.completed:
            return

        currentDistance = (left_motor_1.position() + left_motor_2.position() + 
                           right_motor_1.position() + right_motor_2.position()) / 4
        
        self.error = self.desiredDistance - currentDistance
        
        if -200 < self.error < 200:
            self.integral += self.error
            
        self.derivative = self.error - self.prevError
        self.motorPower = (self.kP * self.error) + (self.kI * self.integral) + (self.kD * self.derivative)

        # Heading and Slew logic
        currentAngle = imu.rotation(DEGREES)
        temp = detectFruit(currentFruit)

        headingCorrect = 0

        if self.heading:
            currentAngle = imu.rotation(DEGREES)
            temp = detectFruit(currentFruit)

            if temp[0] != 0:
                headingCorrect = (temp[0] - currentAngle) * self.kP_steer
        
        # Normalize and Slew
        self.motorPower = max(-1, min(1, self.motorPower))
        if self.motorPower > self.prevMotorPower + self.slewRate:
            self.motorPower = self.prevMotorPower + self.slewRate
        elif self.motorPower < self.prevMotorPower - self.slewRate:
            self.motorPower = self.prevMotorPower - self.slewRate

        # Voltage scaling and Motor spin
        left_raw = (11 * self.motorPower) + headingCorrect
        right_raw = (11 * self.motorPower) - headingCorrect
        max_raw = max(abs(left_raw), abs(right_raw))
        
        v_left = (left_raw / max_raw * 11) if max_raw > 11 else left_raw
        v_right = (right_raw / max_raw * 11) if max_raw > 11 else right_raw

        left_motor_1.spin(FORWARD, v_left, VOLT)
        left_motor_2.spin(FORWARD, v_left, VOLT)
        right_motor_1.spin(FORWARD, v_right, VOLT)
        right_motor_2.spin(FORWARD, v_right, VOLT)

        # Exit condition
        if -10 < self.error < 10 and -10 < (self.error - self.prevError) < 10:
            left_motor_1.stop()
            left_motor_2.stop()
            right_motor_1.stop()
            right_motor_2.stop()
            self.completed = True

        self.prevMotorPower = self.motorPower
        self.prevError = self.error

# PIDTurn(degrees, max_time_msec)
# Takes a angle you want to drive to and a timeout
# to make the robot turn a certain angle using PID
# and the motor encoder position and IMU, with the option
# to exit the PIDTurn early if it takes too long
class PIDTurn:
    global cringe
    def __init__(self, degrees, max_time_msec):
        #2.1, 17
        self.kP, self.kI, self.kD = 2.1, 0, 17
        if brain.battery.capacity() <= 60:
            self.kD = 22
        self.slewRate = 0.1
        self.target_degrees = degrees
        self.max_time = max_time_msec
        
        # Reset motors
        for m in [left_motor_1, left_motor_2, right_motor_1, right_motor_2]:
            m.set_position(0, DEGREES)

        # Timers
        self.start_time = brain.timer.time(MSEC)
        self.stall_start_time = brain.timer.time(MSEC)
        
        # PID Logic Variables
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prevError = 0
        self.motorPower = 0
        self.prevMotorPower = 0
        self.completed = False

    def update(self):
        global cringe
        # cringe = "stupid"
        # Check if already done or timed out
        global left_motor_1, left_motor_2, right_motor_1, right_motor_2
        current_time = brain.timer.time(MSEC)
        # cringe = "stupid"
        
        if self.completed:
            return

        # Hard Timeout check
        if (current_time - self.start_time) > self.max_time:
            self.stop_and_finish()
            # cringe = "fat"
            return

        # PID Calculation
        currentRotation = imu.rotation(DEGREES)
        self.error = self.target_degrees - currentRotation
        
        if -200 < self.error < 200:
            self.integral += self.error
            
        self.derivative = self.error - self.prevError
        self.motorPower = (self.kP * self.error) + (self.kI * self.integral) + (self.kD * self.derivative)

        # Normalize and Slew
        self.motorPower = max(-1, min(1, self.motorPower))
        if self.motorPower > self.prevMotorPower + self.slewRate:
            self.motorPower = self.prevMotorPower + self.slewRate
        elif self.motorPower < self.prevMotorPower - self.slewRate:
            self.motorPower = self.prevMotorPower - self.slewRate

        # Spin Motors (Turning uses opposite voltages)
        v = 11 * self.motorPower
        left_motor_1.spin(FORWARD, v, VOLT)
        left_motor_2.spin(FORWARD, v, VOLT)
        right_motor_1.spin(FORWARD, -v, VOLT)
        right_motor_2.spin(FORWARD, -v, VOLT)

        # Exit Conditions
        # 1. Target Reached
        if -1 < self.error < 1 and -0.5 < (self.error - self.prevError) < 0.5:
            self.stop_and_finish()
            return

        # 2. Stall/Oscillation Check
        # if abs(self.error - self.prevError) > 0.5:
        #     self.stall_start_time = current_time
        # elif (current_time - self.stall_start_time) > 250:
        #     self.stop_and_finish()
        #     return
        if (abs(self.error - self.prevError) > 0.5): 
            self.stall_start_time = current_time  
        # stuck in oscillation
        elif (current_time - self.stall_start_time > 250):
            self.stop_and_finish()
            # cringe = "chud"
            return
        
         # Update Screen and Variables
        # controller_1.screen.clear_screen()
        # controller_1.screen.set_cursor(1,1)
        # controller_1.screen.print(currentRotation)

        self.prevMotorPower = self.motorPower
        self.prevError = self.error

    def stop_and_finish(self):
        left_motor_1.stop()
        left_motor_2.stop()
        right_motor_1.stop()
        right_motor_2.stop()
        self.completed = True

# PIDSwing(distance, heading)
# Takes a distance you want to drive and a boolean
# to make the robot swing drive a certain distance using PID
# and the motor encoder position, with the option
# to heading correct to any fruit
# It mostly just powers half of the drivetrain, it's experimental how far it gets
class PIDSwing:
    def __init__(self, distance, heading, side):
        timer.clear()
        # set gain constants
        self.kP, self.kI, self.kD = 5, 0.03, 85 #31.425
        self.kP_steer = 0.1
        self.slewRate = 0.05
        self.heading = heading
        self.side = side
        
        # reset motors
        left_motor_1.set_position(0, DEGREES)
        left_motor_2.set_position(0, DEGREES)
        right_motor_1.set_position(0, DEGREES)
        right_motor_2.set_position(0, DEGREES)

        # convert distance in cm to degrees to be rotated
        motor_rotations = (distance / CIRCUMFERENCE) * GEAR_RATIO
        self.desiredDistance = motor_rotations * 360
        
        # set initial values for each of the terms
        self.currentHeading = imu.rotation(DEGREES)
        self.fruitError = 0
        self.error = 0
        self.integral = 0
        self.derivative = 0
        self.prevError = 0
        self.motorPower = 0
        self.prevMotorPower = 0
        self.completed = False

    def update(self):
        # calc logic
        if self.completed:
            return
    
        if not self.side:
            currentDistance = (left_motor_1.position() + left_motor_2.position()) / 2
            right_motor_1.set_stopping(COAST)
            right_motor_2.set_stopping(COAST)
        elif self.side:
            currentDistance = (right_motor_1.position() + right_motor_2.position()) / 2
            left_motor_1.set_stopping(COAST)
            left_motor_2.set_stopping(COAST)
        
        self.error = self.desiredDistance - currentDistance
        
        if -200 < self.error < 200:
            self.integral += self.error
            
        self.derivative = self.error - self.prevError
        self.motorPower = (self.kP * self.error) + (self.kI * self.integral) + (self.kD * self.derivative)

        # Heading and Slew logic
        currentAngle = imu.rotation(DEGREES)
        temp = detectFruit(currentFruit)
        headingCorrect = 0

        if self.heading:
            currentAngle = imu.rotation(DEGREES)
            temp = detectFruit(currentFruit)
            
            if temp[0] != 0:
                headingCorrect = (temp[0] - currentAngle) * self.kP_steer
        
        # Normalize and Slew
        self.motorPower = max(-1, min(1, self.motorPower))
        if self.motorPower > self.prevMotorPower + self.slewRate:
            self.motorPower = self.prevMotorPower + self.slewRate
        elif self.motorPower < self.prevMotorPower - self.slewRate:
            self.motorPower = self.prevMotorPower - self.slewRate

        # Voltage scaling and Motor spin
        left_raw = (11 * self.motorPower) + headingCorrect
        right_raw = (11 * self.motorPower) - headingCorrect
        max_raw = max(abs(left_raw), abs(right_raw))
        
        v_left = (left_raw / max_raw * 11) if max_raw > 11 else left_raw
        v_right = (right_raw / max_raw * 11) if max_raw > 11 else right_raw

        if self.side:
            v_left = 0
        elif not self.side:
            v_right = 0

        left_motor_1.spin(FORWARD, v_left, VOLT)
        left_motor_2.spin(FORWARD, v_left, VOLT)
        right_motor_1.spin(FORWARD, v_right, VOLT)
        right_motor_2.spin(FORWARD, v_right, VOLT)

        # Exit condition
        if -10 < self.error < 10 and -10 < (self.error - self.prevError) < 10:
            left_motor_1.stop()
            left_motor_2.stop()
            right_motor_1.stop()
            right_motor_2.stop()
            self.completed = True
            left_motor_1.set_stopping(HOLD)
            left_motor_2.set_stopping(HOLD)
            right_motor_1.set_stopping(HOLD)
            right_motor_2.set_stopping(HOLD)
            

        self.prevMotorPower = self.motorPower
        self.prevError = self.error

# execute_threaded_turn(target_angle, timeout)
# this is the threaded solution we use to run our PIDTurns because it needs
# very precise timing. therefore, we run it in a while loop in it's own thread
# to not break any rules of the state machine
def execute_threaded_turn(target_angle, timeout):
    global left_motor_1, left_motor_2, right_motor_1, right_motor_2
    global is_turning
    global cringe
    global auton
    is_turning = True
    # Create the task locally so it has a fresh start_time
    task = PIDTurn(target_angle, timeout)
    
    # High-frequency loop dedicated ONLY to this turn
    if auton == False:
        return
    else:
        while not task.completed:
            task.update()
            wait(71, MSEC)
        is_turning = False

dist = 0
# detectFruit(currentFruit)
# this function detects the fruit, you give it 0 initially so it looks for any fruit
# and then you can choose any fruit for it to lock onto by sending it's id number
# it tries to find the closest fruit first and then use the vision sensor's focal length
# and other things to calculate the distance and the angle to turn to the fruit
def detectFruit(currentFruit):
    global dist

    objects = [
        {"type": 20, "snap": fruit_vision.take_snapshot(fruit__apricot)},
        {"type": 22, "snap": fruit_vision.take_snapshot(fruit__lime)},
        {"type": 23, "snap": fruit_vision.take_snapshot(fruit__grape)}
    ]

    detected_list = []

    for item in objects:
        snap = item["snap"]

        if snap and len(snap) > 0 and snap[0].score > 80:
            # If currentFruit is set, only accept that fruit type
            if currentFruit != 0 and item["type"] != currentFruit:
                continue

            dist = FRUIT_HEIGHT / (snap[0].height * CAMERA_RATIO)

            detected_list.append({
                "type": item["type"],
                "distance": dist,
                "centerX": snap[0].centerX,
                "rotation": imu.rotation(DEGREES)
            })

    if detected_list == []:
        return 0, 0, 0

    # closest valid fruit
    detected_list.sort(key=lambda x: x["distance"])
    target = detected_list[0]

    xDistance = (target["centerX"] - 160) * target["distance"] * CAMERA_RATIO
    desiredAngle = math.atan2(xDistance, target["distance"]) * (180 / math.pi)

    # Pick offset based on the ACTUAL target type, not currentFruit
    if target["type"] == 20:
        offset = -7.6
    elif target["type"] == 22:
        offset = -7.4
    elif target["type"] == 23:
        offset = -7.6
    else:
        offset = 0

    return desiredAngle + imu.rotation(DEGREES) + offset, target["distance"] - CAMERA_DIFF, target["type"]

# detectTag()
# this function just returns any april tag it sees, it's for finding
# the tags on the baskets for the fruit
def detectTag():
    object1 = april_vision.take_snapshot(AiVision.ALL_TAGS)
    if object1[0].exists:
        return object1[0].id

# wallTag()
# this function returns how far away a big april tag is, it's used
# to find the wall to begin line following in our state machine

def wallTag():
    object1 = april_vision.take_snapshot(AiVision.ALL_TAGS)
    if not object1:
        return 0, 0, 0
    dist = TAG_HEIGHT / (object1[0].height * CAMERA_RATIO)
    xDistance = (object1[0].centerX - 160) * dist * CAMERA_RATIO
    desiredAngle = math.atan2(xDistance, dist) * (180 / math.pi)
    return  desiredAngle + imu.rotation(DEGREES) + 90, 67, object1[0].id

# detectTree()
# this is an unused function, but it basically tries to detect the white of the tree
# to turn away from the tree and avoid it when it is trying to go to a wall
def detectTree():
    object1 = fruit_vision.take_snapshot(fruit__tree)
    if object1[0].exists:
        distance = FRUIT_HEIGHT / (object1[0].height * CAMERA_RATIO)
        xDistance = (object1[0].centerX - 160) * distance * CAMERA_RATIO
        desiredAngle = math.atan2(xDistance, distance) * (180 / math.pi)
        if object1[0].score > 80:
            return desiredAngle + imu.rotation(DEGREES), distance - CAMERA_DIFF
        else:
            return 0,0
    else:
        return 0,0

# PID stuff for line_roberting()
Kp = 0.35
Kd = 4
intersectionCount = 0
lastError = 0

# line_roberting()
# this function is our line following function which has kP and kD
# this allows our 4 wheel drive to line follow accurately
# also depending on what button we press, it will choose which way
# it turns at an intersection
def line_roberting():
    global intersectionCount
    global intersection
    global lastError
    # robert is doing the line
    right_reflectivity = line_tracker_left.reflectivity()
    left_reflectivity = line_tracker_right.reflectivity()
    print(right_reflectivity, left_reflectivity)

    intersection_reflectivity = 40
    line_error =  right_reflectivity - left_reflectivity
    derivative = line_error - lastError
    turning_effort = Kp * line_error + Kd * derivative
    
    base_speed = (15 / CIRCUMFERENCE) * 60 * GEAR_RATIO
    
    if (right_reflectivity > intersection_reflectivity) and (left_reflectivity > intersection_reflectivity):
        return intersection

    else:
        left_motor_1.spin(REVERSE, base_speed - turning_effort, RPM)
        left_motor_2.spin(REVERSE, base_speed , RPM)
        right_motor_1.spin(REVERSE, base_speed , RPM)
        right_motor_2.spin(REVERSE, base_speed + turning_effort, RPM)
        lastError = line_error
        return -1, -1


# set all of our tasks so they can be global and pass through scope
turn_task = PIDTurn(0, 0)
drive_task = PIDDrive(0, False)
swing_task = PIDSwing(0, False, False)

# mission()
# this is our main state machine function, it has a lot of different states and variables
def mission():
    global intersection
    global driveCount
    global second_count
    global count
    global auton
    global swingHeading
    global ROBOT_STATE
    global cringe
    global LAST_STATE
    random  = 1
    global drive_task
    global drive_task2
    global turn_task
    global fruit_count
    global ROBERT
    global distance_values
    global angle_values
    global heading
    global intersectionCount
    global distance
    global desiredAngle
    global currentFruit
    global turning
    global desiredDistance
    global turn_thread
    global is_turning
    global swing_task
    global currentRobert

    # this is to pause the state machine if I enable driver control to fix something
    if auton == False:
        if ROBERT != AUTO_RESET:
            currentRobert = ROBERT
            left_motor_1.stop()
            left_motor_2.stop()
            right_motor_1.stop()
            right_motor_2.stop()
            is_turning = False
            ROBERT = AUTO_RESET

    # this is our starting state, which transitions to RAMP_DRIVE if I hold the button down
    elif ROBOT_STATE == IDLE and controller_1.buttonL1.pressing():
        ROBOT_STATE = RAMP_DRIVE
        LAST_STATE = IDLE
    
    # debug state, we put any code here and just test it when we need to
    elif ROBOT_STATE == DEBUG:
        temp = line_roberting()
    
    # fruit navigation, this is where we start at a fruit, and 
    # turn-drive-swing-drive to get the next fruit on the tree
    # one thing to clarify, making the instance of any PID class needs to be called only once,
    # so you will see a lot of making the instance and then changing the ROBERT and then on the next
    # ROBERT we call the update function for that class to start the PID calculations and movement
    elif ROBOT_STATE == FRUIT_NAVIGATION:
        global turn_thread
        global ROBERT
        global is_turning
        global swingHeading
        if ROBERT == -2:
            drive_task = PIDDrive(-8, False)
            swingHeading = imu.rotation(DEGREES)
            ROBERT = -1
            claw.spin(FORWARD)
            timer.reset()
        elif ROBERT == -1:
            if timer.time() >= 2500:
                drive_task.update()
                if drive_task.completed:
                    ROBERT = 0
                    if currentFruit == 3:
                        claw.stop()
                        ROBERT = 7
                    else:
                        claw.spin_to_position(0, DEGREES, False)
        elif ROBERT == 0:
            target = imu.rotation(DEGREES) - 38 #40
            is_turning = True
            turn_thread = Thread(lambda: execute_threaded_turn(target, 3000))
            ROBERT = 1

        elif ROBERT == 1:
            if not is_turning:
                drive_task = PIDDrive(34 - second_count, False)
                ROBERT = 2

        elif ROBERT == 2:
            drive_task.update()
            if drive_task.completed:
                ROBERT = 3
        
        elif ROBERT == 3:
            swing_task = PIDSwing(74 + (count), False, False) #67 or 70
            ROBERT = 4
        elif ROBERT == 4:
            swing_task.update()
            if swing_task.completed:
                desiredInfo = detectFruit(currentFruit)
                print(desiredInfo[0])
                print(desiredInfo[1])
                distance = desiredInfo[1]
                print(currentFruit)
                drive_task = PIDDrive(21 - driveCount, True) #25
                ROBERT = 5
        elif ROBERT == 5:
            drive_task.update()
            if drive_task.completed:
                # claw.spin(FORWARD)
                ROBERT = 6
                # timer.reset()
        elif ROBERT == 6:
            fruit_count += 1
            turn_thread = Thread(lambda: execute_threaded_turn(swingHeading + 90, 2500))
            is_turning = True
            timer.reset()
            # print("first")
            if fruit_count == 2:
                # print("second")
                count += 13
                driveCount += 8
                second_count += 8.5
            elif fruit_count == 3:
                # print("third")
                second_count -= 4
                count -= 12
                driveCount = 17
                ROBERT = -2
            elif fruit_count == 4:
                ROBERT = 7
                drive_task = PIDDrive(6.7, False)

        elif ROBERT == 7:
            drive_task.update()
            if drive_task.completed:
                claw.spin(FORWARD)
                if timer.time() >= 3000:
                    # claw.stop()
                    ROBOT_STATE = AVOID_DANGER
                    ROBERT = 3
            
    
    # ramp drive, this drives us up the ramp and aligns with the first fruit
    elif ROBOT_STATE == RAMP_DRIVE:
        if ROBERT == -3:
            drive_task.update()
            if drive_task.completed:
                ROBERT = -2
                turn_thread = Thread(lambda: execute_threaded_turn(imu.rotation(DEGREES) - 97, 2500))
                is_turning = True
        if ROBERT == -2:
            if not is_turning:
                ROBERT = 0
                ROBOT_STATE = SEARCHING
        elif ROBERT == -1:
            if not is_turning:
                drive_task = PIDDrive(12, False)
                ROBERT = -3

        elif LAST_STATE != RAMP_DRIVE:
            drive_task = PIDDrive(306.7, True)
            LAST_STATE = RAMP_DRIVE
        elif drive_task.completed:
            ROBERT = -1
            temp = imu.rotation(DEGREES) + 90
            turn_thread = Thread(lambda: execute_threaded_turn(temp, 2500))
            is_turning = True
        else: 
            drive_task.update()

    # searching state, tries to look for fruit and then drives towards it
    elif ROBOT_STATE == SEARCHING:
        if ROBERT == 0:
            global turn_thread
            desiredInfo = detectFruit(currentFruit)
            if desiredInfo[0] == 0 and desiredInfo[1] == 0:
                left_motor_1.set_velocity(50)
                left_motor_2.set_velocity(50)
                right_motor_1.set_velocity(50)
                right_motor_2.set_velocity(50)
                left_motor_1.spin(FORWARD)
                left_motor_2.spin(FORWARD)
                right_motor_1.spin(REVERSE)
                right_motor_2.spin(REVERSE)
            else:
                if desiredInfo[1] > 60: # If fruit is further than 60cm
                    left_motor_1.set_velocity(50)
                    left_motor_2.set_velocity(50)
                    right_motor_1.set_velocity(50)
                    right_motor_2.set_velocity(50)
                    left_motor_1.spin(FORWARD)
                    left_motor_2.spin(FORWARD)
                    right_motor_1.spin(REVERSE)
                else:
                    global turn_thread
                    left_motor_1.stop()
                    left_motor_2.stop()
                    right_motor_1.stop()
                    right_motor_2.stop()
                    desiredInfo = detectFruit(0)
                    print(desiredInfo[0])
                    print(desiredInfo[1])
                    distance = desiredInfo[1]
                    currentFruit = desiredInfo[2]
                    print(desiredInfo[2])
                    turn_thread = Thread(lambda: execute_threaded_turn(desiredInfo[0], 2500))
                    ROBERT = 1
                    is_turning = True

        elif ROBERT == 1:
            if not is_turning:
                ROBOT_STATE = APPROACHING
                LAST_STATE = SEARCHING
        elif ROBERT == 0.5:
            drive_task.update()
            if drive_task.completed:
                ROBERT = 0

    # approaching, it sees the fruit and then drives toward it before moving to HARVESTING
    elif ROBOT_STATE == APPROACHING:
        if LAST_STATE != APPROACHING:
            print(distance)
            drive_task = PIDDrive(distance, True)
            LAST_STATE = APPROACHING
        if drive_task.completed:
            ROBOT_STATE = HARVESTING
            timer.reset()
        else: 
            drive_task.update()

    # harvesting, it kind of got replaced by FRUIT_NAVIGATION because it's so complex,
    # so it changes the ROBERT and moves to FRUIT_NAVIGATION
    elif ROBOT_STATE == HARVESTING:
        if ROBERT == 1:
            ROBERT = -2
            ROBOT_STATE = FRUIT_NAVIGATION

    # avoid danger, it just drives away to back away from the tree it's at before moving to FIND_WALL
    elif ROBOT_STATE == AVOID_DANGER:
        if ROBERT == 3:
            drive_task = PIDDrive(-10.24, False)
            ROBERT = 4
        if ROBERT == 4:
            drive_task.update()
            if drive_task.completed:
                ROBOT_STATE = FIND_WALL
                ROBERT = 5
                timer.reset()
                heading = imu.rotation(DEGREES)
            

    # find wall, it spins in a circle while the second vision sensor on the side of the robot
    # looks around for the big april tags to drive to the wall
    elif ROBOT_STATE == FIND_WALL:
        if ROBERT == 5:
            left_motor_1.spin(FORWARD)
            left_motor_2.spin(FORWARD)
            right_motor_1.spin(REVERSE)
            right_motor_2.spin(REVERSE)
            desiredInfo = wallTag()
            if desiredInfo[0] != 0:

                for m in [left_motor_1, left_motor_2, right_motor_1, right_motor_2]:
                    m.stop()
                distance = desiredInfo[1]
                turn_thread = Thread(lambda: execute_threaded_turn(desiredInfo[0], 2500))
                is_turning = True
                ROBERT = 6

        # some code was removed here, don't worry about it
        if ROBERT == 6:
            ROBERT = 7

        if ROBERT == 7:
            if not is_turning:
                ROBERT = 8
                drive_task = PIDDrive(distance, False)
            else:
                random += 1
        if ROBERT == 8:
            info = detectTree()
            drive_task.update()
            if drive_task.completed:
                ROBERT = 11
                ROBOT_STATE = FIND_LINE
            # unused PANIC mode
            elif random == 69420:
                ROBOT_STATE = PANIC
                desiredAngle = info[0]
                distance = info[1]
                ROBERT = 9

    # currently unused, but it would detect the tree with the vision sensor and turn away from it
    # that way it can find the wall without running into the tree except it doesn't work very well
    elif ROBOT_STATE == PANIC:
        if ROBERT == 9:
            turn_thread = Thread(lambda: execute_threaded_turn(desiredAngle + imu.rotation(DEGREES), 2500))
            ROBERT = 10
        if ROBERT == 10:
            if not is_turning:
                ROBOT_STATE = FIND_WALL
                ROBERT = 5
                timer.reset()
                heading = imu.rotation(DEGREES)

    # find line, after we find the wall, it drives toward the wall and then drives a little more
    # this way it hits the button, which then we back away from the wall a little bit and turn to get
    # on the line and begin line following. additionally, while it's line following, if it sees the fruit basket
    # it's looking for it transitions states to handle that
    elif ROBOT_STATE == FIND_LINE:
        if ROBERT == 11:
            drive_task = PIDDrive(40, False)
            ROBERT = 12
        if ROBERT == 12:
            drive_task.update()
            if button.pressing():
                ROBERT = 13
        if ROBERT == 13:
            drive_task = PIDDrive(-4.2, False)
            ROBERT = 14
        if ROBERT == 14:
            drive_task.update()
            if drive_task.completed:
                ROBERT = 15
        if ROBERT == 15:
            currentHeading = imu.rotation(DEGREES)
            turn_thread = Thread(lambda: execute_threaded_turn(currentHeading - 90, 2500))
            ROBERT = 16
        if ROBERT == 16:
            if not is_turning:
                ROBERT = 17
        if ROBERT == 17:
            print(currentFruit)
            if detectTag() == currentFruit:
                ROBERT = 17.5
                timer.reset()
                
            else:
                if not is_turning:
                    fieldPosition = line_roberting()
                    if fieldPosition == 1:
                        turn_thread = Thread(lambda: execute_threaded_turn(imu.rotation(DEGREES) + 90, 2500))
                        is_turning = True
                    elif fieldPosition == 2:
                        turn_thread = Thread(lambda: execute_threaded_turn(imu.rotation(DEGREES) - 90, 2500))
                        is_turning = True
                    elif fieldPosition == 3:
                        turn_thread = Thread(lambda: execute_threaded_turn(imu.rotation(DEGREES) + 180, 2500))
                        is_turning = True
                    else:
                        pass
        if ROBERT == 17.5:
            if timer.time() >= 1000:
                ROBERT = 18
                ROBOT_STATE = DELIVERING
            else:
                line_roberting()

    # delivering, it turns the robot into place so it faces the basket
    elif ROBOT_STATE == DELIVERING:
        if ROBERT == 18:
            for m in [left_motor_1, left_motor_2, right_motor_1, right_motor_2]:
                m.stop()
            turn_thread = Thread(lambda: execute_threaded_turn(imu.rotation(DEGREES) + 90, 1500))
            is_turning = True
            ROBERT = 19
        if ROBERT == 19:
            if not is_turning:
                ROBOT_STATE = DEPOSIT_RESET
                timer.reset()
    
    # deposit_reset, it handles dumping the hopper and then going back all the way to the beginning
    elif ROBOT_STATE == DEPOSIT_RESET:
    
        if ROBERT != 20:
            if not button.pressing():
                left_motor_1.spin(FORWARD)
                left_motor_2.spin(FORWARD)
                right_motor_1.spin(FORWARD)
                right_motor_2.spin(FORWARD)
            else:
                left_motor_1.stop()
                left_motor_2.stop()
                right_motor_1.stop()
                right_motor_2.stop()

                claw.spin_to_position(0, DEGREES)
                flap.spin_to_position(235, DEGREES)

                ROBERT = 20
                timer.reset()

        elif ROBERT == 20:
            if timer.time() >= 3000:
                flap.spin_to_position(0, DEGREES)

                ROBERT = 0
                fruit_count = 0
                intersectionCount = 0
                currentFruit = 0
                ROBOT_STATE = SEARCHING
    
    # this state should never be reached, but if it does then we know something went wrong
    else:
        print("67")

# while true, where we call mission and bring debug info to the controller screen
# we see current state, the ROBERT count, and the current heading of the robot
# it prints every 500ms in order to not lag the brain
while True:
    mission()
    if brain.timer.time(MSEC) % 500 < 20: 
        controller_1.screen.clear_screen()
        brain.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        brain.screen.set_cursor(1,1)
        controller_1.screen.print("ROBOT_STATE: {:.2f}".format(ROBOT_STATE))
        controller_1.screen.next_row()
        controller_1.screen.print("ROBERT: {:.2f}".format(ROBERT))
        controller_1.screen.next_row()
        controller_1.screen.print("HEADING: {:.2f}".format(imu.rotation(DEGREES)))
    
    wait(20, MSEC)