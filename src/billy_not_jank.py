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
fruit__apricot = Colordesc(1, 233, 77, 81, 10, 0.19)
fruit__lime = Colordesc(2, 50, 234, 125, 8, 0.28)
fruit__grape = Colordesc(3, 146, 89, 195, 10, 0.2)
fruit__tree = Colordesc(4, 121, 163, 160, 40, 0.1)
fruit_vision = AiVision(Ports.PORT16, fruit__apricot, fruit__lime, fruit__grape, fruit__tree)
april_vision = AiVision(Ports.PORT14, AiVision.ALL_TAGS)
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
line_tracker_left = Line(brain.three_wire_port.h)
line_tracker_right = Line(brain.three_wire_port.f)
ultrasonic = Sonar(brain.three_wire_port.c)


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
AVOID_DANGER = 5
FIND_WALL = 6
PANIC = 7
DELIVERING = 8
FIND_LINE = 9
LINE_FOLLOWING = 10
DEPOSIT_RESET = 11
DOING_YA_MOM = 67

ROBOT_STATE = IDLE
LAST_STATE = -1

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
CAMERA_RATIO = 0.0052806653
FRUIT_HEIGHT = 7.62
# 37 cm
# 39 pixels wide
# 7.62 cm tall

timer = Timer()
class PIDDrive:
    def __init__(self, distance):
        timer.clear()
        # set gain constants
        self.kP, self.kI, self.kD = 5, 0.00, 31.425
        self.kP_steer = 0.0
        self.slewRate = 0.1
        
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

        currentDistance = (left_motor_1.position() + left_motor_2.position() + 
                           right_motor_1.position() + right_motor_2.position()) / 4
        
        self.error = self.desiredDistance - currentDistance
        
        if -200 < self.error < 200:
            self.integral += self.error
            
        self.derivative = self.error - self.prevError
        self.motorPower = (self.kP * self.error) + (self.kI * self.integral) + (self.kD * self.derivative)

        # Heading and Slew logic
        currentAngle = imu.rotation(DEGREES)
        headingCorrect = (self.currentHeading - currentAngle) * self.kP_steer
        
        # Normalize and Slew
        self.motorPower = max(-1, min(1, self.motorPower))
        if self.motorPower > self.prevMotorPower + self.slewRate:
            self.motorPower = self.prevMotorPower + self.slewRate
        elif self.motorPower < self.prevMotorPower - self.slewRate:
            self.motorPower = self.prevMotorPower - self.slewRate

        # Voltage scaling and Motor spin
        left_raw = (11 * self.motorPower) - headingCorrect
        right_raw = (11 * self.motorPower) + headingCorrect
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

class PIDTurn:
    def __init__(self, degrees, max_time_msec):
        self.kP, self.kI, self.kD = 2.1, 0, 12
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
        # Check if already done or timed out
        current_time = brain.timer.time(MSEC)
        
        if self.completed:
            return

        # Hard Timeout check
        if (current_time - self.start_time) > self.max_time:
            self.stop_and_finish()
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
        if abs(self.error - self.prevError) > 0.5:
            self.stall_start_time = current_time
        elif (current_time - self.stall_start_time) > 250:
            self.stop_and_finish()
            return

        # Update Screen and Variables
        controller_1.screen.clear_screen()
        controller_1.screen.set_cursor(1,1)
        controller_1.screen.print(currentRotation)

        self.prevMotorPower = self.motorPower
        self.prevError = self.error

    def stop_and_finish(self):
        left_motor_1.stop()
        left_motor_2.stop()
        right_motor_1.stop()
        right_motor_2.stop()
        self.completed = True

def detectFruit():
    object1 = fruit_vision.take_snapshot(fruit__apricot)
    object2 = fruit_vision.take_snapshot(fruit__lime)
    object3 = fruit_vision.take_snapshot(fruit__grape)

    if object1[0].score > 80:
        # apricot logic
        distance = FRUIT_HEIGHT / (object1[0].height * CAMERA_RATIO)
        xDistance = (object1[0].centerX - 160) * distance * CAMERA_RATIO
        desiredAngle = math.tan(xDistance / distance) * (180 / math.pi)
        return desiredAngle, distance
    elif object2[0].score > 80:
        # lime logic
        distance = FRUIT_HEIGHT / (object2[0].height * CAMERA_RATIO)
        xDistance = (object2[0].centerX - 160) * distance * CAMERA_RATIO
        desiredAngle = math.tan(xDistance / distance) * (180 / math.pi)
        return desiredAngle, distance
    elif object3[0].score > 80:
        # grape logic
        distance = FRUIT_HEIGHT / (object3[0].height * CAMERA_RATIO)
        xDistance = (object3[0].centerX - 160) * distance * CAMERA_RATIO
        desiredAngle = math.tan(xDistance / distance) * (180 / math.pi)
        return desiredAngle, distance
    else:
        return 0, 0

def detectTree():
    object1 = fruit_vision.take_snapshot(fruit__tree)
    distance = FRUIT_HEIGHT / (object1[0].height * CAMERA_RATIO)
    xDistance = (object1[0].centerX - 160) * distance * CAMERA_RATIO
    desiredAngle = math.tan(xDistance / distance) * (180 / math.pi)
    if object1[0].score > 80:
        return desiredAngle, distance
    else:
        return 0,0

ROBERT = 0
fruit_count = 0
distance_values = []
angle_values = []
def mission():
    global ROBOT_STATE
    global LAST_STATE
    global drive_task
    global drive_task2
    global turn_task
    global fruit_count
    global ROBERT
    global distance_values
    global angle_values
    global heading
    heading = 67
    distance = -67
    desiredAngle = 670

    if ROBOT_STATE == IDLE and controller_1.buttonL1.pressing():
        ROBOT_STATE = RAMP_DRIVE
        LAST_STATE = IDLE
    elif ROBOT_STATE == RAMP_DRIVE:
        if LAST_STATE != RAMP_DRIVE:
            drive_task = PIDDrive(292)
            LAST_STATE = RAMP_DRIVE
        if drive_task.completed:
            ROBOT_STATE = SEARCHING
        else: 
            drive_task.update()
    elif ROBOT_STATE == SEARCHING:
        if ROBERT == 0:
            desiredInfo = detectFruit()
            if desiredInfo == (0, 0):
                left_motor_1.set_velocity(50)
                left_motor_2.set_velocity(50)
                left_motor_1.spin(FORWARD)
                left_motor_2.spin(FORWARD)
            else:
                left_motor_1.stop()
                left_motor_2.stop()
                desiredInfo = detectFruit()
                distance = desiredInfo[1]
                turn_task = PIDTurn(desiredInfo[0], 2500)
                ROBERT = 1

        if ROBERT == 1 and not turn_task.completed:
            turn_task.update()
        else:
            ROBOT_STATE = APPROACHING
            LAST_STATE = SEARCHING

    elif ROBOT_STATE == APPROACHING:
        if LAST_STATE != APPROACHING:
            drive_task = PIDDrive(distance)
            LAST_STATE = APPROACHING
        if drive_task.completed:
            ROBOT_STATE = HARVESTING
            timer.reset()
            drive_task2 = PIDDrive(-5)
        else: 
            drive_task.update()
    elif ROBOT_STATE == HARVESTING:
        if ROBERT == 1:
            drive_task = PIDDrive(-5)
            ROBERT = 2
        if ROBERT == 2:
            claw.spin(FORWARD)
            if timer.time() > 2500:
                drive_task.update()
            elif drive_task.completed:
                claw.spin_for(REVERSE, 0.2, SECONDS)
                fruit_count += 1
                if fruit_count == 4:
                    ROBOT_STATE = DELIVERING
                    ROBERT = 3
                else:
                    ROBOT_STATE = SEARCHING
                    ROBERT = 0
    elif ROBOT_STATE == AVOID_DANGER:
        if ROBERT == 3:
            drive_task = PIDDrive(-10.24)
            ROBERT = 4
        if ROBERT == 4:
            drive_task.update()
        elif drive_task.completed:
            # global heading
            ROBOT_STATE = FIND_WALL
            ROBERT = 5
            timer.reset()
            heading = imu.rotation(DEGREES)
            

    elif ROBOT_STATE == FIND_WALL:
        if ROBERT == 5:
            right_motor_1.set_velocity(50)
            right_motor_2.set_velocity(50)
            left_motor_1.spin(FORWARD)
            left_motor_2.spin(FORWARD)
            right_motor_1.spin(REVERSE)
            right_motor_2.spin(REVERSE)
            if timer.time() > 500:
                distance_values.append(ultrasonic.distance(MM)*10)
                angle_values.append(imu.rotation(DEGREES))
            if imu.rotation(DEGREES) == heading + 360:
                ROBERT = 6
        if ROBERT == 6:
            distance_values.sort()
            angle_values.sort()
            desiredDistance = distance_values[0]
            desiredAngle = angle_values[0]
            turn_task = PIDTurn(desiredAngle + 180, 2500)
            ROBERT = 7
        if ROBERT == 7:
            turn_task.update()
            if turn_task.completed:
                ROBERT = 8
                drive_task = PIDDrive(desiredDistance)
        if ROBERT == 8:
            info = detectTree()
            if info[0] == 0 and info[1] == 0:
                drive_task.update()
                if drive_task.completed:
                    ROBERT = 11
                    ROBOT_STATE = FIND_LINE
            else:
                ROBOT_STATE = PANIC
                desiredAngle = info[0]
                distance = info[1]
                ROBERT = 9
    elif ROBOT_STATE == PANIC:
        if ROBERT == 9:
            turn_task = PIDTurn(desiredAngle, 2500)
            ROBERT = 10
        if ROBERT == 10:
            turn_task.update()
            if turn_task.completed:
                ROBOT_STATE = FIND_WALL
                ROBERT = 5
                timer.reset()
                heading = imu.rotation(DEGREES)
    elif ROBOT_STATE == FIND_LINE:
        if ROBERT == 11:
            drive_task = PIDDrive(67)
            ROBERT = 12
        if ROBERT == 12:
            drive_task.update()
            if button.pressed:
                ROBERT = 13
        if ROBERT == 13:
            drive_task = PIDDrive(-6.7)
            ROBERT = 14
        if ROBERT == 14:
            drive_task.update()
            if drive_task.completed:
                ROBERT = 15
        if ROBERT == 15:
            currentHeading = imu.rotation(DEGREES)
            turn_task = PIDTurn(currentHeading + 90, 2500)
            ROBERT = 16
        if ROBERT == 16:
            turn_task.update()
            if turn_task.completed:
                ROBERT = 17
        if ROBERT == 17:
            pass
    
    elif ROBOT_STATE == DEPOSIT_RESET:
        pass
    else:
        print("67")


while True:
    mission()