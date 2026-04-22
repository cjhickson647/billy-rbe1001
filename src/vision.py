#region VEXcode Generated Robot Configuration
from vex import *
import urandom #type: ignore
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
bumper_g = Bumper(brain.three_wire_port.g)
right_motor_motor_a = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
right_motor_motor_b = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
right_motor = MotorGroup(right_motor_motor_a, right_motor_motor_b)
left_motor_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_motor_motor_b = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
left_motor = MotorGroup(left_motor_motor_a, left_motor_motor_b)
controller_1 = Controller(PRIMARY)
# AI Vision Color Descriptions
ai_vision_15__apricot = Colordesc(1, 233, 77, 81, 10, 0.19)
ai_vision_15__lime = Colordesc(2, 50, 234, 125, 8, 0.28)
ai_vision_15__grape = Colordesc(3, 146, 89, 195, 10, 0.2)
# AI Vision Code Descriptions
ai_vision_15 = AiVision(Ports.PORT16, ai_vision_15__apricot, ai_vision_15__lime, ai_vision_15__grape)
motor_19 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
motor_17_motor_a = Motor(Ports.PORT17, GearSetting.RATIO_18_1, False)
motor_17_motor_b = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
motor_17 = MotorGroup(motor_17_motor_a, motor_17_motor_b)


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
'''
This code demonstrates a basic search and drive towards behaviour with the camera.

The robot has three states:
    IDLE - waiting for the button press
    SEARCHING - spins slowly until it finds an object
    APPROACHING - drives towards the object

Camera checking is done on a timer. If no object is found, a counter is incremented and
if the counter reaches a threshold, the robot goes back into searching mode.
'''

## Define states and state variable
ROBOT_IDLE = 0
ROBOT_SEARCHING = 1
ROBOT_APPROACHING = 2
ELEVATE = 3
GRAB = 4
left_motor.set_stopping(HOLD)
right_motor.set_stopping(HOLD)

current_state = ROBOT_IDLE

'''
We'll use a timer to read the camera every cameraInterval milliseconds
'''
cameraInterval = 50
cameraTimer = Timer()

def handleButton():
    global current_state

    if(current_state == ROBOT_IDLE):
        print('IDLE -> SEARCHING') ## Pro-tip: print out state _transitions_
        current_state = ROBOT_SEARCHING
        left_motor.spin(FORWARD, 30)
        right_motor.spin(FORWARD, -30)

        ## start the timer for the camera
        cameraTimer.event(cameraTimerCallback, cameraInterval)

    else: ## failsafe; go to IDLE from any other state when button is pressed
        print(' -> IDLE')
        current_state = ROBOT_IDLE
        left_motor.stop()
        right_motor.stop()

controller_1.buttonL1.pressed(handleButton)

def cameraTimerCallback():
    global current_state
    global missedDetections

    ## Here we use a checker-handler, where the checker checks if there is a new object detection.
    ## We don't use a "CheckForObjects()" function because take_snapshot() acts as the checker.
    ## It returns a non-empty list if there is a detection.
    objects = ai_vision_15.take_snapshot(ai_vision_15__apricot)
    if objects: handleObjectDetection()

    # restart the timer
    if(current_state != ROBOT_IDLE):
        cameraTimer.event(cameraTimerCallback, cameraInterval)
# wait(2, SECONDS)
def handleObjectDetection():
    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    object1 = ai_vision_15.take_snapshot(ai_vision_15__apricot)
    object2 = ai_vision_15.take_snapshot(ai_vision_15__lime)
    object3 = ai_vision_15.take_snapshot(ai_vision_15__grape)
    global current_state
    global object_timer
    cx = object1[0].centerX
    cy = object1[0].centerY


    # ## TODO: Add code to print out the coordinates and size
    brain.screen.print("(" + str(cx) + ", " + str(cy) + ") ID: Apricot")
    brain.screen.next_row()
    brain.screen.print("(" + str(object2[0].centerX) + ", " + str(object2[0].centerY) + ") ID: Lime")
    brain.screen.next_row()
    brain.screen.print("(" + str(object3[0].centerX) + ", " + str(object3[0].centerY) + ") ID: Grape")

    if current_state == ROBOT_SEARCHING:
        print('SEARCHING -> APPROACHING')
        current_state = ROBOT_APPROACHING

    if current_state == ROBOT_APPROACHING:

        target_x = 160
        K_x = 0.2

        error = cx - target_x
        turn_effort = K_x * error

        if object1[0].width > 110 and object1[0].score >= 98:
            left_motor.spin_for(FORWARD, 0.3, TURNS,wait=False)
            right_motor.spin_for(FORWARD, 0.3, TURNS, wait=False)
            current_state = ELEVATE
            wait(1, SECONDS)
            left_motor.stop()
            right_motor.stop()
        else: 
            left_motor.spin(FORWARD, 20 + turn_effort)
            right_motor.spin(FORWARD, 20 - turn_effort)

    if current_state == ELEVATE:
        motor_17.spin_to_position(300, DEGREES,wait=True)
        if object1[0].height > 80:
            motor_17.stop()
            current_state = GRAB
    
    if current_state == GRAB:
        motor_17.spin_for(REVERSE, 10, DEGREES, wait=False)
        motor_19.spin_to_position(200, DEGREES)
        left_motor.spin_for(REVERSE, 1, TURNS, wait=False)
        right_motor.spin_for(REVERSE, 1, TURNS, wait=False)
        current_state = ROBOT_IDLE



## Our main loop
controller_1.buttonA.pressed(handleObjectDetection)
while True:
    pass