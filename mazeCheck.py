from controller import Robot

# Constants
base_angular_V = 6.28 
wheelRadius = 0.0205  # meters
trackWidth = 0.053  # Distance between the robot's wheels in meters
kp = 0.5  # Proportional gain
kd = 0.1  # Derivative gain
ki = 0.05  # Integral gain
tstep = 64  # Time step for Webots controller (ms)

# Global Variables
initenL = 0
initenR = 0
lasterror = 0
errorsum = 0
enL = 0
enR = 0

# Helper Functions
def enableEncoders():
    left_encoder.enable(tstep)
    right_encoder.enable(tstep)

def disableEncoders():
    left_encoder.disable()
    right_encoder.disable()

def delay(time):
    for _ in range(int(time * 1000 / tstep)):
        robot.step(tstep)

def encoderPID(n):
    global enL, enR, lasterror, errorsum
    enL = left_encoder.getValue() - initenL
    enR = right_encoder.getValue() - initenR
    error = abs(enL) - abs(enR)
    print(f"enL - {enL} , enR - {enR}")
    print(f"error - {error}")
    errordif = error - lasterror
    errorsum += error
    lasterror = error  
    correction = kp * error + ki * errorsum + kd * errordif
    #correction = max(min(correction, base_angular_V), -base_angular_V)
    print(f"correction - {correction}")

    if n == 1:
        leftVelocity = base_angular_V - correction
        rightVelocity = -(base_angular_V + correction)
    if n == 2:
        leftVelocity = -(base_angular_V - correction)
        rightVelocity = base_angular_V + correction  
    if n == 0:
        leftVelocity = base_angular_V - correction
        rightVelocity = base_angular_V + correction  
    elif n == -1:
        leftVelocity = rightVelocity = 0
    leftVelocity = max(min(leftVelocity, base_angular_V), -base_angular_V)
    rightVelocity = max(min(rightVelocity, base_angular_V), -base_angular_V)
    
    left_motor.setVelocity(leftVelocity)
    right_motor.setVelocity(rightVelocity)

def turnRight():
    global initenL, initenR, enL, enR, lasterror, errorsum
    enableEncoders()
    delay(0.1) 

    # Initialize variables
    initenL = left_encoder.getValue()
    initenR = right_encoder.getValue()
    enL = 0
    enR = 0
    lasterror = 0
    errorsum = 0

    while True:
        print("hellow")
        encoderPID(1)  # Perform PID correction
        if abs(enL) >= 11.20 and abs(enR) >= 11.20:  # Encoder thresholds for turning
            break
        delay(0.1)#error here
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()

def turnLeft():
    global initenL, initenR, enL, enR, lasterror, errorsum
    enableEncoders()
    delay(0.1) 

    # Initialize variables
    initenL = left_encoder.getValue()
    initenR = right_encoder.getValue()
    enL = 0
    enR = 0
    lasterror = 0
    errorsum = 0

    while True:
        print("hellow")
        encoderPID(2)  # Perform PID correction
        if abs(enL) >= 11.20 and abs(enR) >= 11.20:  # Encoder thresholds for turning
            break
        delay(0.1)#error here
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()
    
def moveforward():
    global initenL, initenR, enL, enR, lasterror, errorsum
    enableEncoders()
    delay(0.1) 
    # Initialize variables
    initenL = left_encoder.getValue()
    initenR = right_encoder.getValue()
    enL = 0
    enR = 0
    lasterror = 0
    errorsum = 0
    while True:
        print("hellow")
        encoderPID(0)  # Perform PID correction
        if abs(enL) >= 11.20*mulnum and abs(enR) >= 11.20*mulnum:  # Encoder thresholds for turning
            break
        delay(0.1)#error here
    delay(0.2*mulnum)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()

def turnBack(directFrom):
    #if directFrom = 1 turn from right if directFrom = 2 turn from right
    global initenL, initenR, enL, enR, lasterror, errorsum
    enableEncoders()
    delay(0.1) 

    # Initialize variables
    initenL = left_encoder.getValue()
    initenR = right_encoder.getValue()
    enL = 0
    enR = 0
    lasterror = 0
    errorsum = 0

    while True:
        print("hellow")
        encoderPID(directFrom)  # Perform PID correction
        if isfrontWallDetected():  # Encoder thresholds for turning
            solveMaze()
            break
        delay(0.1)#error here
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()    
# Main Execution

def isfrontWallDetected():
    # have to write the code to identofy the front wall 
    return False

def solveMaze():
    #check for side walls



if __name__ == "__main__":
    robot = Robot()

    # Access motors and encoders
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')   

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')

    # Set motors to velocity control mode
    left_motor.setPosition(float('inf'))  
    right_motor.setPosition(float('inf')) 
    
    left_motor.setVelocity(0)  
    right_motor.setVelocity(0) 

    # Perform a right turn
    moveforward() 
