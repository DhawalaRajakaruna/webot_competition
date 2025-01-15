from controller import Robot
from time import sleep

robot = Robot()

# Constants
base_angular_V = 6.28 
wheelRadius = 0.0205  # meters
trackWidth = 0.053  # Distance between the robot's wheels in meters
kp = 0.5  # Proportional gain
kd = 0.1  # Derivative gain
ki = 0.05  # Integral gain
tstep = int(robot.getBasicTimeStep()) # Time step for Webots controller (ms)

# Global Variables
initenL = 0
initenR = 0
lasterror = 0
errorsum = 0
enL = 0
enR = 0
prox_sensors=[]
prox_readings=[0,0,0,0,0,0,0,0]
colorArray=[]
colorOrder=["Red","Yellow","Pink","Brown","Green"]
countcolordone=0
tempPosition=0
travelside=0
prevtravelside=0

# Helper Functions
def initializeSensors():
    for ind in range(8):
        sensor='ps'+str(ind)
        prox_sensors.append(robot.getDevice(sensor))
        prox_sensors[ind].enable(tstep)
        
def enableEncoders():
    left_encoder.enable(tstep)
    right_encoder.enable(tstep)

def disableEncoders():
    left_encoder.disable()
    right_encoder.disable()                            

def moveForward():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(base_angular_V)

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


def turnRight():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(base_angular_V/8)

def turnLeft():

    left_motor.setVelocity(base_angular_V/8)
    right_motor.setVelocity(base_angular_V)

def rotateRight():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(-base_angular_V)

def rotateLeft():
 
    left_motor.setVelocity(-base_angular_V)
    right_motor.setVelocity(base_angular_V)
    
def readSensors():
    for _ in range(8):
        prox_readings[_]=prox_sensors[_].getValue()
    
def gotoStartRight():
    delay(0.1)
    readSensors()
    if prox_readings[2]>80:
        print("Right Wall Detected Right Right Right Right")
        return
    while prox_readings[0]<80 and robot.step(tstep)!=-1:
        print("inLoop")
        moveForward()
        readSensors()
        
def gotoStartLeft():
    delay(0.1)
    readSensors()
    if prox_readings[5]>80:
        print("Left Wall Detected left left left left")
        return
    while prox_readings[7]<80 and robot.step(tstep)!=-1:
        moveForward()
        readSensors()

def mazeTravelRightwall():
    travelside=11
    gotoStartRight()
    print("goto called")
    while robot.step(tstep)!=-1:
        readSensors()
        checkColor()
        print(prox_readings[0],prox_readings[2])
        if prox_readings[0]>80: #wall in front
            print("Front Wall. Rotate Left")
            rotateLeft()   #immeadiate turn
        else:
            if prox_readings[1]>80:
                print("After a Right Turn")
                turnLeft()
            elif prox_readings[2]>80: #wall on right
                print("Wall on Right. Move Forward")
                moveForward()
            else:
                print("No wall. Turn Right")
                turnRight()
              
def mazeTravelLeftwall():
    travelside=00
    gotoStartLeft()
    while robot.step(tstep)!=-1:
        readSensors()
        checkColor()
        print(prox_readings[7],prox_readings[5])
        if prox_readings[7]>80: #wall in front
            print("Front Wall. Rotate Right")
            rotateRight()   #immeadiate turn
        else:
            if prox_readings[6]>80:
                print("After a Left Turn")
                turnRight()
            elif prox_readings[5]>80: #wall on left
                print("Wall on left. Move Forward")
                moveForward()
            else:
                print("No wall. Turn Left")
                turnLeft()
                
def delay(time):
    for _ in range(int(time * 1000 / tstep)):
        robot.step(tstep)
        
def checkColor():
    if countcolordone==4:
        stop()
        while robot.step(tstep)!=-1:
            delay(1)
    else:
        index=len(colorArray)
#-----------------------------------------------------------------------------
        #if color is detected 
            #append the color to the colorArray only if travelside==11 elif travelside==0 thencolorArray.pop()
        if colorArray(index) == colorOrder[countcolordone] :
            stop()
            delay(5)#stops for 5 seconds infront of the color
            countcolordone+=countcolordone
            if index == 0:
                return 
            else:    
                if colorOrder[countcolordone] in colorArray[:-1]:
                    if travelside==11:
                        turnBack()# need to implement this function
                        mazeTravelLeftwall()
                    if travelside==00:
                        return
                else:
                    turnBack()
                    mazeTravelRightwall()
        else:
            return
#---------------------------------------------------------------------------------  
          
# Main Execution
if __name__ == "__main__":

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
    
    initializeSensors()
    mazeTravelRighttwall()
    
