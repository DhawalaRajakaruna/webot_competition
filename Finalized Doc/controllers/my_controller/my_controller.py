from controller import Robot
from time import sleep
import cv2
import numpy as np

robot = Robot()

# Constants
base_angular_V = 6.28 
wheelRadius = 0.0205  # meters
trackWidth = 0.053  # Distance between the robot's wheels in meters
tstep = int(robot.getBasicTimeStep()) # Time step for Webots controller (ms)

# Global Variables
prox_sensors=[]
prox_readings=[0,0,0,0,0]
colorArray=[]
#Red - Yellow - Pink - Brown - Green
colorOrder=["Red","Yellow","Pink","Brown","Green"]
countcolordone=0
travelside="right" 

# Helper Functions
def initializeSensors():    
    for ind in range(5):
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
    
def moveBackward():
    left_motor.setVelocity(-base_angular_V)
    right_motor.setVelocity(-base_angular_V)

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
    
def turnLeftBackward():
    left_motor.setVelocity(-base_angular_V/8)
    right_motor.setVelocity(-base_angular_V)   
    
def turnRightBackward():
    left_motor.setVelocity(-base_angular_V)
    right_motor.setVelocity(-base_angular_V/8)    

def rotateLeftbackward():
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(-base_angular_V)         
  
def readSensors():
    for _ in range(5):
        prox_readings[_]=prox_sensors[_].getValue()
    
def gotoStartRight():
    delay(0.1)
    readSensors()
    if prox_readings[2]>80:
        return
    while prox_readings[0]<80 and robot.step(tstep)!=-1:
        moveForward()
        readSensors()
        
def gotoStartLeft():
    delay(0.1)
    readSensors()
    if prox_readings[2]>80:
        return
    while prox_readings[3]<80 and robot.step(tstep)!=-1:
        moveBackward()
        readSensors()

def mazeTravelRightwall():
    global travelside
    travelside="right"
    gotoStartRight()
    while robot.step(tstep)!=-1:
        readSensors()
        checkColor()
        if prox_readings[0]>80: #wall in front
            rotateLeft()   #immeadiate turn
        else:
            if prox_readings[1]>80:
                turnLeft()
            elif prox_readings[2]>80: #wall on right
                moveForward()
            else:
                turnRight()
              
def mazeTravelLeftwall():
    global travelside
    travelside="left"
    gotoStartLeft()
    while robot.step(tstep)!=-1:
        readSensors()
        checkColor()
        if prox_readings[4]>80: #wall in front            
            rotateLeftbackward()   #immeadiate turn
        else:
            if prox_readings[3]>80:                
                turnLeftBackward()
            elif prox_readings[2]>80: #wall on left                
                moveBackward()
            else:               
                turnRightBackward()
                                
def delay(time):
    for _ in range(int(time * 1000 / tstep)):
        robot.step(tstep)
        
def checkColor():
    global countcolordone,travelside
    print(countcolordone)
    print(colorArray)
    colorPicked=colorPicked=getColor(robot, "camera")
    if colorPicked == 'none':
        return
    else:
        if travelside == "right":
            if colorPicked not in colorArray:
                colorArray.append(colorPicked)
                if colorPicked == colorOrder[countcolordone]:
                    stop()
                    delay(5)#wait for 5 seconds to show it meets the color in order
                    if countcolordone == 4:
                        while robot.step(tstep)!=-1:
                            print("The End !")
                    countcolordone=countcolordone+1
                    if colorOrder[countcolordone] in colorArray[:-1]:
                        mazeTravelLeftwall()
                    else:
                        return
                else:
                    return
        elif travelside== "left":
            if colorPicked == colorOrder[countcolordone-1] and colorPicked in colorArray:
                colorArray.pop()
                return
            elif colorPicked not in colorArray:
                return
            elif colorPicked == colorArray[-1] and colorPicked != colorOrder[countcolordone]:
                colorArray.pop()
                return
            elif colorPicked == colorArray[-1] and colorPicked == colorOrder[countcolordone]:
                stop()
                delay(5)#wait for 5 seconds to show it meets the color in order
                if countcolordone == 4:
                    while robot.step(tstep)!=-1:
                        print("The End !")
                countcolordone=countcolordone+1
                if colorOrder[countcolordone] in colorArray[:-1]:
                    return
                else:
                    mazeTravelRightwall()
            else:
                return
                
#correct color returning code
def getColor(robot, camera_name):
    """
    Displays the e-puck camera feed using OpenCV.

    :param robot: The Robot instance controlling the simulation.
    :param camera_name: The name of the camera device in the Webots scene tree.
    """
    if camera.getImage() is None:
        print("Camera image is not available.")
        return "none"
    
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    img_rgb = image_array[:, :, :3]

    tolerance = 60
    colors = {
        "Red": np.array([0, 0, 255]),
        "Green": np.array([0, 255, 0]),
        "Pink": np.array([255, 0, 255]),
        "Yellow": np.array([0, 255, 255]),
        "Brown": np.array([30, 105, 165]),
    }   
    
    detected_color = "none"
    pixel_threshold = 1000 # Lower detection threshold for color patches

    for color_name, target_color in colors.items():
        lower_bound = np.clip(target_color - tolerance, 0, 255)
        upper_bound = np.clip(target_color + tolerance, 0, 255)
        mask = cv2.inRange(img_rgb, lower_bound, upper_bound)
        color_area = cv2.countNonZero(mask)  # Count pixels matching the color range
        
        if color_area > pixel_threshold:  # Detect only if color area exceeds the lower threshold
            detected_color = color_name
            break

    return detected_color
# Main Execution
if __name__ == "__main__":

    # Access motors and encoders
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')   

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    
        # Initialize the e-puck's camera
    camera = robot.getDevice("camera")
    camera.enable(tstep)
    
    # Set motors to velocity control mode
    left_motor.setPosition(float('inf'))  
    right_motor.setPosition(float('inf')) 
    
    left_motor.setVelocity(0)  
    right_motor.setVelocity(0) 
    
    initializeSensors()
    mazeTravelRightwall()


    
