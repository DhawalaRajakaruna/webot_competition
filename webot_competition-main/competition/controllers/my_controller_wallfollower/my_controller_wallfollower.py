from controller import Robot
from time import sleep
import cv2
import numpy as np
robot = Robot()

# Constants
base_angular_V = 6.28 
wheelRadius = 0.0205  # meters
trackWidth = 0.053  # Distance between the robot's wheels in meters
kp = 0.5  # Proportional gain
kd = 0.1  # Derivative gain
ki = 0.05  # Integral gain
tstep = int(robot.getBasicTimeStep()) # Time step for Webots controller (ms)

#colors

red = np.array([0,0,255])
yellow = np.array([0,255,255])
green = np.array([0,255,0])
pink = np.array([255,0,255])
brown = np.array([30,105,165])
color_order = [red,yellow,pink,brown,green]
colors = ["red","yellow","pink","brown","green"]
tolerence = 60
 
# Global Variables
initenL = 0
initenR = 0
lasterror = 0
errorsum = 0
enL = 0
enR = 0
prox_sensors=[]
prox_readings=[0,0,0,0,0,0,0,0]

camera = robot.getDevice("camera")
camera.enable(tstep)

# Helper Functions
def initializeSensors():
    
    for ind in range(8):
        sensor='ps'+str(ind)
        prox_sensors.append(robot.getDevice(sensor))
        prox_sensors[ind].enable(tstep)
#camera functions
def use_camera():
    image =camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    image_array = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
    img_bgr = cv2.cvtColor(image_array,cv2.COLOR_RGB2BGR)
    img_rgb = image_array[:,:,:3]
    print(img_rgb)
    target_color = color_order[0]
    lower_bound = np.clip(target_color-tolerence,0,255)
    upper_bound = np.clip(target_color+tolerence,0,255)
    mask = cv2.inRange(img_rgb,lower_bound,upper_bound)

    new_height = int(height*6)
    new_width = int(width*6)
    img_resize = cv2.resize(img_rgb,(new_width,new_height),interpolation=cv2.INTER_LINEAR)

    
    return img_resize, mask

    
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
    
def gotoStart():
    readSensors()
    if prox_readings[2]>90:
        return
    while prox_readings[0]<90 and robot.step(tstep)!=-1:
        moveForward()
        readSensors()


def mazeTravelRightwall():
    gotoStart()
    while robot.step(tstep)!=-1:
        img_resize,mask = use_camera()
        if np.any(mask) and prox_readings[0]>80:###############################
            stop()
            print("{} found".format(colors[0]))
            sleep(3)
            colors.pop(0)
            color_order.pop(0)
            if len(colors)==0:
                break 
        print(colors)
        cv2.imshow("camera",img_resize)
        cv2.waitKey(1)
        readSensors()
        print(prox_readings[0],prox_readings[2])
        if prox_readings[0]>80: #wall in front
            #print("Front Wall. Rotate Left")
            rotateLeft()   #immeadiate turn
        else:
            if prox_readings[1]>80:
                #print("After a Right Turn")
                turnLeft()
            elif prox_readings[2]>80: #wall on right
                #print("Wall on Right. Move Forward")
                moveForward()
            else:
                #print("No wall. Turn Right")
                turnRight()
           

def mazeTravelLeftwall():
    gotoStart()
    while robot.step(tstep)!=-1:
        readSensors()
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
   
    mazeTravelRightwall()
        