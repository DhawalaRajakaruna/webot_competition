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
#Red - Yellow - Pink - Brown - Green
colorOrder=["Red","Yellow","Pink","Brown","Green"]
countcolordone=0
tempPosition=0
travelside="right"
prevtravel=0  

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
def turnBack():
    global travelside
    if travelside == "right":
        left_motor.setVelocity(base_angular_V)
        right_motor.setVelocity(-base_angular_V)
    else:
        left_motor.setVelocity(-base_angular_V)
        right_motor.setVelocity(base_angular_V)       
    delay(4.5)
    stop()
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
    for _ in range(8):
        prox_readings[_]=prox_sensors[_].getValue()
    
def gotoStartRight():
    delay(0.1)
    readSensors()
    if prox_readings[2]>80:
        #print("Right Wall Detected Right Right Right Right")
        return
    while prox_readings[0]<80 and robot.step(tstep)!=-1:
        #print("inLoop")
        moveForward()
        readSensors()
        
def gotoStartLeft():
    delay(0.1)
    readSensors()
    if prox_readings[2]>80:
        #print("Left Wall Detected left left left left")
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
        print(colorArray)
        #print(prox_readings[0],prox_readings[2])
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
    global travelside
    travelside="left"
    gotoStartLeft()
    while robot.step(tstep)!=-1:
        readSensors()
        checkColor()
        print(colorArray)
        #print(prox_readings[7],prox_readings[5])
        if prox_readings[3]>80: #wall in front
            #print("Front Wall. Rotate Right")
            rotateLeftbackward()   #immeadiate turn
        else:
            if prox_readings[2]>80:
                #print("After a Left Turn")
                # turnLeftBackward()
            # elif prox_readings[2]>80: #wall on left
                #print("Wall on left. Move Forward")
                moveBackward()
            else:
                #print("No wall. Turn Left")
                turnRightBackward()
                
def delay(time):
    for _ in range(int(time * 1000 / tstep)):
        robot.step(tstep)
        
def checkColor():
    global countcolordone,travelside
    print(countcolordone)
    if countcolordone == 5:
        while robot.step(tstep)!=-1:
            print("The End !")
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
                    delay(4)
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
                delay(4)
                countcolordone=countcolordone+1
                if colorOrder[countcolordone] in colorArray[:-1]:
                    return
                else:
                    mazeTravelRightwall()
            else:
                print("Error with the code logic")
                 
                #has to do some shit
    print(colorArray) 
        
# def checkColor():
    # global countcolordone,travelside

    # if countcolordone==4:
        # stop()
        # print("The END !")
        # while robot.step(tstep)!=-1:
            # delay(1)
    # else:
        # index=len(colorArray)-1
# #-----------------------------------------------------------------------------
        # colorPicked=getColor(robot, "camera")
        # print(f"colorArray - {colorArray}")
        # print(f"countcolordone - {countcolordone}")
        # print(f"index - {index}")
        # print(colorPicked)
        # print(travelside)
        # if colorPicked != 'none':#if color is detected 
                # #append the color to the colorArray only if travelside==11 elif travelside==0 thencolorArray.pop()
            # print("adooooooooo")
            # print(travelside)
            # if travelside=="right":
                # if colorPicked not in colorArray:
                    # colorArray.append(colorPicked)  
            # else:
                # if colorPicked in colorArray:
                    # colorArray.pop()
            # #print(index)
            # if colorArray[index] == colorOrder[countcolordone] :
                # if colorArray[index] == colorOrder[countcolordone]:
                    # stop()
                    # delay(5)#stops for 5 seconds infront of the color
                    # countcolordone=countcolordone+1
                # if index == 0:
                    # return 
                # else:    
                    # if colorOrder[countcolordone] in colorArray[:-1]:
                        # if travelside=="right":
                            # print("Hotooooooooooooooo")
                            # turnBack()# need to implement this function
                            # mazeTravelLeftwall()
                        # if travelside=="left":
                            # return
                    # else:
                        # #return #topiyak atha
                        # turnBack()
                        # mazeTravelRightwall()
            # else:
                # return
        # else:
            # return
#---------------------------------------------------------------------------------  
# def getColor(robot, camera_name):
    # """
    # Displays the e-puck camera feed using OpenCV.

    # :param robot: The Robot instance controlling the simulation.
    # :param camera_name: The name of the camera device in the Webots scene tree.
    # """
    
    # if camera.getImage() is None:
        # print("Camera image is not available.")
    
    # image = camera.getImage()
    # width = camera.getWidth()
    # height = camera.getHeight()

    # image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    # img_rgb = image_array[:, :, :3]

    # tolerence = 60
    # colors = {
        # "Red": np.array([0,0,255]),
        # "Green": np.array([0,255,0]),
        # "Pink": np.array([255,0,255]),
        # "Yellow": np.array([0,255,255]),
        # "Brown": np.array([30,105,165]),
    # }   
     
    # detected_color = "none"
    # for color_name, target_color in colors.items():
        # lower_bound = np.clip(target_color - tolerence, 0, 255)
        # upper_bound = np.clip(target_color + tolerence, 0, 255)
        # mask = cv2.inRange(img_rgb, lower_bound, upper_bound)
        # if cv2.countNonZero(mask) > 5000:
            # detected_color = color_name
            # break
    # return detected_color      
#---------------------------------------------  
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
            print(f"Detected {detected_color} with area {color_area}")
            break

    return detected_color
#---------------------------------------------  
#---------------------------------------------  
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
    
    # turnBack()
    initializeSensors()
    mazeTravelRightwall()
    # while robot.step(tstep) != -1:
        # print(getColor(robot, "camera"))
        # delay(2)

    
