from controller import Robot

def allglobal():
    global initenL,initenR,tstep, errorsum, lasterror, kp, kd, ki, base_angular_V, wheelRadius, trackWidth
    errorsum = 0
    lasterror = 0
    kp = 0.5  # Proportional gain
    kd = 0.1  # Derivative gain
    ki = 0.05  # Integral gain
    base_angular_V = 6.28  # rad/s
    wheelRadius = 0.0205  # meters
    trackWidth = 0.053  # Distance between the robot's wheels in meters
    initenL=0
    initenR=0

def enableEncoders():
    left_encoder.enable(tstep)
    right_encoder.enable(tstep)

def disableEncoders():
    left_encoder.disable()
    right_encoder.disable()
    
def moveforward(distance):
    linear_velocity = base_angular_V * wheelRadius 
    time_required = distance / linear_velocity 
    left_motor.setVelocity(base_angular_V)
    right_motor.setVelocity(base_angular_V)
    delay(time_required+0.05)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    return

def turnRight():
    enableEncoders()
    delay(0.1)
    initenL=left_encoder.getValue()
    initenR=right_encoder.getValue()
    print(f"{initenL}  -  {initenR}")
    enL=0
    enR=0
    while True:
        print(f"{enL}  -  {enR}")
        left_motor.setVelocity(base_angular_V)
        right_motor.setVelocity(-base_angular_V)    
        if enL>10.5 and -1*enR>10.5:
            break
        enL=left_encoder.getValue()-initenL
        enR=right_encoder.getValue()-initenR
        delay(0.1)# computer stucking error occured here
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()
    return
def delay(time):
    for _ in range(int(time * 1000 / tstep)):  # Convert time to steps
        robot.step(tstep)
        
if __name__ == "__main__":
    allglobal()

    robot = Robot()
    tstep = 64

    # Access motors and encoders
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')   

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    
    left_motor.setPosition(float('inf'))  
    right_motor.setPosition(float('inf')) 
    
    left_motor.setVelocity(0)  
    right_motor.setVelocity(0) 
    num=0
    while True:
        moveforward(0.25)
        turnRight()
        if num==10:
            break
        num=num+1