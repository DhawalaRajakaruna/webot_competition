from controller import Robot

def movforward(angularVelocity):

    # Enable velocity control mode for the motors
    left_motor.setPosition(float('inf'))  # Infinite position means velocity control
    right_motor.setPosition(float('inf'))
    
    # Set the motors' velocities
    left_motor.setVelocity(angularVelocity)
    right_motor.setVelocity(angularVelocity)
    return
    
def stopMotors():

    # Enable velocity control mode for the motors
    left_motor.setPosition(float('inf'))  # Infinite position means velocity control
    right_motor.setPosition(float('inf'))
    
    # Set the motors' velocities
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    return
def delay(time):
        for _ in range(time* 1000 / timestep):  # Convert `timePerOneBlock` to ms
        robot.step(timestep)
        return

def turnLeft():
    return
def turnRight():
    return
def encorderPID():
    
    return
if __name__ == "__main__":
    # Create the Robot instance
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    angularVelocity = 6.28  # rad/s
    wheelRadius = 0.0205  # meters
    
    # Calculate linear velocity and time to travel one block
    linearVelocity = angularVelocity * wheelRadius
    length_side = 0.25  # meters
    
    # Access the left and right motors by their names
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    
    timePerOneBlock = length_side / linearVelocity  # Time to travel one block (in seconds)
    
    movforward(angularVelocity)
    delay(timePerOneBlock)

    stopMotors()
    delay(2)

    # Main loop
    while robot.step(timestep) != -1:
        pass  # Continue running the simulation



