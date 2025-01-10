from controller import Robot


def allglobal():
    global tstep, errorsum, lasterror, kp, kd, ki, base_angular_V, wheelRadius, trackWidth
    errorsum = 0
    lasterror = 0
    kp = 0.5  # Proportional gain
    kd = 0.1  # Derivative gain
    ki = 0.05  # Integral gain
    base_angular_V = 6.28  # rad/s
    wheelRadius = 0.0205  # meters
    trackWidth = 0.053  # Distance between the robot's wheels in meters


def delay(time):
    for _ in range(int(time * 1000 / tstep)):  # Convert time to steps
        robot.step(tstep)


def enableEncoders():
    left_encoder.enable(tstep)
    right_encoder.enable(tstep)


def disableEncoders():
    left_encoder.disable()
    right_encoder.disable()


def resetEncoders():
    left_encoder.getValue()  # Clear the encoder values
    right_encoder.getValue()


def moveForward(distance):
    """
    Moves the robot forward for the given distance (in meters).
    """
    enableEncoders()
    resetEncoders()

    target_rotation = distance / wheelRadius  # Target rotation in radians
    while True:
        enL = left_encoder.getValue()
        enR = right_encoder.getValue()

        if enL >= target_rotation and enR >= target_rotation:
            break

        left_motor.setVelocity(base_angular_V)
        right_motor.setVelocity(base_angular_V)
        print(f"Encoders (Forward): Left={enL}, Right={enR}")

    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    disableEncoders()

if __name__ == "__main__":
    allglobal()

    robot = Robot()
    tstep = int(robot.getBasicTimeStep())

    # Access motors and encoders
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))  # Velocity control mode
    right_motor.setPosition(float('inf'))

    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')

    # Move forward 0.25 meters
    print("Hellow")
    moveForward(0.25)


