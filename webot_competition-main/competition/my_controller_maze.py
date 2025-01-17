"""my_controller_encoder_testing controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
############################## motors ########################################
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

WHEEL_RADIUS = 0.02
WHEEL_DISTANCE = 0.053
ANGULAR_SPEED = 1.0

TURN_90 = (WHEEL_DISTANCE * 3.14159265359 / 4) / (WHEEL_RADIUS )
TURN_180 = TURN_90 * 2
################################## IR sensors #######################################
sensor_names = ["ps7","ps2","ps5"]
proximity_sensors = []
for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    proximity_sensors.append(sensor)






###################################### functions ########################################

def turn_left_90():
   initial_left = left_encoder.getValue()
   initial_right = right_encoder.getValue()

   left_motor.setVelocity(-ANGULAR_SPEED)
   right_motor.setVelocity(ANGULAR_SPEED)

   while robot.step(timestep) != -1:
      left_change = abs(left_encoder.getValue() - initial_left)
      right_change = abs(right_encoder.getValue() - initial_right)
      if left_change > TURN_90 and right_change > TURN_90:
             break

   left_motor.setVelocity(0)
   right_motor.setVelocity(0)
   
def turn_right_90():
   initial_left = left_encoder.getValue()
   initial_right = right_encoder.getValue()
   #print("Initial left: ", initial_left, "Initial right: ", initial_right)
   left_motor.setVelocity(ANGULAR_SPEED)
   right_motor.setVelocity(-ANGULAR_SPEED)

   while robot.step(timestep) != -1:
      left_change = abs(left_encoder.getValue() - initial_left)
      right_change = abs(right_encoder.getValue() - initial_right)
      #print("Left change: ", left_change, "Right change: ", right_change)
      if left_change > TURN_90 and right_change > TURN_90:
             break

   left_motor.setVelocity(0)
   right_motor.setVelocity(0)

def turn_back():
    initial_left = left_encoder.getValue()
    initial_right = right_encoder.getValue()
    
    left_motor.setVelocity(ANGULAR_SPEED)
    right_motor.setVelocity(-ANGULAR_SPEED)
    
    while robot.step(timestep) != -1:
        left_change = abs(left_encoder.getValue() - initial_left)
        right_change = abs(right_encoder.getValue() - initial_right)
        if left_change > TURN_180 and right_change > TURN_180:
                 break
    
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
def move_forward():
    left_motor.setVelocity(ANGULAR_SPEED)
    right_motor.setVelocity(ANGULAR_SPEED)

    

def move_backward():
    left_motor.setVelocity(-ANGULAR_SPEED)
    right_motor.setVelocity(-ANGULAR_SPEED)

    

def motor_stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# Main loop:
while robot.step(timestep) != -1:
    motor_stop()
    sensor_values = [1 if sensor.getValue() > 80 else 0 for sensor in proximity_sensors]
    print(sensor_values)
    if sensor_values[0]==1:#front wall
        turn_right_90()
    else:
        if sensor_values[1]==1:#right wall
            move_forward()
        elif sensor_values
    # move_forward()
    # robot.step(5000)
    # motor_stop()
    # turn_left_90()
    # robot.step(100)
    # motor_stop()
    # move_forward()
    # robot.step(5000)
    # motor_stop()
    # turn_right_90()
    # robot.step(100)
    # motor_stop()
    