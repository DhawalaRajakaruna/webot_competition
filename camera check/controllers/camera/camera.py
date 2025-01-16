from controller import Robot, Camera

robot = Robot()
timestep = int(robot.getBasicTimeStep())


camera = robot.getDevice('camera')  # Replace 'camera' with the name of your camera in Webots
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()
print(f"Camera resolution: {width}x{height}")

while robot.step(timestep) != -1:
    # Get the current image from the camera
    image = camera.getImage()
    
    # Check if the image is captured
    if image:
        print("Camera is working and capturing images.")

