from controller import Robot
import cv2
import numpy as np

# Time step for the simulation
TIME_STEP = 32

# Initialize the robot
robot = Robot()

# Initialize the e-puck's camera
camera = robot.getDevice('camera')  # Make sure the camera name matches the one in Webots
camera.enable(TIME_STEP)

# Get camera properties
width = camera.getWidth()
height = camera.getHeight()
print(f"Camera resolution: {width}x{height}")

print("Press 'q' to quit the camera feed.")

# Main simulation loop
while robot.step(TIME_STEP) != -1:
    # Capture the current camera image
    image = camera.getImage()

    if image:
        # Convert Webots BGRA image to a NumPy array
        image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))

        # Convert BGRA to BGR for OpenCV
        bgr_image = cv2.cvtColor(image_array, cv2.COLOR_BGRA2BGR)

        # Display the image in an OpenCV window
        cv2.imshow('e-puck Camera', bgr_image)

        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Clean up
camera.disable()
cv2.destroyAllWindows()
