from controller import Robot, Motor 
from controller import Camera
TIME_STEP = 64
MAX_SPEED = 6.28
frameColor = "red"

#create robot
robot = Robot()
#get camera device
cm = robot.getDevice('camera')
# Check if the camera device is obtained successfully
if cm is not None:
    # Enable the camera with a specific time step
    TIME_STEP = 32  # Set your desired time step here
    cm.enable(TIME_STEP)

    # Capture an image
    image = cm.getImage()

    # Now you can process the 'image' variable as needed
    # For example, you might want to access the image data:
    image_data = image.getData()

    # Don't forget to release the image when you're done with it
    image.delete()
else:
    print("Error: Could not obtain the camera device.")
#get motor devices
claw1 = robot.getDevice('finger_1_joint_1')
claw2 = robot.getDevice('finger_2_joint_1')
claw3 = robot.getDevice('finger_middle_joint_1')
shoulder = robot.getDevice('shoulder_lift_joint')
elbow = robot.getDevice('elbow_joint')
wrist1 = robot.getDevice('wrist_1_joint')
wrist2 = robot.getDevice('wrist_2_joint')
#set position of motor devices
claw1.setPosition(float('inf'))
claw2.setPosition(float('inf'))
claw3.setPosition(float('inf'))
shoulder.setPosition(float('inf'))
elbow.setPosition(float('inf'))
wrist1.setPosition(float('inf'))
wrist2.setPosition(float('inf'))
#set up motor speed at 10% of MAX_SPEED
claw1.setVelocity(0.1 * MAX_SPEED)
claw2.setVelocity(0.1 * MAX_SPEED)
claw3.setVelocity(0.1 * MAX_SPEED)
shoulder.setVelocity(0.1 * MAX_SPEED)
elbow.setVelocity(0.1 * MAX_SPEED)
wrist1.setVelocity(0.1 * MAX_SPEED)
wrist1.setVelocity(0.1 * MAX_SPEED)

color = frameColor
if frameColor == "red":
    claw1.setVelocity(1.0)
    claw2.setVelocity(1.0)
    claw3.setVelocity(1.0)

while robot.step(TIME_STEP) != -1:


   pass