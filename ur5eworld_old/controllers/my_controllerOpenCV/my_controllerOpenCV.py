from controller import Robot
import numpy as np
import cv2
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
import time
from time import sleep

robot = Robot()
TIME_STEP = 32
MAX_STEPS = 150

# Create camera
camera = robot.getDevice('camera')
# Enable the camera and recognition
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# Create URDF
with open("C:\\YEAR3\\FYP\\UR5e.urdf", "w") as file:
    file.write(robot.getUrdf())

joint_names = [
    'base_link',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
    'wrist_3_link_ROBOTIQ_3f_Gripper_joint',
    'finger_middle_joint_1',
    'finger_middle_joint_2',
    'finger_middle_joint_3'
]

# Create IK chain
ur5e = Chain.from_urdf_file("C:\\YEAR3\\FYP\\UR5e.urdf",
                            active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])
#print(ur5e.links)

# Get motors for each joint
ur_motors = [
    robot.getDevice("shoulder_pan_joint"),
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
    robot.getDevice("wrist_3_joint")
]

# Enable position sensors for each joint
position_sensors = [
    robot.getDevice("shoulder_pan_joint_sensor"),
    robot.getDevice("shoulder_lift_joint_sensor"),
    robot.getDevice("elbow_joint_sensor"),
    robot.getDevice("wrist_1_joint_sensor"),
    robot.getDevice("wrist_2_joint_sensor"),
    robot.getDevice("wrist_3_joint_sensor"),
]
#enable sensors
for sensor in position_sensors:
    sensor.enable(TIME_STEP)

for motor in ur_motors:
    motor.setPosition(0.0) #set position to 0 for all motors in list
    motor.setVelocity(1.0) #set velocity for movement
    
 #establish target positions
target_positions = [
    [0, -1.57, 1.57, -1.57, -1.57, 0.0]
]  

#joint position function
def move_to_joint_position(target_positions):
    for target_position in target_positions:
        # move arm toward the target position
        for i, motor in enumerate(ur_motors):
            motor.setPosition(target_position[i])
        
        #step through simulation
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)
            
            # Get image from the camera
            image = camera.getImageArray()

            if image is not None: #if image is valid
                # Convert the image array to an OpenCV format
                image = np.array(image, dtype=np.uint8)

                # display the image using OpenCV
                image = cv2.resize(image, (200, 200))
                cv2.imshow("Camera Image", image)
                cv2.waitKey(1)  # delay

            #check if the robot is close to target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target joint position.")
                break #exit loop
        else:
            print("Max steps reached. Robot did not reach the target joint position.")
            return  # Exit the loop if max steps are reached without reaching the target

# function loop
move_to_joint_position(target_positions)

# Function to detect a yellow object
def detect_yellow_object(image):
    # Convert the image from BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the yellow color
    lower_yellow = np.array([90, 100, 100])
    upper_yellow = np.array([110, 255, 255])

    # Threshold image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Check if any contours are found
    if contours:
        return True  # Found a yellow object
    else:
        return False  # No yellow object found

# Function to move the end effector to a specific position (x, y, z)
def move_end_effector(x, y, z):
    # Inverse kinematics calculation
    IKPY_MAX_ITERATIONS = 4
    initial_position = [0] + [m.getPositionSensor().getValue() for m in ur_motors] + [0, 1, 1, -1]
    ikResults = ur5e.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    # Check if the inverse kinematics solution is valid
    if ikResults is not None:
        # Setting positions based on Inverse Kinematics
        for i, motor in enumerate(ur_motors):
            motor.setPosition(ikResults[i + 1])  # Skip the first element

        # Wait for the arm to reach the target position
        sleep(1)

# Step through simulation
    for step in range(MAX_STEPS):
        robot.step(TIME_STEP)

        # Get image from the camera
        image = camera.getImageArray()

        if image is not None:
            # Convert image array to an OpenCV format
            image = np.array(image, dtype=np.uint8)

            # Resize the image
            image = cv2.resize(image, (200, 200))

            # Display image using OpenCV
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)  # delay

            # Check if a yellow object is detected
            if detect_yellow_object(image):
                print("Found yellow object!")
                break  # Exit loop if a yellow object is found

    else:
        print("Max steps reached. No yellow object found.")

# Function loop
move_end_effector(0.30, -0.20, 0.77)

joint_positions = [sensor.getValue() for sensor in position_sensors]
print("Joint positions:", joint_positions)