from controller import Robot, Camera
import time
from time import sleep
import cv2
import numpy as np
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain

robot = Robot()
TIME_STEP = 32
MAX_STEPS = 100

# Create camera
camera = robot.getDevice('camera')
# Enable the camera and recognition
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)

# Create URDF
with open("C:\\YEAR3\\FYP\\UR5e.urdf", "w") as file:
    file.write(robot.getUrdf())

ur5e = Chain.from_urdf_file("C:\\YEAR3\\FYP\\UR5e.urdf",
                            active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])
#print(ur5e.links)

ur_motors = [
    robot.getDevice("shoulder_pan_joint"),
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
    robot.getDevice("wrist_3_joint"),
]
    
position_sensors = [
    robot.getDevice("shoulder_pan_joint_sensor"),
    robot.getDevice("shoulder_lift_joint_sensor"),
    robot.getDevice("elbow_joint_sensor"),
    robot.getDevice("wrist_1_joint_sensor"),
    robot.getDevice("wrist_2_joint_sensor"),
    robot.getDevice("wrist_3_joint_sensor"),
]

#establish target positions
target_positions = [  
    [0, -1.57, 1.57, -1.57, -1.57, 0.0]
]

#enable sensors
for sensor in position_sensors:
    sensor.enable(TIME_STEP)

for motor in ur_motors:
    motor.setPosition(0.0) #set position to 0 for all motors in list
    motor.setVelocity(1.0) #set velocity for movement

# Function to check for the colour yellow
def check_for_yellow(image):
    print("checking for yellow objects")
    # Define lower and upper bounds for yellow color in HSV
    lower_yellow = np.array([90, 100, 100])
    upper_yellow = np.array([110, 255, 255])

    # Convert image from BGR to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create a mask to extract yellow regions
    yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

    # Check if any yellow pixels are present in the image
    return cv2.countNonZero(yellow_mask) > 0

# Capture an image from the camera
image = camera.getImageArray()

# Check if the image is not None
if image is not None:
    # Convert the image array to a NumPy array
    image_np = np.array(image, dtype=np.uint8)

    # Call the function to check for yellow color
    if check_for_yellow(image_np):
        print("Yellow color detected")
    else:
        print("Yellow color not detected")
else:
    print("Error: Unable to capture camera image")
            # Wait for the arm to reach the target position
    sleep(1)

#joint position function
def move_to_joint_position(target_positions):
    print("moving to joint positions")
    for target_position in target_positions:
        # move arm toward the target position
        for i, motor in enumerate(ur_motors):
            motor.setPosition(target_position[i])
            
        sleep(1) #blocking behaviour
        
        #step through simulation using MAX_STEPS
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)

            #check if the robot is close to target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target joint position.")
                break #exit loop
        else:
            print("Max steps reached. Robot did not reach the target joint position.")
            return  # Exit the outer loop if max steps are reached without reaching the target
#function loop
move_to_joint_position(target_positions)

def move_end_effector(x, y, z, target_orientation=[0,0,0]):
    print("moving end effector")
    # Inverse kinematics calculation
    IKPY_MAX_ITERATIONS = 4
    initial_position = [0] + [m.getPositionSensor().getValue() for m in ur_motors] + [0, 1, 1, -1]
    target_pose = [x, y, z] + target_orientation
    ikResults = ur5e.inverse_kinematics(target_pose, max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    # Check if the inverse kinematics solution is valid
    if ikResults is not None:
        # Setting positions based on Inverse Kinematics
        for i, motor in enumerate(ur_motors):
            motor.setPosition(ikResults[i + 1])  # Skip the first element

        # Wait for the arm to reach the target position
        sleep(1)

        # Step through simulation using MAX_STEPS
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)

            # Check if the robot is close to the target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target end effector position.")
                break  # Exit loop if the target position is reached
        else:
            print("Max steps reached. Robot did not reach the target end effector position.")

# Move end effector to the specified position
move_end_effector(0.3, 0.5, 0.3, target_orientation=[0, -1.57, 1.57, -1.57, -1.57, 0.0])


# Add a small delay to allow the simulation to catch up
sleep(0.1)

def look_for_yellow_object():
    print("looking for yellow objects")
    # Define the table's center and radius
    table_center = (0.73, 0.26, 0.37)

    # Define the search areas
    search_areas = [
        # Area 1: x cm to the right and y cm in front of the table's center
        (table_center[0] + 0.05, table_center[1] + 0.05, 0.1, 0.1),
        # Area 2: x cm to the left and y cm behind the table's center
        (table_center[0] - 0.1, table_center[1] - 0.5, 0.1, 0.1),
        # Area 3: x cm to the left and y cm above the table's center
        (table_center[0] - 0.025, table_center[1] + 0.75, 0.1, 0.1)
    ]

    # Move the end effector to the initial position
    move_end_effector(*table_center)

    # Loop through the search areas
    for area in search_areas:
        # Calculate the target position for the end effector
        target_x, target_y, target_width, target_height = area
        target_z = table_center[2]  # Keep the same z-coordinate as the table's center
        move_end_effector(target_x, target_y, target_z)

        # Capture an image from the camera
        image = camera.getImageArray()

        # Check if the image is not None
        if image is not None:
            # Convert the image array to a NumPy array
            image_np = np.array(image, dtype=np.uint8)
                    # Display the camera feed in a window
            cv2.imshow('Camera Feed', image_np)
            # Call the function to check for yellow color
            if check_for_yellow(image_np):
                print(f"Yellow color detected in area: {area}")
            else:
                print(f"Yellow color not detected in area: {area}")
        else:
            print("Error: Unable to capture camera image")

    # Move the end effector back to the initial position
    move_end_effector(*table_center)
# Call the function to look for yellow object
look_for_yellow_object()

#spiral search
#move orientatio to search 
