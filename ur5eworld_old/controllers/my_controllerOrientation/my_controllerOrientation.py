from controller import Robot, Camera
import time
from time import sleep
import cv2
import numpy as np
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
import math
import random 

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
#load  chain
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

finger_motors = [
    robot.getDevice("palm_finger_1_joint"),
    robot.getDevice("finger_1_joint_1"),
    robot.getDevice("finger_1_joint_2"),
    robot.getDevice("finger_1_joint_3"),
    robot.getDevice("palm_finger_2_joint"),
    robot.getDevice("finger_2_joint_1"),
    robot.getDevice("finger_2_joint_2"),
    robot.getDevice("finger_2_joint_3"),
    robot.getDevice("finger_middle_joint_1"),
    robot.getDevice("finger_middle_joint_2"),
    robot.getDevice("finger_middle_joint_3"),
]

finger_sensors = [
    robot.getDevice("palm_finger_1_joint_sensor"),
    robot.getDevice("finger_1_joint_1_sensor"),
    robot.getDevice("finger_1_joint_2_sensor"),
    robot.getDevice("finger_1_joint_3_sensor"),
    robot.getDevice("palm_finger_2_joint_sensor"),
    robot.getDevice("finger_2_joint_1_sensor"),
    robot.getDevice("finger_2_joint_2_sensor"),
    robot.getDevice("finger_2_joint_3_sensor"),
    robot.getDevice("finger_middle_joint_1_sensor"),
    robot.getDevice("finger_middle_joint_2_sensor"),
    robot.getDevice("finger_middle_joint_3_sensor")
]

#establish target positions
target_positions = [  
    [0, -1.57, 1.57, -1.57, -1.57, 0.0],
    [0.0, -1.0, 1.0, -1.0, -1.0, 0.0],
    [1.0, -0.5, 1.0, -0.5, -1.0, 0.5],
    [0, -1.57, 1.57, -1.57, -1.57, 0.0],

]

#enable sensors
for sensor in position_sensors:
    sensor.enable(TIME_STEP)

for motor in ur_motors:
    motor.setPosition(0.0) #set position to 0 for all motors in list
    motor.setVelocity(1.0) #set velocity for movement


# Function to compute and print the orientation of the end effector
def print_end_effector_orientation(ur_motors):
    # Get the joint positions from the motor sensors
    active_joint_positions = [motor.getPositionSensor().getValue() for motor in ur_motors]

    # Fill inactive joint positions with zeros
    all_joint_positions = active_joint_positions + [0] * (len(ur5e.links) - len(active_joint_positions))

    # Compute the forward kinematics to get the end effector pose
    end_effector_pose = ur5e.forward_kinematics(all_joint_positions)

    # Extract the orientation matrix from the pose
    orientation = end_effector_pose[:3, :3]

    # Print the orientation
    print("End effector orientation:")
    print(orientation)
    
    
# Joint position function
def move_to_joint_position(target_positions):
    print("Moving to joint positions")
    for target_position in target_positions:
        # Move the arm toward the target position
        for i, motor in enumerate(ur_motors):
            motor.setPosition(target_position[i])
            
        sleep(1)  # Blocking behavior
        
        # Step through simulation using MAX_STEPS
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)

            # Check if the robot is close to the target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target joint position.")
                # Print the orientation of the end effector
                print_end_effector_orientation(ur_motors)
                break  # Exit loop
        else:
            print("Max steps reached. Robot did not reach the target joint position.")
            return  # Exit the outer loop if max steps are reached without reaching the target

#function loop
move_to_joint_position(target_positions)

def move_end_effector(x, y, z):
    print("moving end effector")
    # Inverse kinematics calculation
    IKPY_MAX_ITERATIONS = 4
    #set initial positions
    initial_position = [0] + [m.getPositionSensor().getValue() for m in ur_motors] + [0, 1, 1, -1]
    #compute IK with position and orientation
    ikResults = ur5e.inverse_kinematics([x, y, z], target_orientation=target_orientation, orientation_mode="all", max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    #final positions and orientations of robot
    position = ur5e.forward_kinematics(ikResults)[:3, 3]
    orientation = ur5e.forward_kinematics(ikResults)[:3, :3]
    
    #compare resulting positions and orientations with targets
    print("Requested position: {} vs Reached position: {}".format([x, y, z], position))
    print("Requested orientation on the Y axis: {} vs Reached orientation on the Y axis: {}".format(target_orientation, orientation))
    
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
            


target_orientation = np.eye(3) 
#returns 2-dimensional array
#[1, 0, 0]
#[0, 1, 0]
#[0, 0, 1]

# Move end effector to the specified position (x,y,z)
move_end_effector(0.5, 0.3, 0.5)

sleep(0.1)

# Function to capture an image from the camera
def capture_image():
    # Add try-except block for error handling
    try:
        image = camera.getImageArray()
        return np.array(image, dtype=np.uint8) if image is not None else None
    except Exception as e:
        print(f"Error capturing camera image: {e}")
        return None
        
# Function to check for the colour yellow
def check_for_yellow(image):
    print("checking for yellow objects")
    # Define lower and upper bounds for yellow color in HSV
    lower_yellow = np.array([65, 100, 100])
    upper_yellow = np.array([55, 255, 255])

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
    print('image shape', image_np.shape)

    # Call the function to check for yellow color
    if check_for_yellow(image_np):
        print("Yellow color detected")
    else:
        print("Yellow color not detected")
else:
    print("Error: Unable to capture camera image")
            # Wait for the arm to reach the target position
    sleep(1)


# list of XYZ coordinates
xyz_coordinates = [
    (0.2, 0.6, 0.6),
    (0.3, 0.5, 0.4)
]

def look_for_yellow_object(coordinates):
    print("Looking for yellow objects")
    
    for coord in coordinates:
        # Move the end effector to the specified location
        move_end_effector(*coord)
        
        # Capture an image
        image = capture_image()
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        if image is not None:
            # Display the image in a resizable OpenCV window
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            
            # Resize the window to be bigger
            cv2.resizeWindow('Camera Feed', 300, 300)  # Set the width and height as desired
            
            cv2.imshow('Camera Feed', image)
            cv2.waitKey(1000)  # Add a delay for window display
            
            # Check for yellow color in the captured image
            if check_for_yellow(image):
                print("Yellow color detected")
            else:
                print("Yellow color not detected")
        else:
            print("Skipping processing for this area due to camera image capture error")

# Call the function to look for yellow objects
look_for_yellow_object(xyz_coordinates)

#could hardcode the gripper ot move, see if the joints work
#look to see if theres a motor that controls all the jonts in the gripper - research gripper
#research camera-recognition function in webots. see if there are any methods n recognising objects.
#fix get orientation function- fix the 