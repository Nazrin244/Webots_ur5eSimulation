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

target_orientation = np.eye(3) #returns 2-dimensional array
#[1, 0, 0]
#[0, 1, 0]
#[0, 0, 1]

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
    orientation = ur5e.forward_kinematics(ikResults)[:3, 0]
    
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

# Move end effector to the specified position
move_end_effector(0.2, 0.4, 0.4)