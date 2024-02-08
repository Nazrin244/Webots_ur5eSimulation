from controller import Robot, Camera
import time
from time import sleep
import numpy as np
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain

robot = Robot()
TIME_STEP = 32
MAX_STEPS = 100

# Create URDF
with open("C:\\YEAR3\\FYP\\UR5e.urdf", "w") as file:
    file.write(robot.getUrdf())

ur5e = Chain.from_urdf_file("C:\\YEAR3\\FYP\\UR5e.urdf",
                            active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])
print(ur5e.links)

ur_motors = [
    robot.getDevice("shoulder_pan_joint"),
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
    robot.getDevice("wrist_3_joint")]
    
position_sensors = [
    robot.getDevice("shoulder_pan_joint_sensor"),
    robot.getDevice("shoulder_lift_joint_sensor"),
    robot.getDevice("elbow_joint_sensor"),
    robot.getDevice("wrist_1_joint_sensor"),
    robot.getDevice("wrist_2_joint_sensor"),
    robot.getDevice("wrist_3_joint_sensor"),
]

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
#establish target positions
target_positions = [
    [0, -1.57, 1.57, -1.57, -1.57, 0.0],
    [0.0, -1.0, 1.0, -1.0, -1.0, 0.0],
    [1.0, -0.5, 1.0, -0.5, -1.0, 0.5],
    [0.5, -1.0, 1.2, -0.8, -0.8, 0.0],
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

        # Step through simulation using MAX_STEPS
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)

            # Check if the robot is close to the target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target end effector position.")
                break  # Exit loop if the target position is reached
        else:
            print("Max steps reached. Robot did not reach the target end effector position.")

# Function loop
move_end_effector(0.5, 0.3, 0.5)

joint_positions = [sensor.getValue() for sensor in position_sensors]
print("Joint positions:", joint_positions)
   
#Camera:
#open cv package - done
#camera sensor
# can add as a node
#camera getimage commnd
#open cv to use image procesing algorithms to detect cube 

#TODO:
#restructure to blocking behaviours - nearly done
#introduce the object, hardcode to grab it, place in box
#use functions that i have to move between positions 
#intermediate positions, way points can define like a list
#orientation of the robot
