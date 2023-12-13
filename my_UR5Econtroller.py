from controller import Robot
from ikpy.chain import Chain

robot = Robot()
TIME_STEP = 32

#create URDF 
with open("C:\\YEAR3\\FYP\\UR5e.urdf", "w") as file:  
    file.write(robot.getUrdf()) 
#create chain
ur5e = Chain.from_urdf_file("C:\\YEAR3\\FYP\\UR5e.urdf", active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])

#get motors
ur_motors = [
    robot.getDevice("shoulder_pan_joint"),
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint"),
    robot.getDevice("wrist_3_joint"),
]

#get position sensors
position_sensors = [
    robot.getDevice("shoulder_pan_joint_sensor"),
    robot.getDevice("shoulder_lift_joint_sensor"),
    robot.getDevice("elbow_joint_sensor"),
    robot.getDevice("wrist_1_joint_sensor"),
    robot.getDevice("wrist_2_joint_sensor"),
    robot.getDevice("wrist_3_joint_sensor"),
]

# set initial position and velocities
for motor in ur_motors:
    motor.setPosition(0)  # set position to 0 for all motors in list
    motor.setVelocity(1.0)  # set velocity for movement
    
for sensor in position_sensors:
    sensor.enable(TIME_STEP)

#inverse kinematics function
def calculate_inverse_kinematics(x, y, z):
    IKPY_MAX_ITERATIONS = 4
    # Create initial position using joint positions
    initial_position = [0] + [m.getPositionSensor().getValue() for m in ur_motors] + [0, 1, 1, -1]
    # Calculate inverse kinematics
    ikResults = ur5e.inverse_kinematics([1, 1, 1], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    return ikResults  #skips the first element


#move to joint position function
def move_to_joint_positions(x, y, z):
    # Calculate inverse kinematics
    ikResults = calculate_inverse_kinematics(x, y, z)
    # Set target position to motors using ik_results
    for i, motor in enumerate(ur_motors):
        motor.setPosition(ikResults[i])
        

#move to end effector function
def move_end_effector(x, y, z):
    # Calls inverse kinematics function
    ikResults = calculate_inverse_kinematics(x, y, z)
    #set target position using ik_results 
    for i, motor in enumerate(ur_motors):
        motor.setPosition(ikResults[i])


# Main Control
while robot.step(TIME_STEP) != -1:
    # Move arm toward target_position (x=0.5, y=-0.3, z=0.5)
    move_to_joint_positions(0.5, 0.3, 0.5)
    

    # Print joint positions
    joint_positions = [sensor.getValue() for sensor in position_sensors]
    print("Arm Joint positions:", joint_positions)
    
