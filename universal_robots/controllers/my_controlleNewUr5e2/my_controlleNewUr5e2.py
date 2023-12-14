from controller import Robot

robot = Robot()
TIME_STEP = 32

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
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain

ur5e = Chain.from_urdf_file("C:\\YEAR3\\FYP\\UR5e.urdf",
                            active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])
print(ur5e.links)

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

for sensor in position_sensors:
    sensor.enable(TIME_STEP)

# Move the end effector to a position
def move_end_effector(x, y, z):
    # Inverse kinematics calculation
    IKPY_MAX_ITERATIONS = 4
    initial_position = [0] + [m.getPositionSensor().getValue() for m in ur_motors] + [0, 1, 1, -1]
    ikResults = ur5e.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    # setting positions based on Inverse Kinematics
    for i, motor in enumerate(ur_motors):
        motor.setPosition(ikResults[i + 1]) #skip first element

# Set initial position and velocities
for motor in ur_motors:
    motor.setPosition(0)  # Set position to 0 for all motors in the list
    motor.setVelocity(1.0)  # Set velocity for movement

while robot.step(TIME_STEP) != -1:
    # Move arm toward target_position (x=0.5, y=-0.3, z=0.5)
    move_end_effector(0.9, 0.2, 0.5)

    joint_positions = [sensor.getValue() for sensor in position_sensors]
    print("Joint positions:", joint_positions)
