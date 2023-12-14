from controller import Robot
import time

robot = Robot()
TIME_STEP = 32

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

for sensor in position_sensors:
    sensor.enable(TIME_STEP)


for motor in ur_motors:
    motor.setPosition(0.0) #set position to 0 for all motors in list
    motor.setVelocity(1.0) #set velocity for movement
   
target_positions = [
    [0, -1.57, 1.57, -1.57, -1.57, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [-1.57, -1.57, 1.57, -1.57, 0, 0]
]

while True:
    # Iterate through the joint configurations
    for target_position in target_positions:
        # Move the arm toward the target position
        for i, motor in enumerate(ur_motors):
            motor.setPosition(target_position[i])

        # Wait for the arm to reach the target position
        while robot.step(TIME_STEP) != -1:
            time.sleep(1)  # Add a delay to observe the movement
            joint_positions = [sensor.getValue() for sensor in position_sensors]
    print("Joint positions:", joint_positions)
            
#if joint positio = target
