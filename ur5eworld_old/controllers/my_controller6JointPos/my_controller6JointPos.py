from controller import Robot
import time

robot = Robot()
TIME_STEP = 32
MAX_STEPS = 100

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
   
def move_to_joint_position(target_positions):
    for target_position in target_positions:
        # move arm toward the target position
        for i, motor in enumerate(ur_motors):
            motor.setPosition(target_position[i])
            
        for step in range(MAX_STEPS):
            robot.step(TIME_STEP)

            #check if the robot is close to target position
            if all(abs(motor.getTargetPosition() - motor.getPositionSensor().getValue()) < 0.1 for motor in ur_motors):
                print("Robot reached the target joint position.")
                break #exit loop
        
        else:
            print("Max steps reached. Robot did not reach the target joint position.")
            return  # Exit the outer loop if max steps are reached without reaching the target
                    
        #joint_positions = [sensor.getValue() for sensor in position_sensors]
        #print("Joint positions:", joint_positions)
    

target_positions = [
    [0, -1.57, 1.57, -1.57, -1.57, 0.0],
    [0.0, -1.0, 1.0, -1.0, -1.0, 0.0],
    [1.0, -0.5, 1.0, -0.5, -1.0, 0.5],
    [0.5, -1.0, 1.2, -0.8, -0.8, 0.0]
]

move_to_joint_position(target_positions)