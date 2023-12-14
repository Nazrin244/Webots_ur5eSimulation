from controller import Robot, Motor, DistanceSensor, PositionSensor

TIME_STEP = 32

# Enum to represent different states of the robot
class State:
    WAITING = 0
    GRASPING = 1
    ROTATING = 2
    RELEASING = 3
    ROTATING_BACK = 4

def main():
    # Initialize the robot
    robot = Robot()
    counter = 0
    state = State.WAITING  # Initial state is WAITING
    target_positions = [-1.88, -2.14, -2.38, -1.51]
    speed = 1.0

    # Get motor devices for hand and arm
    hand_motors = [robot.getDevice("finger_1_joint_1"),
                   robot.getDevice("finger_2_joint_1"),
                   robot.getDevice("finger_middle_joint_1")]

    ur_motors = [robot.getDevice("shoulder_lift_joint"),
                 robot.getDevice("elbow_joint"),
                 robot.getDevice("wrist_1_joint"),
                 robot.getDevice("wrist_2_joint")]

    # Set arm motors to infinite position and set velocity
    for motor in ur_motors:
        motor.setPosition(float('inf'))
        motor.setVelocity(speed)

    # Enable sensors for distance and joint position
    distance_sensor = robot.getDevice("distance sensor")
    distance_sensor.enable(TIME_STEP)

    position_sensor = robot.getDevice("wrist_1_joint_sensor")
    position_sensor.enable(TIME_STEP)

    # Main control loop
    while robot.step(TIME_STEP) != -1:
        if counter <= 0:
            if state == State.WAITING:
                # Check if the distance sensor value is below 500
                if distance_sensor.getValue() < 500:
                    # Transition to GRASPING state
                    state = State.GRASPING
                    counter = 8
                    print("Grasping can")
                    # Set hand motors to grasp the can
                    for motor in hand_motors:
                        motor.setPosition(0.85)

            elif state == State.GRASPING:
                # Set arm motors to target positions
                for i, motor in enumerate(ur_motors):
                    motor.setPosition(target_positions[i])
                print("Rotating arm")
                state = State.ROTATING

            elif state == State.ROTATING:
                # Check if the wrist joint has rotated sufficiently
                if position_sensor.getValue() < -2.3:
                    counter = 8
                    print("Releasing can")
                    state = State.RELEASING
                    # Set hand motors to release the can
                    for motor in hand_motors:
                        motor.setPosition(motor.getMinPosition())

            elif state == State.RELEASING:
                # Set arm motors to initial positions
                for motor in ur_motors:
                    motor.setPosition(0.0)
                print("Rotating arm back")
                state = State.ROTATING_BACK

            elif state == State.ROTATING_BACK:
                # Check if the wrist joint has rotated back sufficiently
                if position_sensor.getValue() > -0.1:
                    state = State.WAITING
                    print("Waiting can")

        counter -= 1

if __name__ == "__main__":
    main()
