from controller import Robot
import numpy as np

TIME_STEP = 32

class UR5e(Robot):
    def __init__(self):
        super().__init__()
        
        self.motors = [
            self.getDevice("shoulder_pan_joint"),
            self.getDevice("shoulder_lift_joint"),
            self.getDevice("elbow_joint"),
            self.getDevice("wrist_1_joint"),
            self.getDevice("wrist_2_joint"),
            self.getDevice("wrist_3_joint"),
        ]
        for m in self.motors:
            m.getPositionSensor().enable(TIME_STEP)
            
        print('robot initialised.')
        
    def create_urdf(self, urdf_fn='../../resources/ur5e.urdf'):
        with open(urdf_fn, "w") as file:
            file.write(self.getUrdf())
        print(f'saved urdf to {urdf_fn} with joint pos {self.joint_pos()}')

    def joint_pos(self):
        return np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        
    def move_to_joint_pos(self, target_joint_pos, timeout=2):
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
            
        #step through simulation using MAX_STEPS
        for step in range(timeout * 1000 // TIME_STEP):
            self.step()

if __name__ == '__main__':
    robot = UR5e()
    robot.create_urdf('../../resources/ur5e_1.urdf')
    robot.move_to_joint_pos([0, -1.57, 1.57, -1.57, -1.57, 0.0])
    robot.create_urdf('../../resources/ur5e_2.urdf')