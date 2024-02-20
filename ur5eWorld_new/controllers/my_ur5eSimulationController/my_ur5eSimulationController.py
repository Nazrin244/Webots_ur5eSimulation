from controller import Robot
import numpy as np
import cv2
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
from time import sleep

TIME_STEP = 32
MAX_STEPS = 100
IKPY_MAX_ITERATIONS = 4

class UR5e(Robot):
    def __init__(self):
        super().__init__()
        
        position_sensors = [
            self.getDevice("shoulder_pan_joint_sensor"),
            self.getDevice("shoulder_lift_joint_sensor"),
            self.getDevice("elbow_joint_sensor"),
            self.getDevice("wrist_1_joint_sensor"),
            self.getDevice("wrist_2_joint_sensor"),
            self.getDevice("wrist_3_joint_sensor"),
        ]
        
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
        
        # Initialize camera
        camera = self.getDevice('camera')
        camera.enable(TIME_STEP)
        camera.recognitionEnable(TIME_STEP)
        
                                
        print('robot initialised.')
        
    def create_urdf(self, urdf_fn='C:/Users/Nazrin/Webots_ur5eSimulation/UR5e.urdf'):
        with open(urdf_fn, "w") as file:
            file.write(self.getUrdf())
            #load chain
        self.ur5e = Chain.from_urdf_file(urdf_fn,
                            active_links_mask=[0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1])
        print(f'saved urdf to {urdf_fn} with joint pos {self.joint_pos()}')
        
    def joint_pos(self):
        return np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        
    def move_to_joint_pos(self, target_joint_pos, timeout=2):
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
            
        #step through simulation using MAX_STEPS
        for step in range(timeout * 1000 // TIME_STEP):
            self.step()

    def move_end_effector(self, x, y, z):
        #returns 2-dimensional array
        target_orientation = (1, 0, 1)

        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0, 1, 1, -1]
        # Compute IK with position and orientation
        ikResults = self.ur5e.inverse_kinematics([x, y, z], target_orientation=target_orientation, orientation_mode="Y", max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        #final positions and orientations of robot
        position = self.ur5e.forward_kinematics(ikResults)[:3, 3]
        orientation = self.ur5e.forward_kinematics(ikResults)[:3, :3]
        
        if ikResults is not None:
            for i, motor in enumerate(self.motors):
                motor.setPosition(ikResults[i+1])

        for step in range(MAX_STEPS):
            self.step(TIME_STEP)

    def print_orientation(self):
        active_joint_pos = [motor.getPositionSensor().getValue() if (i+1) not in [1, 8] else 0 for i, motor in enumerate(self.motors)]
        all_joint_pos = active_joint_pos + [0] * (len(self.ur5e.links) - len(active_joint_pos))
        end_effector_pose = self.ur5e.forward_kinematics(all_joint_pos)
        orientation = end_effector_pose[:3, :3]
        
        # Extracting the y-axis coordinates
        y_axis_coordinates = orientation[:, 1]  # Second column corresponds to the y-axis
        
        print('Orientation of the y-axis:', y_axis_coordinates)

        
if __name__ == '__main__':
    robot = UR5e()
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e_1.urdf')
    robot.move_to_joint_pos([0, -1.57, 1.57, -1.57, -1.57, 0.0])    
    robot.print_orientation()
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e_2.urdf') 
    robot.move_end_effector(0.1, 0.5, 0.6)

