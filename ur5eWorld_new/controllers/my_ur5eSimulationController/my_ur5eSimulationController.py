from controller import Robot
import numpy as np
import cv2
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
from time import sleep

TIME_STEP = 32
MAX_STEPS = 100
IKPY_MAX_ITERATIONS = 4
speed = 1.0

class UR5e(Robot):
    def __init__(self):
        super().__init__()
        
        position_sensors = [
            self.getDevice("shoulder_pan_joint_sensor"),
            self.getDevice("shoulder_lift_joint_sensor"),
            self.getDevice("elbow_joint_sensor"),
            self.getDevice("wrist_1_joint_sensor"),
            self.getDevice("wrist_2_joint_sensor"),
            self.getDevice("wrist_3_joint_sensor")
        ]
        
        self.motors = [
            self.getDevice("shoulder_pan_joint"),
            self.getDevice("shoulder_lift_joint"),
            self.getDevice("elbow_joint"),
            self.getDevice("wrist_1_joint"),
            self.getDevice("wrist_2_joint"),
            self.getDevice("wrist_3_joint")
        ]
        
        self.grippers = [
            self.getDevice("finger_middle_joint_1"),
            self.getDevice("finger_middle_joint_2"),
            #self.getDevice("finger_middle_joint_3"),
            self.getDevice("finger_2_joint_1"),
            self.getDevice("finger_2_joint_2"),
            #self.getDevice("finger_2_joint_3"),
            self.getDevice("finger_1_joint_1"),
            self.getDevice("finger_1_joint_2"),
            #self.getDevice("finger_1_joint_3"),

        ]
        
        #Set positions and velocity
        for motor in self.motors:
            motor.getPositionSensor().enable(TIME_STEP)
            motor.setVelocity(speed)
        

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
        target_orientation = [0, 1, 0]

        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0, 1, 1, -1]
        # Compute IK with position and orientation
        ikResults = self.ur5e.inverse_kinematics([x, y, z], target_orientation=target_orientation, orientation_mode="Y", max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
    
        #final positions and orientations of robot
        position = self.ur5e.forward_kinematics(ikResults)[:3, 3]
        orientation = self.ur5e.forward_kinematics(ikResults)[:3, 1]

        print("Requested orientation on the X axis: {} vs Reached orientation on the X axis: {}".format(target_orientation, orientation))
        # We see that the chain reached its position!
        
        if ikResults is not None:
            for i, motor in enumerate(self.motors):
                motor.setPosition(ikResults[i+1])

        for step in range(MAX_STEPS):
            self.step(TIME_STEP)
            
        
           
    def grasp(self):
        print("grasping...")
        for m in self.grippers:
            m.setPosition(0.96) 
            m.setVelocity(speed)
        for step in range(MAX_STEPS):
            self.step(TIME_STEP)
            
            
    def release(self):
        print("releasing...")
        for m in self.grippers:
            m.setPosition(0.1)
            m.setVelocity(speed)
        for step in range(MAX_STEPS):
            self.step(TIME_STEP)

                        
if __name__ == '__main__':        
    robot = UR5e()
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e_1.urdf')
    robot.move_to_joint_pos([0, -1.57, 1.57, -1.57, -1.57, 0.0])
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e_2.urdf') 
    robot.move_end_effector(0.4, 0.6, 0.5)
    robot.grasp()
    robot.release()


#could hardcode the gripper to move, see if the joints work
#look to see if theres a motor that controls all the jonts in the gripper - research gripper
#research camera-recognition function in webots. see if there are any methods in recognising objects.
#fix get orientation function