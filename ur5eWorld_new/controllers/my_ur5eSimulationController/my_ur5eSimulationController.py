from controller import Robot
import numpy as np
import cv2
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
from time import sleep
import math

# Constants
TIME_STEP = 32
MAX_STEPS = 100
IKPY_MAX_ITERATIONS = 4
speed = 1.0

class UR5e(Robot):
    def __init__(self):
        super().__init__()
        
        #Position sensors
        position_sensors = [
            self.getDevice("shoulder_pan_joint_sensor"),
            self.getDevice("shoulder_lift_joint_sensor"),
            self.getDevice("elbow_joint_sensor"),
            self.getDevice("wrist_1_joint_sensor"),
            self.getDevice("wrist_2_joint_sensor"),
            self.getDevice("wrist_3_joint_sensor")
        ]
        
        # Motors
        self.motors = [
            self.getDevice("shoulder_pan_joint"),
            self.getDevice("shoulder_lift_joint"),
            self.getDevice("elbow_joint"),
            self.getDevice("wrist_1_joint"),
            self.getDevice("wrist_2_joint"),
            self.getDevice("wrist_3_joint")
        ]
        
        # Grippers
        self.grippers = [
            self.getDevice("finger_middle_joint_1"),
            self.getDevice("finger_middle_joint_2"),
            self.getDevice("finger_middle_joint_3"),
            self.getDevice("finger_2_joint_1"),
            self.getDevice("finger_2_joint_2"),
            self.getDevice("finger_2_joint_3"),
            self.getDevice("finger_1_joint_1"),
            self.getDevice("finger_1_joint_2"),
            self.getDevice("finger_1_joint_3"),
        ]
        
        # Set positions and velocity
        for motor in self.motors:
            motor.getPositionSensor().enable(TIME_STEP)
            motor.setVelocity(speed)
        
        # Initialize camera
        self.camera = self.getDevice('CAM')
        self.camera.enable(TIME_STEP)

        print('robot initialised.')
        
        # Create and load chain
    def create_urdf(self, urdf_fn='C:/Users/Nazrin/Webots_ur5eSimulation/UR5e.urdf'):
        self.ur5e = Chain.from_urdf_file(urdf_fn, active_links_mask=[0, 1, 1, 1, 1, 1, 0])
        print(f'Initial joint positions: {self.joint_pos()}')

    # Get current joint positions
    def joint_pos(self):
        return np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        
    # Move to joint position function
    def move_to_joint_pos(self, target_joint_pos, timeout=2):
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
        for step in range(timeout * 1000 // TIME_STEP):
            self.step()

    # Move end effector function
    def move_end_effector(self, x, y, z):
        # Set orientation and initial position
        target_orientation = [0, 0, -1]
        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors]
        # Compute IK with position and orientation
        ikResults = self.ur5e.inverse_kinematics([x, y, z], target_orientation=target_orientation, orientation_mode="Y", max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
        # Final positions and orientation 
        position = self.ur5e.forward_kinematics(ikResults)[:3, 3]
        orientation = self.ur5e.forward_kinematics(ikResults)[:3, 1]   

        if ikResults is not None:
            for i, motor in enumerate(self.motors):
                motor.setPosition(ikResults[i+1])
        for step in range(MAX_STEPS):
            self.step(TIME_STEP) 
           
    # Get imsge function
    def get_image(self):
        print("capturing image...")
        # Get image array
        img = self.camera.getImageArray()  
        img = np.array(img)    
        img_height, img_width, channels = img.shape          
        img = np.array(img, dtype=np.uint8)
        img = np.reshape(img, (img_height, img_width, channels))        
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return img
                
    # Object detection funtion
    def object_detection(self):
        print("capturing image...")
        # Capture image
        img = self.get_image()
        # Default displacement values
        displacement_x = 0.08
        displacement_y = 0.08
        pixel_to_meter = -0.07
        
        if img is not None:
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Establish upper and lower bounds
            lower_bound = np.array([40, 40, 40])
            upper_bound = np.array([80, 255, 255])
            # Create mask and contours
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
            if contours:
                print('Green object found!')
                return
            else:
                print("no green object found")  
                           
    # Rotational search function
    def rotational_search(self):
        print("performing rotational search...")
        # Move to the left position
        target_joint_pos_left = [0, -1.57, 1.57, -1.57, -0.6, 0.0]
        self.move_to_joint_pos(target_joint_pos_left)
        self.object_detection()
        if self.object_detection():
            print("Green object found!")
            # Return to the original position
            original_position = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
            self.move_to_joint_pos(original_position)
            return

        # Move to the right position
        target_joint_pos_right = [0, -1.57, 1.57, -1.57, -2.5, 0.0]
        self.move_to_joint_pos(target_joint_pos_right)
        self.object_detection() 
        if self.object_detection():
            print("Green object found!")       
            # Return to the original position
            original_position = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
            self.move_to_joint_pos(original_position)
            return
            
    # Grid search function
    def grid_search(self):
        print("implementing grid search...")
        # Initial parameters
        x = 0.6
        y = 0.5
        z = 0.3
        step_size = 0.3      
        step_up = 0.2
        step = 3
        
        # Move to initial position
        self.move_end_effector(x,y,z)  
        
        # Decrease y value
        for i in range(2):
            for k in range(step):
                y -= step_size
                self.move_end_effector(x, y, z)
                if self.object_detection():
                    self.grasp()
                    return   
            # Decrease x value 
            for j in range(1):
                x += step_up
                self.move_end_effector(x,y,z) 
                if self.object_detection():
                    self.grasp()
                    return   
            # Increase y value    
            for m in range(step): 
                y += step_size
                self.move_end_effector(x,y,z)
                if self.object_detection():
                    self.grasp()
                    return     
            # Increase x value 
            for n in range(1):
                x += step_up
                self.move_end_effector(x,y,z)   
                if self.object_detection():
                    self.grasp()
                    return

    # Spiral search function
    def spiral_search(self):
        print("implementing spiral search...")
        # Initial parameters
        x_center = 0.6 
        y_center = 0
        z = 0.3 
        start_radius = 0.09
        num_turns = 3
        angle_increment = 1
        radius_increment = 0.09

        radius = start_radius
        for i in range(num_turns):
            # Create circle with increasing radius
            for angle in np.arange(0, 2 * np.pi, angle_increment):
                # Calculate x and y coordinates
                x = x_center + radius * np.cos(angle)
                y = y_center + radius * np.sin(angle)
                # Move end effector to new position
                self.move_end_effector(x, y, z)
                if self.object_detection():
                    print("Green Object found!")
                    return
            # Increase radius for the next loop
            radius += radius_increment
    
                             
if __name__ == '__main__':        
    robot = UR5e()
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e.urdf')
    robot.move_to_joint_pos([0, -1.57, 1.57, -1.57, -1.57, 0.0])
    robot.rotational_search()     
    robot.grid_search()    
    robot.spiral_search()   
