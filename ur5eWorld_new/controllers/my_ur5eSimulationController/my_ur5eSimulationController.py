from controller import Robot
import numpy as np
import cv2
from ikpy.link import OriginLink, URDFLink
from ikpy.chain import Chain
from time import sleep
import math

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
            self.getDevice("finger_middle_joint_3"),
            self.getDevice("finger_2_joint_1"),
            self.getDevice("finger_2_joint_2"),
            self.getDevice("finger_2_joint_3"),
            self.getDevice("finger_1_joint_1"),
            self.getDevice("finger_1_joint_2"),
            self.getDevice("finger_1_joint_3"),
        ]
        
        #Set positions and velocity
        for motor in self.motors:
            motor.getPositionSensor().enable(TIME_STEP)
            motor.setVelocity(speed)
        
        # Initialize camera
        self.camera = self.getDevice('CAM')
        self.camera.enable(TIME_STEP)

        print('robot initialised.')
        
    def create_urdf(self, urdf_fn='C:/Users/Nazrin/Webots_ur5eSimulation/UR5e.urdf'):
        # Load chain
        self.ur5e = Chain.from_urdf_file(urdf_fn, active_links_mask=[0, 1, 1, 1, 1, 1, 0])
        
        print(f'Number of links: {len(self.ur5e.links)}')
        print('List of links:')
        for link in self.ur5e.links:
            print(link.name)
            
        print(f'Initial joint positions: {self.joint_pos()}')
    
        # Print out end effector link
        end_effector_link = self.ur5e.links[-1]
        print(f"End Effector Link: {end_effector_link.name}")
    
        
    def joint_pos(self):
        return np.asarray([m.getPositionSensor().getValue() for m in self.motors])
        
    def move_to_joint_pos(self, target_joint_pos, timeout=2):
        for pos, motor in zip(target_joint_pos, self.motors):
            motor.setPosition(pos)
        #step through simulation using MAX_STEPS
        for step in range(timeout * 1000 // TIME_STEP):
            self.step()

    def move_end_effector(self, x, y, z):
        #set orientation and initial position
        target_orientation = [0, 0, -1]
        initial_position = [0] + [m.getPositionSensor().getValue() for m in self.motors]
        #Compute IK with position and orientation
        ikResults = self.ur5e.inverse_kinematics([x, y, z], target_orientation=target_orientation, orientation_mode="Y", max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
        #final positions and orientation 
        position = self.ur5e.forward_kinematics(ikResults)[:3, 3]
        orientation = self.ur5e.forward_kinematics(ikResults)[:3, 1]   

        if ikResults is not None:
            for i, motor in enumerate(self.motors):
                motor.setPosition(ikResults[i+1])
        for step in range(MAX_STEPS):
            self.step(TIME_STEP) 
            
    def grasp(self):
        print("grasping...")


    def release(self):
        print("releasing...")
        for m in self.grippers:
            m.setPosition(0.1)
            m.setVelocity(speed)
        for step in range(MAX_STEPS):
            self.step(TIME_STEP)
            
    def get_image(self):
        print("capturing image...")
        img = self.camera.getImageArray()  
        img = np.array(img)    
        #print('image shape', img.shape)
        img_height, img_width, channels = img.shape          
        img = np.array(img, dtype=np.uint8)
        img = np.reshape(img, (img_height, img_width, channels))        
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return img
                
    def object_detection(self):
        print("capturing image...")
        img = self.get_image()
        
        # Default displacement values
        displacement_x = 0.08
        displacement_y = 0.08
        pixel_to_meter = -0.07 #1 pixel = 0.01 meter
        
        if img is not None:
            hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_bound = np.array([40, 40, 40])
            upper_bound = np.array([80, 255, 255])
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
            if contours:
                print('Green object found!')
                contour = max(contours, key=cv2.contourArea)
                x,y,w,h = cv2.boundingRect(contour)
                # Draw bounding box on the image
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)  # Green color, thickness=1
                
                #Calculates center of bounding box
                center_x = x + w //2
                center_y = y + h //2
                
                print("center of bounding box: ({}, {})".format(center_x, center_y))
                
                # Display the image with bounding box
                cv2.circle(img, (center_x, center_y), 2, (0, 0, 255), -1)
                cv2.imshow("Image with bounding box", img)
                cv2.waitKey(1)  # Wait for a key press to close the window
                cv2.destroyAllWindows()  # Close all OpenCV windows     
                           
                
    def rotation_search(self):
        print("performing rotational search...")
        # Move to the left position
        target_joint_pos_left = [0, -1.57, 1.57, -1.57, -0.6, 0.0]
        self.move_to_joint_pos(target_joint_pos_left)
        self.object_detection()
        if self.object_detection():
        #TODO: move to position of object
            self.grasp()
            self.release()
            # Return to the original position
            original_position = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
            self.move_to_joint_pos(original_position)
            return

        # Move to the right position
        target_joint_pos_right = [0, -1.57, 1.57, -1.57, -2.5, 0.0]
        self.move_to_joint_pos(target_joint_pos_right)
        self.object_detection() 
        if self.object_detection():
        #TODO: move to position of object
            self.grasp()
            self.release()        
            # Return to the original position
            original_position = [0, -1.57, 1.57, -1.57, -1.57, 0.0]
            self.move_to_joint_pos(original_position)
            return

    def grid_search(self):
        print("implementing grid search...")
        x = 0.6
        y = 0.5
        z = 0.3
        step_size = 0.3      
        step_up = 0.2
        step = 3
        
        self.move_end_effector(x,y,z)  
        
        for i in range(2):
            for k in range(step):
                y -= step_size
                self.move_end_effector(x, y, z)
                if self.object_detection():
                    self.grasp()
                    return    
            for j in range(1):
                x += step_up
                self.move_end_effector(x,y,z) 
                if self.object_detection():
                    self.grasp()
                    return       
            for m in range(step): 
                y += step_size
                self.move_end_effector(x,y,z)
                if self.object_detection():
                    self.grasp()
                    return      
            for n in range(1):
                x += step_up
                self.move_end_effector(x,y,z)   
                if self.object_detection():
                    self.grasp()
                    return

    def spiral_search(self):
        print("implementing spiral search...")
    
        # Define initial parameters
        x_center = 0.6  # Central x-coordinate
        y_center = 0 # Central y-coordinate
        z = 0.3  # Fixed z-coordinate
        start_radius = 0.09  # Initial radius of the spiral
        num_turns = 3 # Number of turns for the spiral
        angle_increment = 1  # Angle increment for each step
        radius_increment = 0.09  # Radius increment for each turn

        # Perform spiral search
        radius = start_radius
        for i in range(num_turns):
            # Move along a circle with increasing radius
            for angle in np.arange(0, 2 * np.pi, angle_increment):
                # Calculate x and y coordinates based on angle and radius
                x = x_center + radius * np.cos(angle)
                y = y_center + radius * np.sin(angle)
    
                # Move end effector to position
                self.move_end_effector(x, y, z)
                #if self.object_detection():
                 #   print("Green Object found!")
                  #  return
            # Increase the radius for the next turn
            radius += radius_increment
    
                             
if __name__ == '__main__':        
    robot = UR5e()
    robot.create_urdf('C:/Users/Nazrin/Webots_ur5eSimulation/UR5e.urdf')
    robot.move_to_joint_pos([0, -1.57, 1.57, -1.57, -1.57, 0.0])
    #robot.grid_search()    
    #robot.rotation_search() 
    robot.spiral_search()   
    #robot.object_detection()     
    #robot.move_end_effector(0.6, 0.2, 0.0) 
    #robot.grasp()
    #robot.move_end_effector(0.1, 0.4, 0.7)
    #robot.release()
