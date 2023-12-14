from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from controller import Robot
from controller import CameraRecognitionObject
from controller import Camera
import math

#create robot and camera
robot = Robot()
camera = robot.getDevice('camera')
timeStep = 1
IKPY_MAX_ITERATIONS = 4

#create URDF 
with open("C:\\Program Files\\Webots\\resources\\ur5e.urdf", "w") as file:  
    file.write(robot.getUrdf()) 
#create chain
base_elements=["base_link","shoulder_pan_joint","shoulder_link", "shoulder_lift_joint", "upper_arm_link", "elbow_joint"]

ur5e = Chain.from_urdf_file("C:\\Program Files\\Webots\\resources\\ur5e.urdf", base_elements=["base_link", "shoulder_pan_joint","shoulder_link", "shoulder_lift_joint", "upper_arm_link", "elbow_joint"])
print(ur5e.links)

#add last link to chain
ur5e = Chain.from_urdf_file("C:\\Program Files\\Webots\\resources\\ur5e.urdf", last_link_vector=[0.039, 0, 0], base_elements=["base_link", "shoulder_pan_joint","shoulder_link", "shoulder_lift_joint", "upper_arm_link", "elbow_joint"])
print(ur5e.links)

#deactivate links that cannot be controlled (COMPLETE)
part_names = ("wrist_3_joint")
for link_id in range(len(ur5e.links)):
#link object
    link = ur5e.links[link_id]
#manually disable 'wrist_3_joint'
if link.name not in part_names or link.name == "wrist_3_joint":
    print("Disabling {}".format(link.name))
    ur5e.active_links_mask[link_id] = False

# Initialize the arm motors
motors = []
for link in ur5e.links:
    if 'motor' in link.name:
        motor = robot.getDevice(link.name)
        motor.setVelocity(3.0)#set position - joint position 
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

#get arm and target nodes
target = robot.getDevice('TARGET')
arm = robot.get

#Loop 1: move arm to the target
print('move arm to red cube')
while robot.step(timeStep) != -1:
    #get position of the target and arm
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

#compute position of the target so it is relative to the arm
    x = (targetPosition[0] - armPosition[0])
    y = targetPosition[1] - armPosition[1]
    z = targetPosition[2] - armPosition[2]
   
#call IKPY to cpmpute the inverse kinematics
initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

   # if link.name in part_names and link.name != "wrist_3_joint":
    #    motor = robot.getDevice(link.name)

        # Make sure to account for any motors that
        # require a different maximum velocity!
     #   if link.name == "wrist_3_joint":
       #     motor.setVelocity(0.07)
      #  else:
        #    motor.setVelocity(1)
            
        #position_sensor = motor.getPositionSensor()
        #position_sensor.enable(timestep)
        #motors.append(motor)
        
       
 

