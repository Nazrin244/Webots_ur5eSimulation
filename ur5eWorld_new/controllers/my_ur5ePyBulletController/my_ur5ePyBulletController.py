import pybullet as p
import pybullet_data
import numpy as np

PATH_TO_URDF = '../../../universal_robots/UR5e_2.urdf'

class Robot():
    def __init__(self, urdf_fn):
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf')
        
        self.robot_id = p.loadURDF(urdf_fn, useFixedBase=True)
        self.joint_ids = list(range(6))
        
    def joint_pos(self):
        joint_states = p.getJointStates(self.robot_id, self.joint_ids)
        pos = [joint_states[joint][0] for joint in self.joint_ids]
        return np.array(pos)
        
    def reset_joints(self, joint_pos):
        for joint_id, q in zip(self.joint_ids, joint_pos):
            p.resetJointState(self.robot_id, joint_id, q)

def main():
    robot = Robot(PATH_TO_URDF)
    robot.reset_joints([0] * 6)
    print('pos:', robot.joint_pos())
    input('press any key')
    p.disconnect()

if __name__ == '__main__':
	main()