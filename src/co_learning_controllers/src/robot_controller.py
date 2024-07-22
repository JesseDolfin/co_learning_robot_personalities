# robotic_arm_control.py

from math import floor
from torch import cartesian_prod
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from scipy.spatial.transform import Rotation as R
from robot.robot import Robot
from co_learning_messages.msg import hand_pose
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

# Constants
ROT = np.pi/4
HOME_POSITION = [np.pi/2, ROT, 0, -ROT, 0, ROT, 0]
INTERMEDIATE_POSITION = [np.pi/2, 0, 0, 0, 0, 0, 0]
GOAL_MODE = 'joint_ds'
GOAL_TIME = 5.0
GOAL_PRECISION = 1e-1
GOAL_RATE = 20
GOAL_STIFFNESS = [150.0, 150.0, 75.0, 75.0, 40.0, 15.0, 10.0]
GOAL_DAMPING = 2 * np.sqrt(GOAL_STIFFNESS)
NULLSPACE_GAIN = [0, 0, 0, 0, 0, 0, 0]

class RoboticArmController:
    def __init__(self):
        self.q = None
        self.goal_time = GOAL_TIME
        self.ee_pose = [0, 0, 0, 0, 0, 0]
        ns = rospy.get_param('/namespaces')

        self.robot = Robot(model=ns.replace('/',''))
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)
        rospy.Subscriber(ns+'/joint_states', JointState, self.joint_callback, queue_size=10)
        rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)

        self.publish_human_input = rospy.Publisher('human_input', Bool, queue_size=1)

        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        #rospy.Rate(0.5).sleep() # Give enough time for the FRIoverlay app to start up

        self.goal_time = GOAL_TIME
        self.hand_pose = [0,0,1.3]

        self.type = 'none'

    def joint_callback(self, msg):
        self.q = msg.position
        self.q_dot = msg.velocity
        if self.robot is not None:
            ee_T = np.array(self.robot.fkine(self.q, end='iiwa_link_7', start='iiwa_link_0'))
            ee_T_dot = np.array(self.robot.fkine(self.q_dot, end='iiwa_link_7', start='iiwa_link_0'))

            ee_vel = [ee_T_dot[0, 3], ee_T_dot[1, 3], ee_T_dot[2, 3]]

            
            if np.linalg.norm(ee_vel) > 10:
                self.publish_human_input.publish(True)
            else:
                self.publish_human_input.publish(False)


            translation = [ee_T[0, 3], ee_T[1, 3], ee_T[2, 3]]
            rot_mat = ee_T[0:3, 0:3]
            r = R.from_matrix(rot_mat)
            euler_angles = r.as_euler('xyz', degrees=False).tolist()
            self.ee_pose = translation + euler_angles[::-1] # reverses the list

    def shake_arm(self,amplitude,frequency):
        num_points = 20 * frequency
        x = np.linspace(0, 2*np.pi, num=int(np.ceil(num_points)))
        y = amplitude * np.sin(frequency * x) * 360
        y = np.deg2rad(y)

        total_time = GOAL_TIME #s
        time_per_segment = total_time / num_points

        for p in y:
            target = [0,p,0,p,0,p,0]
            goal = self.create_goal(target)
            goal.time = time_per_segment
            goal.mode = 'joint'
            self.send_position_command(goal)
            rospy.Rate(25).sleep()
            

    def hand_pose_callback(self, msg):
        self.hand_pose = [msg.x, msg.y, msg.z]

    def create_goal(self, position, nullspace=None, goal_time = None):

        goal = ControllerGoal()
        if goal_time == None:
            if self.type == 'fast':
                goal.time = GOAL_TIME - 2.0
            elif self.type == 'slow':
                goal.time = GOAL_TIME + 2.0
            else:
                goal.time = GOAL_TIME
        else:
            goal.time = goal_time

        # Auto select correct goal-mode based on input arguments
        if len(position) == 7:
            goal.mode = 'joint_ds'
            stiffness = [100.0, 100.0, 50.0, 50.0, 25.0, 10.0, 10.0]
            goal.stiffness = stiffness
            goal.damping = (2 * np.sqrt(stiffness)).tolist()
        elif len(position) == 6:
            goal.mode = 'ee_cartesian_ds'
            stiffness = [80.0, 80.0, 80.0, 5.0, 5.0, 5.0]
            goal.stiffness = stiffness
            goal.damping = (2 * np.sqrt(stiffness)).tolist()

        goal.precision = GOAL_PRECISION
        goal.rate = GOAL_RATE
        goal.nullspace_gain = NULLSPACE_GAIN

        if nullspace is None:
            goal.nullspace_reference = self.q
        else:
            goal.nullspace_reference = nullspace

        goal.reference = position
        goal.velocity_reference = np.zeros(6)
        return goal

    def send_position_command(self, position, nullspace=None,goal_time = None):
        if not isinstance(position,ControllerGoal):
            goal = self.create_goal(position, nullspace,goal_time)
        else:
            goal = position
        try:
            self.client.wait_for_server()
            self.client.send_goal(goal)
            self.client.wait_for_result()
        except actionlib.ActionException as e:
            rospy.logerr(f"Action client error: {e}")
            return False
        return True

    def move_towards_hand(self):
        rospy.loginfo("Moving towards hand")

        target_position = np.array(self.hand_pose) / 1000
        current_position = np.array(self.ee_pose[:3])
        fixed_orientation = self.ee_pose[3:]

        if target_position.all() == 0:
            target_position = current_position

        nullspace = self.q
        position_threshold = 0.1
        while np.linalg.norm(target_position - current_position) > position_threshold:
            if np.array(self.hand_pose).all() == 0:
                target_position = target_position
            else:
                target_position = np.array(self.hand_pose) / 1000

            current_position = np.array(self.ee_pose[:3])
            rospy.loginfo(f"target reached: {not np.linalg.norm(target_position - current_position) > position_threshold}, currentpos: {np.round(current_position, 3)}, target: {np.round(target_position, 3)}")
            target_pose = target_position.tolist() + fixed_orientation
            goal_time = 1
            self.send_position_command(target_pose, nullspace,goal_time)



        rospy.loginfo("Reached the hand position")
        return
    
    def test(self):
        rot= np.deg2rad([107, -47, -11, 100, -82, -82, -35])
        count = 0
        while not rospy.is_shutdown():
            if count == 0:
                rospy.loginfo("Sending pre-defined position command")
                self.send_position_command(rot)

            target = self.ee_pose
     
            # for some reason this axis is flipped (KUKA 14 only?)
            target[4] = -target[4] 
        
            # Modify values to test if arm orientation changes
            target[0] += 0.1
            target[1] -= 0.1
            target[2] += 0.1

            rospy.loginfo("sending dynamic position command, forward")
            self.send_position_command(target)

            target = self.ee_pose
            target[4] = -target[4] 

            # Move back to original position
            target[0] -= 0.1
            target[1] += 0.1
            target[2] -= 0.1

            rospy.loginfo("sending dynamic position command, backwards")
            self.send_position_command(target)

            count += 1
    

            


if __name__=='__main__':
    rospy.init_node("tet")
    controller = RoboticArmController()
    controller.shake_arm(0.05,1)