#!/usr/bin/python3
import rosgraph
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys
import numpy as np
import actionlib
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction

class SoftHandController:
    def __init__(self, fake=False):
        self.fake = fake
        self.position = None

        if not self.fake:
            self.ros_running = rosgraph.is_master_online()
            if self.ros_running:
                try:
                    rospy.init_node('hand_controller')
                except:
                    rospy.logwarn("Cannot initialize node 'hand_controller' as it has already been initialized at the top level as a ROS node.")

                self.client = actionlib.SimpleActionClient('/qbhand1/control/qbhand1_synergy_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
                rospy.loginfo('Waiting for joint trajectory action')
                self.client.wait_for_server()
                rospy.loginfo('Found joint trajectory action!')
                rospy.Subscriber('/qbhand1/control/joint_states', JointState, self.callback)
            else:
                print("ROS is offline! Shutting down")
                sys.exit(0)
        else:
            print("[INFO] Running qb_hand in mock mode. No connection to real hardware or ROS required.")
            self.position = 0.5  # Mock an initial position

    def callback(self, msg):
        if not self.fake:
            self.position = msg.position[0]

    def send_goal(self, mode:str, duration:int):
        '''
        modes: 'open', 'closed', 'partial'
        '''
        if not self.fake:
            rospy.loginfo('{} for {} s'.format(mode, duration))
     
        goal = self.get_qbhand_goal(mode, duration)
        if not self.fake:
            self.client.send_goal_and_wait(goal)
 

    def get_qbhand_goal(self, mode='open', duration=5, n_interval=20):
        if not self.fake:
            while self.position is None:
                rospy.Rate(1).sleep()
                rospy.loginfo("waiting for joint position")
        else:
            print("[MOCK] Generating mock trajectory")

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['qbhand1_synergy_joint']

        if mode == 'open':
            tp = np.linspace(self.position, 0.00, n_interval)
        elif mode == 'close':
            tp = np.linspace(self.position, 0.8, n_interval)
        elif mode == 'partial':
            tp = np.linspace(self.position, 0.5, n_interval)
        else:
            raise NotImplementedError

        tt = np.linspace(0, duration, n_interval)

        for i in range(n_interval):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(float(tt[i]))
            point.positions = [float(tp[i])]
            goal.trajectory.points.append(point)

        return goal

if __name__ == '__main__':
    # Example: Pass fake=True to run in mock mode
    controller = SoftHandController(fake=True)
    controller.send_goal('open', 1)
