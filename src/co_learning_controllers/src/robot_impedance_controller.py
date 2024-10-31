#!/usr/bin/env python3
import sys
import time
from typing import Union

import numpy as np
from scipy.spatial.transform import Rotation as R


import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
#from iiwa_impedance_control.action import JointTrajectoryExecutionAction,CartesianTrajectoryExecutionAction
from geometry_msgs.msg import PoseStamped
from co_learning_messages.msg import hand_pose
from dynamic_reconfigure.client import Client
from controller_manager_msgs.srv import SwitchController
from iiwa_impedance_control.msg import CartesianTrajectoryExecutionAction, CartesianTrajectoryExecutionGoal
from iiwa_impedance_control.msg import JointTrajectoryExecutionAction, JointTrajectoryExecutionGoal



from robot.robot import Robot


# Constants
HOME_POSITION = [np.pi / 2, np.pi / 4, 0, -np.pi / 4, 0, np.pi / 4, 0]
INTERMEDIATE_POSITION = [np.pi / 2, 0, 0, 0, 0, 0, 0]

class RoboticArmController:
    def __init__(self):
        self.init_action_servers()

    # Dummy callback functions to handle action results and feedback
    def cartesian_trajectory_done_callback(self, status, result):
        rospy.loginfo("Cartesian trajectory execution done with status: {}".format(status))

    def cartesian_trajectory_active_callback(self):
        rospy.loginfo("Cartesian trajectory execution is active.")

    def cartesian_trajectory_feedback_callback(self, feedback):
        rospy.loginfo("Received feedback for cartesian trajectory execution.")

    def init_action_servers(self):
        self.cartesian_action_client = actionlib.SimpleActionClient(
            '/CartesianImpedanceController/cartesian_trajectory_execution_action',
            CartesianTrajectoryExecutionAction)
        self.joint_action_client = actionlib.SimpleActionClient(
            '/JointImpedanceController/joint_trajectory_execution_action', JointTrajectoryExecutionAction)

        try:
            rospy.loginfo("Waiting for cartesian_trajectory_execution action server...")
            self.cartesian_action_client.wait_for_server()
            rospy.loginfo("cartesian_trajectory_execution action server found!")
        except:
            rospy.loginfo("cartesian_trajectory_execution action server not found...", "warn")
        try:
            rospy.loginfo("Waiting for joint_trajectory_execution action server...")
            self.joint_action_client.wait_for_server()
            rospy.loginfo("joint_trajectory_execution server found!")
        except:
            rospy.loginfo("joint_trajectory_execution server not found...", "warn")
        try:
            rospy.loginfo("Waiting for dynamic reconfigure CartesianImpedanceController server...")
            self.dynamic_reconfigure_cartesian_impedance_controller_client = Client(
                "/CartesianImpedanceController/dynamic_reconfigure_server_node/", timeout=10, config_callback=None)
            rospy.loginfo("Dynamic reconfigure CartesianImpedanceController server found!")
        except:
            rospy.loginfo("Dynamic reconfigure CartesianImpedanceController server not found...", "warn")
        # try:
        #     rospy.loginfo("Waiting for iiwa controller manager...")
        #     rospy.wait_for_service('/iiwa/controller_manager/switch_controller')
        #     self.controller_manager = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller',
        #                                                 SwitchController)
        #     rospy.loginfo("Iiwa controller manager found!")
        # except:
        #     rospy.loginfo("Iiwa controller manager not found...", "warn")

         # Allow some time for action servers to start and connections to be established
        rospy.sleep(2)

    def send_cartesian_trajectory_goal(self, goal):
        goal = CartesianTrajectoryExecutionGoal()
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.seq = 0
        goal_pose_msg.header.stamp = rospy.Time.now()
        goal_pose_msg.pose.position.x = 0.3
        goal_pose_msg.pose.position.y = 0.3
        goal_pose_msg.pose.position.z = 0.3

        goal_pose_msg.pose.orientation.x = -0.383
        goal_pose_msg.pose.orientation.y = 0.924
        goal_pose_msg.pose.orientation.z = 0.0
        goal_pose_msg.pose.orientation.w = 0.0

        """
        If the trajectory need to start from a different point apply a start_point
        """
        # start_pose_msg = PoseStamped()
        # start_pose_msg.header.seq = 0
        # start_pose_msg.header.stamp = rospy.Time.now()
        # start_pose_msg.pose.position.x = start_pose[0] if start_pose else 99
        # start_pose_msg.pose.position.y = start_pose[1] if start_pose else 99
        # start_pose_msg.pose.position.z = 99
        # # if trajectory_start[2]:
        # #     start_pose_msg.pose.position.z = trajectory_start[2]
        # # else:
        # #     start_pose_msg.pose.position.z = self.screen_position_z
        # start_pose_msg.pose.orientation.x = 99
        # start_pose_msg.pose.orientation.y = 99
        # start_pose_msg.pose.orientation.z = 99
        # start_pose_msg.pose.orientation.w = 99
        
        #goal.pose_start = start_pose_msg
        goal.pose_goal = goal_pose_msg
        goal.translational_velocity_goal = 0.5
        goal.rotational_velocity_goal = 0.5
        self.cartesian_action_client.send_goal(goal, self.cartesian_trajectory_done_callback,
                                            self.cartesian_trajectory_active_callback,
                                            self.cartesian_trajectory_feedback_callback)
        
    def send_joint_trajectory_goal(self, joint_positions_goal, joint_velocities_goal):
        self.log("Sending joint positions goal: " + str(joint_positions_goal) + " and joint velocities goal:" + str(
            joint_velocities_goal))
        if self.use_ros:
            response = self.controller_manager(start_controllers=['/JointImpedanceController'],
                                            stop_controllers=['/CartesianImpedanceController'], strictness=1,
                                            start_asap=True, timeout=0.0)
            if not response.ok:
                self.log("Failed to switch controllers", "error")
            else:
                goal = JointTrajectoryExecutionGoal()
                goal.joint_positions_goal.data = joint_positions_goal
                goal.joint_velocities_goal.data = joint_velocities_goal
                self.joint_action_client.send_goal(goal, self.joint_trajectory_done_callback,
                                                self.joint_trajectory_active_callback,
                                                self.joint_trajectory_feedback_callback)
                
if __name__ == '__main__':
    rospy.init_node("test")
    controller = RoboticArmController() 
    controller.send_cartesian_trajectory_goal(CartesianTrajectoryExecutionGoal())
    rospy.spin() 
 