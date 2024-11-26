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
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from co_learning_messages.msg import hand_pose
from co_learning_messages.msg import secondary_task_message

from robot.robot import Robot

# Constants
HOME_POSITION = [np.pi / 2, np.pi / 4, 0, -np.pi / 4, 0, np.pi / 4, 0]
INTERMEDIATE_POSITION = [np.pi / 2, 0, 0, 0, 0, 0, 0]

class RoboticArmController:
    def __init__(self):
        self.q = None
        self.q_dot = None
        self.ee_pose = [0, 0, 0, 0, 0, 0]
        self._effort_mag_save = 0
        self.goal_time = 5.0
        self.hand_pose = [0, 0, 0]
        self.type = 'none'
        self.save_target = None

        try:
            ns = rospy.get_param('/namespaces')
        except KeyError as e:
            rospy.logerr(f"Parameter '/namespaces' not found: {e}")
            sys.exit(1)

        if Robot is not None:
            self.robot = Robot(model=ns.replace('/', ''))
        else:
            rospy.logwarn("Robot class not available.")
            self.robot = None

        self.client = actionlib.SimpleActionClient(f"{ns}/torque_controller", ControllerAction)
        self.joint_state_sub = rospy.Subscriber(f"{ns}/joint_states", JointState, self.joint_callback, queue_size=10)
        self.hand_pose_sub = rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)
        self.task_status = rospy.Subscriber("/Task_status", secondary_task_message, self.task_status)

        self.publish_human_input = rospy.Publisher('human_input', Bool, queue_size=1)

        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        input("press ENTER to start the robot controller")
    def task_status(self,msg):
        self.handover_status = msg.handover_successful

    def joint_callback(self, msg):
        """Callback function for joint_states subscriber."""
        self.q = msg.position
        self.q_dot = msg.velocity

        effort = msg.effort
        effort_magnitude = np.linalg.norm(effort)
        self._effort_mag_save = effort_magnitude

    def hand_pose_callback(self, msg):
        """Callback function for hand_pose subscriber."""
        self.hand_pose = [msg.x, msg.y, msg.z]

    def create_goal(self, position, nullspace=None, goal_time=None, mode=None):
        """
        Create a ControllerGoal based on the desired position.

        Parameters:
            position (list): Desired position, can be joint angles (length 7) or Cartesian pose (length 6).
            nullspace (list, optional): Nullspace reference for redundancy resolution.
            goal_time (float, optional): Duration for the goal.
            mode (str, optional): Control mode.

        Returns:
            ControllerGoal: Configured goal object.
        """
        goal = ControllerGoal()

        if goal_time is None:
            if self.type == 'fast':
                goal.time = 2.0
            elif self.type == 'slow':
                goal.time = 7.0
            else:
                goal.time = 5.0
        else:
            goal.time = goal_time

        if len(position) == 7:
            goal.mode = 'joint_ds'
            stiffness = [100.0, 100.0, 50.0, 50.0, 25.0, 10.0, 10.0]
        elif len(position) == 6:
            if mode is None:
                goal.mode = 'ee_cartesian_ds'
                if self.type == 'fast':
                    stiffness = [220.0, 220.0, 220.0, 15.0, 15.0, 15.0]
                elif self.type == 'slow':
                    stiffness = [150.0, 150.0, 150.0, 15.0, 15.0, 15.0]
                else:
                    stiffness = [180.0, 180.0, 180.0, 15.0, 15.0, 15.0]
            elif mode == 'ee_cartesian':
                goal.mode = mode
                stiffness = [180.0, 180.0, 180.0, 15.0, 13.0, 13.0]
                time.sleep(goal.time)
            else:
                rospy.logwarn(f"Unknown mode '{mode}'. Defaulting to 'ee_cartesian_ds'.")
                goal.mode = 'ee_cartesian_ds'
                stiffness = [180.0, 180.0, 180.0, 15.0, 15.0, 15.0]
        else:
            raise ValueError("Requested position must be either joint angles (length 7) or Cartesian pose (length 6).")

        goal.stiffness = stiffness
        goal.damping = (2 * np.sqrt(stiffness)).tolist()
        goal.precision = 0.2
        goal.rate = 20

        if nullspace is None:
            goal.nullspace_reference = [0] * 7
            goal.nullspace_gain = [0] * 7
        else:
            goal.nullspace_reference = nullspace
            goal.nullspace_gain = [100, 100, 100, 100, 100, 0, 0]

        goal.reference = position
        goal.velocity_reference = [0.0] * 6

        return goal

    def send_position_command(self, position: Union[list, ControllerGoal], nullspace: list = None, goal_time: float = None, mode: str = None):
        """
        Send a position command to the robot.

        Parameters:
            position (list or ControllerGoal): Desired position or pre-configured ControllerGoal.
            nullspace (list, optional): Nullspace reference.
            goal_time (float, optional): Duration for the movement.
            mode (str, optional): Control mode.

        Returns:
            bool: True if the goal was sent successfully, False otherwise.
        """
        rospy.loginfo("sending position command")
        if not isinstance(position, ControllerGoal):
            goal = self.create_goal(position, nullspace, goal_time, mode)
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

    def detect_human_interaction(self, duration=5.0):
        """
        Detect human interaction with the robot arm over a specified duration.

        Parameters:
            duration (float): Time in seconds to monitor for human interaction.

        Returns:
            bool: True if human interaction is detected, False otherwise.
        """
        rospy.loginfo(f"Monitoring for human interaction for {duration} seconds...")
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz monitoring rate
        interaction_detected = False
        delta_threshold = 1  # Adjust this threshold based on your robot
        previous_effort_magnitude = None

        

        while time.time() - start_time < duration:
            print(time.time()-start_time < duration)
            effort_magnitude = self._effort_mag_save
            if previous_effort_magnitude is not None:
                effort_delta = abs(effort_magnitude - previous_effort_magnitude)
                rospy.loginfo(f"Effort delta: {effort_delta:.2f}")
                if effort_delta > delta_threshold:
                    print("effort_delta",effort_delta)
                    print("delta_threshold",delta_threshold)
                    interaction_detected = True
                    rospy.loginfo("Human interaction detected based on effort delta.")
                    break
            else:
                rospy.loginfo(f"Effort magnitude: {effort_magnitude:.2f}")

            previous_effort_magnitude = effort_magnitude
            rate.sleep()

        return interaction_detected



    def move_towards_hand(self, update=False):
        """
        Move the robot's end-effector towards the detected hand position.

        Parameters:
            update (bool): Whether to update the target based on the current hand position.
        """
        rospy.loginfo("Moving towards hand")
        if self.robot is not None and self.q is not None:
            ee_T = np.array(self.robot.fkine(self.q, end='iiwa_link_7', start='iiwa_link_0'))

            translation = ee_T[:3, 3]
            rot_mat = ee_T[:3, :3]
            r = R.from_matrix(rot_mat)
            euler_angles = r.as_euler('xyz', degrees=False)
            self.ee_pose = np.hstack((translation, np.flip(euler_angles)))
            rospy.loginfo(f"ee_pose:{self.ee_pose}")

        if update:
            self.fixed_orientation = self.ee_pose[3:]
            self.q_save = self.q

            # Correct for possible axis flipping
            if self.q is not None and self.q[4] < -1:
                self.fixed_orientation[1] = -self.fixed_orientation[1]

            fixed_position = self.ee_pose[:3]

            if np.all(np.array(self.hand_pose) == 0):
                target_position_arm = fixed_position
            else:
                target_position_cam = np.array(self.hand_pose)
                target_position_arm = self.frame_transform(target_position_cam)

            # When initially no hand is detected, keep checking before moving on
            wait_for_hand = np.all(np.array(self.hand_pose) == 0)
            self.save_target = None
        else:
            self.q_save = None
            wait_for_hand = False

        rospy.logwarn(f"self.q:{self.q}")
        rospy.logwarn(f"self.ee_pose[:3]:{self.ee_pose[:3]}")

        if self.save_target is not None:
            target_position_arm = self.save_target
        else:
            target_position_arm = self.frame_transform(np.array(self.hand_pose))

        current_position = np.array(self.ee_pose[:3])
        position_threshold = 0.2
        self.saved_pose = np.array(self.hand_pose)
        update_pose = False

        while (np.linalg.norm(target_position_arm - current_position) > position_threshold) or wait_for_hand:
            if self.handover_status in [-1,1]:
                break
            if self.robot is not None and self.q is not None:
                ee_T = np.array(self.robot.fkine(self.q, end='iiwa_link_7', start='iiwa_link_0'))

                translation = ee_T[:3, 3]
                rot_mat = ee_T[:3, :3]
                r = R.from_matrix(rot_mat)
                euler_angles = r.as_euler('xyz', degrees=False)
                self.ee_pose = np.hstack((translation, np.flip(euler_angles)))

            if np.any(np.array(self.hand_pose) != 0):  # When a hand is in the workspace
                hand_movement = np.linalg.norm(np.array(self.hand_pose) - self.saved_pose)
                update_pose = hand_movement > 0.0  # Update pose if hand moved significantly
                wait_for_hand = False  # Stop waiting as the hand is detected

                if update_pose:
                    self.saved_pose = np.array(self.hand_pose)

                target_position_arm = self.frame_transform(self.saved_pose)
                target_position_arm[2] = max(target_position_arm[2], 0.1)  # Prevent collision with the table

            error = np.linalg.norm(target_position_arm - current_position)
            rospy.loginfo(f"Error norm: {error:.3f}, threshold: {position_threshold}")

            target_pose = np.hstack((target_position_arm, self.fixed_orientation))

            goal_time = 2.0
            if self.type == 'fast':
                goal_time -= 1.0
            elif self.type == 'slow':
                goal_time += 1.0

            self.save_target = target_position_arm

            if update_pose:
                self.send_position_command(target_pose, self.q_save, goal_time)
            else:
                rospy.sleep(1.0)  # Sleep for 1 second

            current_position = np.array(self.ee_pose[:3])  # Update the current position

        rospy.loginfo("Reached the hand position")
        rospy.loginfo("Waiting for 5 seconds to detect human interaction...")
        time.sleep(2) # give the arm time to settle

        if self.handover_status not in [-1,1]:
            interaction_detected = self.detect_human_interaction(duration=5.0)

            msg = Bool()
            if interaction_detected:
                rospy.loginfo("Human interaction is detected during waiting period.")
                msg.data = True
            else:
                rospy.loginfo("No human interaction detected during waiting period.")
                msg.data = False
            self.publish_human_input.publish(msg)

        self.hand_pose = [0, 0, 0]

    def frame_transform(self, target):
        """
        Transform the target position from camera frame to robot frame.

        Parameters:
            target (array): Target position in camera frame.

        Returns:
            array: Transformed position in robot frame.
        """
        # Calibration parameters
        offset =    np.array([0.356, 0.256, 2.643])  # Experimental values
        rot_x_180 = np.array([[1,  0,  0],
                              [0, -1,  0],
                              [0,  0, -1]])
        
        rot_z_90 =  np.array([[0, -1,  0],
                              [1,  0,  0],
                              [0,  0,  1]])
        rot_tot =   np.dot(rot_x_180, rot_z_90)

        transform = np.column_stack((rot_tot, offset))
        transform = np.vstack((transform, np.array([0, 0, 0, 1])))

        target_hom = np.append(target[:3], 1)
        transformed_target = np.dot(transform, target_hom)[:3]

        return transformed_target

    def test(self,n=0):
        if n == 0:
            self.send_position_command([0,0,0,0,0,0,0]) # go to position 0

        if n == 1:
            self.send_position_command([ 0.02075587, -0.50972173,  0.24752321,  0.66051142,  -1.35919863, -2.69775329]) # Serve from forward kinematics


if __name__ == '__main__':
    try:
        rospy.init_node("RoboticArmController")
        node = RoboticArmController()
        node.test(0)
        #node.move_towards_hand()
        node.test(1)
    except rospy.ROSInterruptException:
        pass
