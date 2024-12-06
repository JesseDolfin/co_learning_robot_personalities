#!/usr/bin/env python
import numpy as np
from typing import List
import rospy
import actionlib
from scipy.spatial.transform import Rotation as R
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from iiwa_impedance_control.msg import CartesianTrajectoryExecutionAction, CartesianTrajectoryExecutionGoal
from iiwa_impedance_control.msg import JointTrajectoryExecutionAction, JointTrajectoryExecutionGoal
from co_learning_messages.msg import hand_pose
from co_learning_messages.msg import secondary_task_message
from geometry_msgs.msg import Quaternion
import signal
import sys
import time


class RobotArmController():
    def __init__(self,type):
        self._effort_mag_save = None
        self.pose = None
        self.pose_ref = None
        self.fixed_orientation = None
        self.trajectory_state = False
        self.hand_detected = False
        self.hand_pose = None
        self.handover_status = 0

        if type == 'impatient':
            self.type = 'fast'
        elif type == 'patient':
            self.type = 'slow'
        else: self.type = 'baseline'

        self.init_ros()
        self.reconfigure_parameters()
        signal.signal(signal.SIGINT, self.signal_handler)
        input("Press ENTER to start the robot controller")
       

    def signal_handler(self, sig, frame):
        rospy.signal_shutdown("Shutdown signal received.")
        sys.exit(0)
        

    def init_ros(self):
        self.task_status = rospy.Subscriber("/Task_status", secondary_task_message, self.task_status)
        self.publish_human_input = rospy.Publisher('/human_input', Bool, queue_size=1)
        self.hand_pose_sub = rospy.Subscriber('/hand_pose', hand_pose, self.hand_pose_callback)
        self.joint_state = rospy.Subscriber("/CartesianImpedanceController/joint_states", 
                                            JointState, self.cartesian_joint_callback, queue_size=10)
        # Subscribe to the pose topic
        rospy.Subscriber('/CartesianImpedanceController/cartesian_pose', PoseStamped,
                            self.cartesian_pose_callback)
        rospy.Subscriber('/CartesianImpedanceController/trajectory', PoseStamped,
                            self.trajectory_callback)
        
        self.cartesian_action_client = actionlib.SimpleActionClient(
                '/CartesianImpedanceController/cartesian_trajectory_execution_action', CartesianTrajectoryExecutionAction)
        
        self.joint_action_client = actionlib.SimpleActionClient(
            '/JointImpedanceController/joint_trajectory_execution_action', JointTrajectoryExecutionAction)

        try:
            rospy.loginfo("Waiting for cartesian_trajectory_execution action server...")
            self.cartesian_action_client.wait_for_server()
            rospy.loginfo("cartesian_trajectory_execution action server found!")
        except:
            rospy.loginfo("cartesian_trajectory_execution action server not found...")
        try:
            rospy.loginfo("Waiting for joint_trajectory_execution action server...")
            self.joint_action_client.wait_for_server()
            rospy.loginfo("joint_trajectory_execution server found!")
        except:
            rospy.loginfo("joint_trajectory_execution server not found...")
        try:
            rospy.loginfo("Waiting for dynamic reconfigure CartesianImpedanceController server...")
            self.dynamic_reconfigure_cartesian_impedance_controller_client = dynamic_reconfigure.client.Client(
                "/CartesianImpedanceController/dynamic_reconfigure_server_node/", timeout=10, config_callback=None)
            rospy.loginfo("Dynamic reconfigure CartesianImpedanceController server found!")
        except:
            rospy.loginfo("Dynamic reconfigure CartesianImpedanceController server not found...")
        try:
            rospy.loginfo("Waiting for iiwa controller manager...")
            rospy.wait_for_service('/iiwa/controller_manager/switch_controller')
            self.controller_manager = rospy.ServiceProxy('/iiwa/controller_manager/switch_controller',
                                                            SwitchController)
            rospy.loginfo("Iiwa controller manager found!")
        except:
            rospy.loginfo("Iiwa controller manager not found...")
        rospy.sleep(1)


    def reconfigure_parameters(
        self,
        stiffnes_matrix: List[float] = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0],
        nullspace_reference: List[float] = [0.0] * 7,
        nullspace_stiffness: List[float] = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0],
        nulspace_damping: List[float] = [0.7] * 7,
        seperate_axis: bool = False,
        translational_stiffness: float = 650.0,
        rotational_stiffness: float = 80.0
    ) -> None:
        """Reconfigure parameters for the Cartesian impedance controller."""
        try:
            # # Ensure lists have correct lengths
            # if len(stiffnes_matrix) != 6 or len(nullspace_reference) != 7 or len(nullspace_stiffness) != 7 or len(nulspace_damping) != 7:
            #     raise ValueError("Stiffness matrices and nullspace parameters must have correct lengths.")

            # Update configuration
            self.dynamic_reconfigure_cartesian_impedance_controller_client.update_configuration(
                {
                    "seperate_axis": seperate_axis,
                    "translational_stiffness": translational_stiffness,
                    "rotational_stiffness": rotational_stiffness,

                    "translational_stiffness_x": stiffnes_matrix[0],
                    "translational_stiffness_y": stiffnes_matrix[1],
                    "translational_stiffness_z": stiffnes_matrix[2],
                    "rotational_stiffness_alpha": stiffnes_matrix[3],
                    "rotational_stiffness_theta": stiffnes_matrix[4],
                    "rotational_stiffness_phi": stiffnes_matrix[5],

                    "nullspace_control": True,
                    "q_nullspace_joint_1": nullspace_reference[0],
                    "q_nullspace_joint_2": nullspace_reference[1],
                    "q_nullspace_joint_3": nullspace_reference[2],
                    "q_nullspace_joint_4": nullspace_reference[3],
                    "q_nullspace_joint_5": nullspace_reference[4],
                    "q_nullspace_joint_6": nullspace_reference[5],
                    "q_nullspace_joint_7": nullspace_reference[6],

                    # BUG: MAKING THE VALUES FOR NULLSPACE TO HIGH WILL cause the arm not to follow reference

                    # "nullspace_stiffness_joint_1": nullspace_stiffness[0],
                    # "nullspace_stiffness_joint_2": nullspace_stiffness[1],
                    # "nullspace_stiffness_joint_3": nullspace_stiffness[2],
                    # "nullspace_stiffness_joint_4": nullspace_stiffness[3],
                    # "nullspace_stiffness_joint_5": nullspace_stiffness[4],
                    # "nullspace_stiffness_joint_6": nullspace_stiffness[5],
                    # "nullspace_stiffness_joint_7": nullspace_stiffness[6],

                    # "nullspace_damping_ratio_joint_1": nulspace_damping[0],
                    # "nullspace_damping_ratio_joint_2": nulspace_damping[1],
                    # "nullspace_damping_ratio_joint_3": nulspace_damping[2],
                    # "nullspace_damping_ratio_joint_4": nulspace_damping[3],
                    # "nullspace_damping_ratio_joint_5": nulspace_damping[4],
                    # "nullspace_damping_ratio_joint_6": nulspace_damping[5],
                    # "nullspace_damping_ratio_joint_7": nulspace_damping[6]
                }
            )
        except ValueError as e:
            rospy.logwarn(f"ValueError: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            raise RuntimeError(f"An unexpected error occurred: {e}")
        
    def task_status(self,msg):
        self.handover_status = msg.handover_successful
        
    def hand_pose_callback(self, msg):
        hand_pose = np.array([msg.x, msg.y, msg.z],dtype="float64")
        if np.all(hand_pose) == 0:
            self.hand_detected = False
            self.hand_pose = hand_pose
        else:
            self.hand_detected = True
            self.hand_pose = self.frame_transform(hand_pose)
        

    def cartesian_joint_callback(self,data):
        effort = data.effort
        effort_magnitude = np.linalg.norm(effort)
        self._effort_mag_save = effort_magnitude

    def cartesian_pose_callback(self, data):
        self.pose = data.pose

    def trajectory_callback(self, data):
        self.pose_ref = data.pose
        
    def cartesian_trajectory_active_callback(self):
        rospy.logdebug("Cartesian Trajectory Active callback")

    def cartesian_trajectory_feedback_callback(self, feedback):
        pass

    def cartesian_trajectory_done_callback(self, status, result):
        rospy.loginfo('Cartesian Trajectory Done callback. Result: ' + str(result))
        self.trajectory_state = True 
     
    def joint_trajectory_active_callback(self):
        rospy.loginfo("Joint Trajectory Active callback")

    def joint_trajectory_feedback_callback(self, feedback):
        pass

    def joint_trajectory_done_callback(self, status, result):
        self.trajectory_state = True
        response = self.controller_manager(start_controllers=['/CartesianImpedanceController'],
                                           stop_controllers=['/JointImpedanceController'], strictness=1,
                                           start_asap=True, timeout=0.0)
        if not response.ok:
            rospy.logerr("Failed to switch controllers")
        else:
            rospy.loginfo("Controllers switched")

    def send_cartesian_trajectory_goal(self, position, orientation, velocity=None, trajectory_start=None):
        rospy.loginfo("Sending cartesian trajectory goal: " + str(position) + " and adjusted start: " + str(trajectory_start))
        if trajectory_start is None:
            start_pose = [99] * 7
        else:
            pass

        if velocity is None:
            velocity = [0.2,0.6]
        else:
            pass

        if self.type == 'fast':
            velocity = [0.4,0.6]
        elif self.type == 'slow':
            velocity = [0.1,0.6]

        goal = CartesianTrajectoryExecutionGoal()
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.seq = 0
        goal_pose_msg.header.stamp = rospy.Time.now()
        goal_pose_msg.pose.position.x = position[0]
        goal_pose_msg.pose.position.y = position[1]
        goal_pose_msg.pose.position.z = position[2]
        goal_pose_msg.pose.orientation = orientation
       
        start_pose_msg = PoseStamped()
        start_pose_msg.header.seq = 0
        start_pose_msg.header.stamp = rospy.Time.now()
        start_pose_msg.pose.position.x = start_pose[0] 
        start_pose_msg.pose.position.y = start_pose[1] 
        start_pose_msg.pose.position.z = start_pose[2]
        start_pose_msg.pose.orientation.x = start_pose[3]
        start_pose_msg.pose.orientation.y = start_pose[4]
        start_pose_msg.pose.orientation.z = start_pose[5]
        start_pose_msg.pose.orientation.w = start_pose[6]

        goal.pose_goal = goal_pose_msg
        goal.pose_start = start_pose_msg
        goal.translational_velocity_goal = velocity[0]
        goal.rotational_velocity_goal = velocity[1]
        self.trajectory_state = False
        self.cartesian_action_client.send_goal(goal, self.cartesian_trajectory_done_callback,
                                                self.cartesian_trajectory_active_callback,
                                                self.cartesian_trajectory_feedback_callback)
        
        rate = rospy.Rate(20)
        while not self.trajectory_state:
            rate.sleep()

    def send_joint_trajectory_goal(self, joint_positions_goal, joint_velocities_goal=None):
        rospy.loginfo("Sending joint positions goal: " + str(joint_positions_goal) + " and joint velocities goal:" + str(
            joint_velocities_goal))
        
        if joint_velocities_goal is None:
            joint_velocities_goal = [0.5] * 7  
        
        if self.type == 'fast':
            joint_velocities_goal = [0.8] * 7  
        elif self.type == 'slow':
            joint_velocities_goal = [0.2] * 7  
        
        response = self.controller_manager(start_controllers=['/JointImpedanceController'],
                                            stop_controllers=['/CartesianImpedanceController'], strictness=1,
                                            start_asap=True, timeout=0.0)
        if not response.ok:
            rospy.logerr("Failed to switch controllers")
        else:
            goal = JointTrajectoryExecutionGoal()
            goal.joint_positions_goal.data = joint_positions_goal
            goal.joint_velocities_goal.data = joint_velocities_goal
            self.trajectory_state = False
            self.joint_action_client.send_goal(goal, self.joint_trajectory_done_callback,
                                                self.joint_trajectory_active_callback,
                                                self.joint_trajectory_feedback_callback)
            rate = rospy.Rate(20)
            while not self.trajectory_state:
                rate.sleep()

    def detect_human_interaction(self, duration=3.0,wait_for_hand=False):
        """
        Detect human interaction with the robot arm over a specified duration.
        Parameters:
        duration (float): Time in seconds to monitor for human interaction.
        Returns:
        bool: True if human interaction is detected, False otherwise.
        """
        rospy.loginfo(f"Monitoring for human interaction for {duration} seconds...")
        
        rate = rospy.Rate(30)
        wait_time = rospy.Time.now()
        wait_duration = rospy.Duration(2) # S

        # Wait for the arm to settle
        while rospy.Time.now() - wait_time < wait_duration:
            rate.sleep()
        
        interaction_detected = False
        delta_threshold = 1
        previous_effort_magnitude = None
        start_time = rospy.Time.now()
        duration_ros = rospy.Duration(duration)
        
        while (rospy.Time.now() - start_time) < duration_ros:
            effort_magnitude = self._effort_mag_save
            if wait_for_hand:
                rospy.loginfo(f"hand_status:{self.hand_detected}")

            if wait_for_hand and self.hand_detected:
                break
            
            if previous_effort_magnitude is not None:
                effort_delta = abs(effort_magnitude - previous_effort_magnitude)
                if effort_delta > delta_threshold:
                    interaction_detected = True
                    rospy.loginfo("Human interaction detected based on effort delta.")
                    break
                
            previous_effort_magnitude = effort_magnitude
            rate.sleep()
            
        return interaction_detected

    def move_towards_hand(self, update=False):
        """
        Move the robot's end-effector towards the detected hand position.

        Parameters:
            update (bool): Whether to update the target's orienation based on the current hand orientation.
        """
        rospy.loginfo("Moving towards hand")

        rate = rospy.Rate(10)

        # We save the position and orientation the first time this function is called as a reference
        if self.fixed_orientation is None or update: 
            self.fixed_orientation = self.pose.orientation 
            p = self.pose.position
            self.ref_pos = np.array([p.x,p.y,p.z],dtype="float64")
        else:
            pass

        # Unpack current position
        pos = self.pose.position
        current_position = np.array([pos.x,pos.y,pos.z],dtype="float64")
        
        # We wait until either a hand is detected OR interaction with the arm is detected
        rospy.loginfo("Waiting to detect hand")
        self.detect_human_interaction(duration=100,wait_for_hand=True)

        # If self.hand_pose.x is close to 0 we know that this value is out of bounds (happens when no hand is detected)
        # To make sure the arm does not crash into itself we give it the reference position as target
        if abs(self.hand_pose[0]) > 0.1:
            target_position_arm = self.hand_pose
        else: target_position_arm = self.ref_pos

        # Z- value cannot be too low, making sure arm does not smash into table
        target_position_arm[2] = max(target_position_arm[2], 0.1) 
        error = np.linalg.norm(target_position_arm - current_position)

        position_threshold = 0.2
        while error > position_threshold:
            # When we have a update on the handover we dont need to move to hand anymore because either the handover 
            # Is already done or the person already failed the secondary task
            if self.handover_status in [-1,1]:
                break

            if abs(self.hand_pose[0]) > 0.1:
                target_position_arm = self.hand_pose
            else: target_position_arm = self.ref_pos

            target_position_arm[2] = max(target_position_arm[2], 0.1) 
            self.send_cartesian_trajectory_goal(target_position_arm,self.fixed_orientation)

            # Update our current position and error values
            pos = self.pose.position
            current_position = np.array([pos.x,pos.y,pos.z],dtype="float64")
            error = np.linalg.norm(target_position_arm - current_position)
            rate.sleep()
        rospy.loginfo("Reached the hand position")

        duration = 4
        if self.type == 'fast':
            duration = 5
        elif self.type == 'slow':
            duration = 3

        # When the hand is reached we will wait a little while to detect human interaction, differs per personality
        interaction_detected = self.detect_human_interaction(duration)
        msg = Bool()
        msg.data = interaction_detected # For the RL algorithm
        rospy.loginfo("Human interaction detected." if interaction_detected else "No human interaction detected.")
        self.publish_human_input.publish(msg)

    def frame_transform(self, target):
        """
        Transform the target position from camera frame to robot frame.

        Parameters:
            target (array): Target position in camera frame.

        Returns:
            array: Transformed position in robot frame.
        """
        # Calibration parameters
        offset = np.array([0.356, 0.256, 2.643],dtype="float64")  # Experimental values
        rot_x_180 = np.array([[1,  0,  0],
                              [0, -1,  0],
                              [0,  0, -1]],dtype="float64")

        rot_z_90 = np.array([[0, -1,  0],
                             [1,  0,  0],
                             [0,  0,  1]],dtype="float64")
        rot_tot = np.dot(rot_x_180, rot_z_90)

        r = R.from_matrix(rot_tot)

        transform = np.column_stack((rot_tot, offset))
        transform = np.vstack((transform, np.array([0, 0, 0, 1],dtype="float64")))

        target_hom = np.append(target[:3], 1)
        transformed_target = np.dot(transform, target_hom)[:3]

        return transformed_target
                    
if __name__ == '__main__':
    rospy.init_node("RoboticArmController")

    drop = np.deg2rad([55, -40, -8, 82, 5, 50, 0]).tolist()
    serve = np.deg2rad([107, -47, -11, 100, -82, -82, -35]).tolist()
    target = [np.pi/2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocity = [0.3] * 7

    controller = RobotArmController()
    controller.send_joint_trajectory_goal(drop,velocity)
    controller.move_towards_hand(update=True)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)
    controller.move_towards_hand(update=False)

    # target_cart = [0.0,0.0,1.255]

    # orientation = Quaternion()
    # orientation.w = 1.0  # Default/neutral orientation
    # orientation.x = 0.0
    # orientation.y = 0.0
    # orientation.z = 0.0

    # controller.send_cartesian_trajectory_goal(target_cart,orientation)

    
    # # input("send to joint 2")
    # controller.send_joint_trajectory_goal(target_joint_2,velocity)
