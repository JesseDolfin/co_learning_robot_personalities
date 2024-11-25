#!/usr/bin/env python3
import time
from typing import List, Literal

import numpy as np
from scipy.spatial.transform import Rotation as R
import signal
import sys

import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from iiwa_impedance_control.msg import JointTrajectoryExecutionAction, CartesianTrajectoryExecutionAction
from geometry_msgs.msg import PoseStamped
from co_learning_messages.msg import hand_pose
from dynamic_reconfigure.client import Client
from controller_manager_msgs.srv import SwitchController, ListControllers
from iiwa_impedance_control.msg import CartesianTrajectoryExecutionGoal
from iiwa_impedance_control.msg import JointTrajectoryExecutionGoal


# Constants
HOME_POSITION = [0.0,  0.0,   1.3,  1.0, -0.0, -0.0, -0.0]
INTERMEDIATE_POSITION = [0, 0, 1.5, 0, 0, np.sqrt(2) / 2, np.sqrt(2) / 2]
INTERMEDIATE_POSITION_JOINT = [np.pi/2,0,0,0,0,0,0]

class RoboticArmController:
    def __init__(self):
        self.q = None
        self.q_dot = None
        self.ee_pose = None
        self._effort_mag_save = 0
        self.goal_time = 5.0
        self.hand_pose = [0, 0, 0]
        self.type = 'none'
        self.save_target = None
        self.robot = None
        self.movement_finished = False
        self.fixed_orientation = None

        self.init_action_servers()
        self.init_subscriber_publishers()
    
        self.reconfigure_parameters()

        input("press enter to start the robot controller")

    def init_subscriber_publishers(self):
        self.joint_state = rospy.Subscriber("CartesianImpedanceController/joint_states", JointState, self.joint_callback, queue_size=10)
        self.cartesian_state = rospy.Subscriber("CartesianImpedanceController/cartesian_pose", PoseStamped, self.cartesian_callback,queue_size=10)
        self.hand_pose_sub = rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)
        self.publish_human_input = rospy.Publisher('human_input', Bool, queue_size=1)

    def init_action_servers(self):
        # Initialize action clients
        self.cartesian_action_client = actionlib.SimpleActionClient(
            '/CartesianImpedanceController/cartesian_trajectory_execution_action',
            CartesianTrajectoryExecutionAction)
        self.cartesian_action_client.wait_for_server()
        self.joint_action_client = actionlib.SimpleActionClient(
            '/JointImpedanceController/joint_trajectory_execution_action', JointTrajectoryExecutionAction)

        # Wait for action servers to become available
        try:
            rospy.loginfo("Waiting for cartesian_trajectory_execution action server...")
            self.cartesian_action_client.wait_for_server()
            rospy.loginfo("cartesian_trajectory_execution action server found!")
        except Exception as e:
            rospy.logwarn(f"Cartesian trajectory action server not found: {e}")

        try:
            rospy.loginfo("Waiting for joint_trajectory_execution action server...")
            self.joint_action_client.wait_for_server()
            rospy.loginfo("joint_trajectory_execution server found!")
        except Exception as e:
            rospy.logwarn(f"Joint trajectory action server not found: {e}")

        # Initialize dynamic reconfigure client for Cartesian Impedance Controller
        try:
            rospy.loginfo("Waiting for dynamic reconfigure CartesianImpedanceController server...")
            self.dynamic_reconfigure_cartesian_impedance_controller_client = Client(
                "/CartesianImpedanceController/dynamic_reconfigure_server_node/", timeout=10, config_callback=None)
            rospy.loginfo("Dynamic reconfigure CartesianImpedanceController server found!")
        except Exception as e:
            rospy.logwarn(f"Dynamic reconfigure server not found: {e}")

        # Set up controller manager service proxies
        try:
            rospy.loginfo("Waiting for iiwa controller manager services...")
            
            # Switch controller service
            rospy.wait_for_service('iiwa/controller_manager/switch_controller')
            self.controller_manager = rospy.ServiceProxy('iiwa/controller_manager/switch_controller', SwitchController,persistent=True)
            
            # List controllers service
            rospy.wait_for_service('iiwa/controller_manager/list_controllers')
            self.list_controllers = rospy.ServiceProxy('iiwa/controller_manager/list_controllers', ListControllers)

            rospy.loginfo("iiwa controller manager services found!")
        except Exception as e:
            rospy.logwarn(f"iiwa controller manager services not found: {e}")


    # Callback functions to handle action results and feedback
    def cartesian_trajectory_done_callback(self, status, result):
        self.movement_finished = True
        rospy.loginfo(f"Cartesian trajectory execution done with status: {status}, {result}")

    def cartesian_trajectory_active_callback(self):
        rospy.loginfo("Cartesian trajectory execution is active.")

    def cartesian_trajectory_feedback_callback(self, feedback):
        pass  # Feedback handling can be implemented here if needed

    def joint_trajectory_done_callback(self, status, result):
        self.movement_finished = True
        rospy.loginfo(f"Joint trajectory execution done with status: {status}, {result}")

    def joint_trajectory_active_callback(self):
        rospy.loginfo("Joint trajectory execution is active.")

    def joint_trajectory_feedback_callback(self, feedback):
        pass  # Feedback handling can be implemented here if needed

    def joint_callback(self, msg):
        """Callback function for joint_states subscriber."""
        self.q = msg.position
        self.q_dot = msg.velocity

        effort = msg.effort
        effort_magnitude = np.linalg.norm(effort)
        self._effort_mag_save = effort_magnitude

    def cartesian_callback(self,msg):
        self.ee_pose = msg.pose

    def hand_pose_callback(self, msg):
        """Callback function for hand_pose subscriber."""
        self.hand_pose = [msg.x, msg.y, msg.z]
        self.hand_pose_ee = self.frame_transform(self.hand_pose)

    def reconfigure_parameters(
        self,
        stiffnes_matrix: List[float] = [100.0, 100.0, 100.0, 50.0, 50.0, 50.0],
        nullspace_reference: List[float] = [0.0] * 7,
        nullspace_stiffness: List[float] = [100.0, 100.0, 50.0, 50.0, 50.0, 50.0, 10.0],
        nulspace_damping: List[float] = [0.7] * 7,
        seperate_axis: bool = False,
        translational_stiffness: float = 300.0,
        rotational_stiffness: float = 10.0
    ) -> None:
        """Reconfigure parameters for the Cartesian impedance controller."""
        try:
            # Ensure lists have correct lengths
            if len(stiffnes_matrix) != 6 or len(nullspace_reference) != 7 or len(nullspace_stiffness) != 7 or len(nulspace_damping) != 7:
                raise ValueError("Stiffness matrices and nullspace parameters must have correct lengths.")

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

                    "nullspace_stiffness_joint_1": nullspace_stiffness[0],
                    "nullspace_stiffness_joint_2": nullspace_stiffness[1],
                    "nullspace_stiffness_joint_3": nullspace_stiffness[2],
                    "nullspace_stiffness_joint_4": nullspace_stiffness[3],
                    "nullspace_stiffness_joint_5": nullspace_stiffness[4],
                    "nullspace_stiffness_joint_6": nullspace_stiffness[5],
                    "nullspace_stiffness_joint_7": nullspace_stiffness[6],

                    "nullspace_damping_ratio_joint_1": nulspace_damping[0],
                    "nullspace_damping_ratio_joint_2": nulspace_damping[1],
                    "nullspace_damping_ratio_joint_3": nulspace_damping[2],
                    "nullspace_damping_ratio_joint_4": nulspace_damping[3],
                    "nullspace_damping_ratio_joint_5": nulspace_damping[4],
                    "nullspace_damping_ratio_joint_6": nulspace_damping[5],
                    "nullspace_damping_ratio_joint_7": nulspace_damping[6]
                }
            )
        except ValueError as e:
            rospy.logwarn(f"ValueError: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            raise RuntimeError(f"An unexpected error occurred: {e}")
        
    def is_controller_running(self, controller_name):
        try:
            rospy.wait_for_service('iiwa/controller_manager/list_controllers')
            controllers = self.list_controllers().controller
            for controller in controllers:
                if controller.name == controller_name and controller.state == 'running':
                    return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to list_controllers failed: {e}")
        return False

    def send_trajectory_goal(self, goal, mode: Literal["joint", "cartesian"]):
        self.movement_finished = False
        rospy.loginfo("sending trajectory goal")
        try:
            # Determine which controller to start and stop
            start_controller = '/CartesianImpedanceController' if mode == 'cartesian' else '/JointImpedanceController'
            stop_controller = '/JointImpedanceController' if mode == 'cartesian' else '/CartesianImpedanceController'

            # Check if the target controller is already running
            if self.is_controller_running(start_controller):
                rospy.loginfo(f"{start_controller} is already running; no switch necessary.")
            else:
                rospy.loginfo(f"Switching to {start_controller}...")
                response = self.controller_manager(
                    start_controllers=[start_controller],
                    stop_controllers=[stop_controller],
                    strictness=1, start_asap=True, timeout=0.0)
                
                if not response.ok:
                    rospy.logerr("Failed to switch controllers")
                    raise RuntimeError("Controller switch failed")
            # Check and create goal based on mode
            if mode == "cartesian":
                # If the goal is a list, verify its length and create a Cartesian goal
                if isinstance(goal, list):
                    if len(goal) != 7:
                        raise ValueError("For Cartesian mode, goal must be a list with exactly 7 elements (x, y, z, x, y, z, w).")
                    rospy.logwarn("Goal is not a CartesianTrajectoryExecutionGoal; using provided list as the target.")
                    goal = self.create_cartesian_goal(target=goal)

                # Send the Cartesian goal
                self.cartesian_action_client.send_goal(
                    goal,
                    self.cartesian_trajectory_done_callback,
                    self.cartesian_trajectory_active_callback,
                    self.cartesian_trajectory_feedback_callback)

            elif mode == "joint":
                # If the goal is a list, verify its length and create a Joint goal
                if isinstance(goal, list):
                    if len(goal) != 7:
                        raise ValueError("For Joint mode, goal must be a list with exactly 7 elements (joint positions).")
                    rospy.logwarn("Goal is not a JointTrajectoryExecutionGoal; using provided list as joint positions.")
                    goal = self.create_joint_goal(joint_positions_goal=goal)
                    rospy.loginfo("Joint goal created")
                
                # Send the Joint goal
                rospy.loginfo("sending goal")
                self.joint_action_client.send_goal(
                    goal,
                    self.joint_trajectory_done_callback,
                    self.joint_trajectory_active_callback,
                    self.joint_trajectory_feedback_callback)
            
            else:
                rospy.logwarn("Invalid mode specified.")
                raise ValueError("Mode must be either 'cartesian' or 'joint'")
            
            # Monitor the movement
            rate = rospy.Rate(10)
            while not self.movement_finished:
                rate.sleep()

        except Exception as e:
            rospy.logerr(f"Unexpected error in send_trajectory_goal: {e}")


    def create_joint_goal(self, joint_positions_goal: List[float], joint_velocities_goal: List[float] = None):
        try:
            if len(joint_positions_goal) != 7:
                raise ValueError("Joint positions must have exactly 7 elements.")
            
            if joint_velocities_goal is None:
                joint_velocities_goal = [0.3] * 7
            elif len(joint_velocities_goal) != 7:
                raise ValueError("Velocity must have exactly 7 elements")

            goal = JointTrajectoryExecutionGoal()

            joint_positions_msg = Float64MultiArray()
            joint_positions_msg.data = joint_positions_goal

            joint_velocities_msg = Float64MultiArray()
            joint_velocities_msg.data = joint_velocities_goal

            goal.joint_positions_goal = joint_positions_msg
            goal.joint_velocities_goal = joint_velocities_msg

            return goal

        except ValueError as e:
            rospy.logwarn(f"ValueError in create_joint_goal: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"Unexpected error in create_joint_goal: {e}")
            raise RuntimeError(f"An unexpected error occurred in create_joint_goal: {e}")

    def create_cartesian_goal(self, target: List[float], velocity: List[float] = None, begin_point: List[float] = None):
        try:
            if len(target) != 7:
                raise ValueError("Target must have 7 elements")
            
            if velocity is None:
                velocity = [0.01, 0.2]
            elif len(velocity) != 2:
                raise ValueError("Velocity must have 2 elements.")

            if begin_point is None:
                begin_point = [99] * 7

            if len(begin_point) != 7:
                raise ValueError("Begin point must have exactly 7 elements.")

            goal = CartesianTrajectoryExecutionGoal()
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = rospy.Time.now()
            goal_pose_msg.pose.position.x = target[0]
            goal_pose_msg.pose.position.y = target[1]
            goal_pose_msg.pose.position.z = target[2]
            goal_pose_msg.pose.orientation.x = target[3]
            goal_pose_msg.pose.orientation.y = target[4]
            goal_pose_msg.pose.orientation.z = target[5]
            goal_pose_msg.pose.orientation.w = target[6]

            start_pose_msg = PoseStamped()
            start_pose_msg.header.stamp = rospy.Time.now()
            start_pose_msg.pose.position.x = begin_point[0]
            start_pose_msg.pose.position.y = begin_point[1]
            start_pose_msg.pose.position.z = begin_point[2]
            start_pose_msg.pose.orientation.x = begin_point[3]
            start_pose_msg.pose.orientation.y = begin_point[4]
            start_pose_msg.pose.orientation.z = begin_point[5]
            start_pose_msg.pose.orientation.w = begin_point[6]

            goal.pose_start = start_pose_msg
            goal.pose_goal = goal_pose_msg
            goal.translational_velocity_goal = velocity[0]
            goal.rotational_velocity_goal = velocity[1]

            return goal

        except ValueError as e:
            rospy.logwarn(f"ValueError in create_cartesian_goal: {e}")
            raise
        except Exception as e:
            rospy.logerr(f"Unexpected error in create_cartesian_goal: {e}")
            raise RuntimeError(f"An unexpected error occurred in create_cartesian_goal: {e}")

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
        rate = rospy.Rate(10) 
        interaction_detected = False
        delta_threshold = 1 
        previous_effort_magnitude = None

        while time.time() - start_time < duration:
            effort_magnitude = self._effort_mag_save
            if previous_effort_magnitude is not None:
                effort_delta = abs(effort_magnitude - previous_effort_magnitude)
                rospy.loginfo(f"Effort delta: {effort_delta:.2f}")
                if effort_delta > delta_threshold:
                    rospy.logdebug(f"Effort delta: {effort_delta}")
                    rospy.logdebug(f"Delta threshold: {delta_threshold}")
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
            update (bool): Whether to update the target's orienation based on the current hand orientation.
        """
        rospy.loginfo("Moving towards hand")

        # Turn on cartesian controller to obtain ee_pose
        ok = False
        if self.is_controller_running('/JointImpedanceController'): 
            ok = self.controller_manager( 
                    start_controllers=['/CartesianImpedanceController'],
                    stop_controllers=['/JointImpedanceController'],
                    strictness=1, start_asap=True, timeout=0.0)
        elif self.is_controller_running('/CartesianImpedanceController'):
            ok = True
        else:
            raise RuntimeError(
                "No correct controller is currently running. Ensure that either "
                "JointImpedanceController or CartesianImpedanceController is active.")

        rate = rospy.Rate(10)
        self.ee_pose = None # obtain latest pose information
        while self.ee_pose is None:
            rate.sleep()

        if self.fixed_orientation is None or update:
            self.fixed_orientation = self.ee_pose.orientation # ROS MSG
        else:
            self.fixed_orientation = [-0.5,-0.3,0.6,0.27,0.56,0.32,0.72] #TODO: find correct orientation

        pos = self.ee_pose.position
        current_position = np.array([pos.x,pos.y,pos.z])
        #TODO: change hand pose to also use a PoseStamed msg? -> orientation does not translate well in that case

        # Wait for hand to be detected if not already
        while np.all(self.hand_pose) == 0:
            rospy.loginfo("Waiting for hand to be detected...")
            rate.sleep()

        target_position_arm = self.hand_pose_ee 
        target_position_arm[2] = max(target_position_arm[2], 0.1) # Z- value cannot be too low

        error = np.linalg.norm(target_position_arm - current_position)

        position_threshold = 0.2
        rate = rospy.Rate(10)

        while error > position_threshold:
   
            target_position_arm = self.hand_pose_ee
            target_position_arm[2] = max(target_position_arm[2], 0.1) 
            velocity = [0.01, 0.1]

            goal = self.create_cartesian_goal([target_position_arm,0,0,0,1],velocity) # Fake orientation
            goal.orientation = self.fixed_orientation # Correct orientation

            self.send_trajectory_goal(goal, "cartesian")

            pos = self.ee_pose.position
            current_position = np.array([pos.x,pos.y,pos.z])

            rate.sleep()
            error = np.linalg.norm(target_position_arm - current_position)
            rospy.loginfo(f"Error norm: {error:.3f}, threshold: {position_threshold}")

        rospy.loginfo("Reached the hand position")

        # Wait for arm to settle
        time.sleep(1.0)

        interaction_detected = self.detect_human_interaction(duration=5.0)
        msg = Bool()
        msg.data = interaction_detected
        rospy.loginfo("Human interaction detected." if interaction_detected else "No human interaction detected.")
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
        offset = np.array([0.356, 0.256, 2.643])  # Experimental values
        rot_x_180 = np.array([[1,  0,  0],
                              [0, -1,  0],
                              [0,  0, -1]])

        rot_z_90 = np.array([[0, -1,  0],
                             [1,  0,  0],
                             [0,  0,  1]])
        rot_tot = np.dot(rot_x_180, rot_z_90)

        r = R.from_matrix(rot_tot)

        transform = np.column_stack((rot_tot, offset))
        transform = np.vstack((transform, np.array([0, 0, 0, 1])))

        target_hom = np.append(target[:3], 1)
        transformed_target = np.dot(transform, target_hom)[:3]

        return transformed_target
    
    def test(self,n = 0):
        drop = np.deg2rad([55, -40, -8, 82, 5, 50, 0]).tolist()
        if n == 0:
            goal = self.create_joint_goal(drop)
            self.send_trajectory_goal(goal,'joint')

        if n == 1:
            ok = self.controller_manager( 
                        start_controllers=['/CartesianImpedanceController'],
                        stop_controllers=['/JointImpedanceController'],
                        strictness=1, start_asap=True, timeout=0.5)
            
            rospy.loginfo(f"controller switched with {ok}")

            pos = self.ee_pose.position
            ori = self.ee_pose.orientation

            rospy.loginfo(f"obtained ee_pose:{pos}")
            rospy.loginfo(f"obtained orientation{ori}")

            rospy.loginfo("creating goal ...")

            velocity = [0.01, 0.1]

            goal = self.create_cartesian_goal([pos.x,pos.y,pos.z,0,0,0,1],velocity) # Fake orientation
            goal.orientation = ori # Correct orientation

            rospy.loginfo(f"created goal is:{goal}")
            rospy.loginfo("sending goal ...")

            self.send_trajectory_goal(goal,'cartesian')






if __name__ == '__main__':
    rospy.init_node("RoboticArmController")
    controller = RoboticArmController()
    controller.test(0)
    controller.test(1)

    

    #controller.move_towards_hand(True)


    #drop = np.deg2rad([55, -40, -8, 82, 5, 50, 0]).tolist()
    #controller.send_trajectory_goal(INTERMEDIATE_POSITION_JOINT,'joint')


    # Define the Euler angles for a 90-degree rotation around the z-axis
    # roll, pitch, yaw = 0, 90, 0

    # # Convert the Euler angles to a quaternion
    # rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    # quaternion = rotation.as_quat()  # [x, y, z, w]

    # # Define the goal with the quaternion for orientation
    # goal = controller.create_cartesian_goal([0.2, 0.2, 0.5, *quaternion], [0.01, 0.01])

    # # Send the goal to the controller
    # controller.send_trajectory_goal([0,np.pi/2,0,0,0,0,0],'joint')
    # controller.send_trajectory_goal(INTERMEDIATE_POSITION,'joint')
    # controller.send_trajectory_goal(goal, mode="cartesian")

    rospy.spin()

