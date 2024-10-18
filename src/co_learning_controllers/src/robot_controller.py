#!/usr/bin/env python3

import sys
import time
from typing import Union

import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import rospy
    import actionlib
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from cor_tud_msgs.msg import ControllerAction, ControllerGoal
    from co_learning_messages.msg import hand_pose
except ImportError as e:
    print(f"Import Error: {e}")
    sys.exit(1)

try:
    from robot.robot import Robot
except ImportError as e:
    rospy.logerr(f"Could not import Robot class: {e}")
    Robot = None

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

        self.publish_human_input = rospy.Publisher('human_input', Bool, queue_size=1)

        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        rospy.sleep(2.5)  # Give enough time for the FRIoverlay app to start up

    def joint_callback(self, msg):
        """Callback function for joint_states subscriber."""
        self.q = msg.position
        self.q_dot = msg.velocity

        effort = msg.effort
        effort_magnitude = np.linalg.norm(effort)
        self._effort_mag_save = effort_magnitude

        if self.robot is not None and self.q is not None:
            ee_T = np.array(self.robot.fkine(self.q, end='iiwa_link_7', start='iiwa_link_0'))

            translation = ee_T[:3, 3]
            rot_mat = ee_T[:3, :3]
            r = R.from_matrix(rot_mat)
            euler_angles = r.as_euler('xyz', degrees=False)
            self.ee_pose = np.hstack((translation, np.flip(euler_angles)))

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
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz monitoring rate
        interaction_detected = False
        effort_threshold = 5.0  # Adjust this threshold based on your robot

        while (rospy.Time.now() - start_time).to_sec() < duration:
            effort_magnitude = self._effort_mag_save
            rospy.loginfo(f"Effort magnitude: {effort_magnitude:.2f}")
            if effort_magnitude > effort_threshold:
                interaction_detected = True
                rospy.loginfo("Human interaction detected based on effort magnitude.")
                break
            rate.sleep()

        return interaction_detected

    def move_towards_hand(self, update=False):
        """
        Move the robot's end-effector towards the detected hand position.

        Parameters:
            update (bool): Whether to update the target based on the current hand position.
        """
        rospy.loginfo("Moving towards hand")

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

        if self.save_target is not None:
            target_position_arm = self.save_target
        else:
            target_position_arm = self.frame_transform(np.array(self.hand_pose))

        current_position = np.array(self.ee_pose[:3])
        position_threshold = 0.2
        self.saved_pose = np.array(self.hand_pose)
        update_pose = False

        while (np.linalg.norm(target_position_arm - current_position) > position_threshold) or wait_for_hand:
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
        interaction_detected = self.detect_human_interaction(duration=5.0)

        msg = Bool()
        if interaction_detected:
            rospy.loginfo("Human interaction is detected during waiting period.")
            msg.data = True
        else:
            rospy.loginfo("No human interaction detected during waiting period.")
            msg.data = False
        self.publish_human_input(msg)

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

    def test(self,experiment,positions = None):
        rot= np.deg2rad([107, -47, -11, 100, -82, -82, -35])
        count = 0

        initialise = True

        #random set of x,y,z coordinates from simulated camera:
        if positions == None:
            positions= np.array([[-0.6,0.5,2.1],
                                [-0.7,0.4,2.0],
                                [-0.6,0.3,2.1],
                                [-0.5,0.4,2.0]])
                           
        while not rospy.is_shutdown():
            if experiment == 0:
                rospy.loginfo("Sending pre-defined position command")
                self.send_position_command(rot,None)

                if self.delta > self.delta_save:
                    self.delta_save = self.delta

                rospy.loginfo(f"maximum delta:{self.delta_save:.2f}, current delta:{self.delta:.2f}")

                target = self.ee_pose
        
                # for some reason this axis is inverted
                target[4] = -target[4] 
            
                # Modify values to test if arm orientation changes
                target[0] += 0.1
                target[1] -= 0.1
                target[2] += 0.1

                rospy.loginfo("sending dynamic position command, forward")
                self.send_position_command(target,None)

                if self.delta > self.delta_save:
                    self.delta_save = self.delta

                rospy.loginfo(f"maximum delta:{self.delta_save:.2f}, current delta:{self.delta:.2f}")

                target = self.ee_pose
                target[4] = -target[4] 

                # Move back to original position
                target[0] -= 0.1
                target[1] += 0.1
                target[2] -= 0.1

                rospy.loginfo("sending dynamic position command, backwards")
                self.send_position_command(target,None)

                if self.delta > self.delta_save:
                    self.delta_save = self.delta

                rospy.loginfo(f"maximum delta:{self.delta_save:.2f}, current delta:{self.delta:.2f}")

    

            if experiment == 1:
                if initialise:
                    self.send_position_command(rot,None)
                    count = 0
                    orientation = self.ee_pose[3:6]
                    orientation[1] = -orientation[1] #BUG
                    full_pose = self.q
                    initialise = False
                    
                print("ee_pose at handover orientation:",orientation)

                xyz = self.frame_transform()

                target = np.hstack((xyz,orientation))

                print("target:",target)

                self.send_position_command(target,None)

                count+=1
                if count == 4:
                    count = 0

            if experiment == 2:
                if initialise:
                    self.send_position_command(rot,None)
                    initialise = False
                    saved_pose = self.ee_pose
                    saved_pose[4] = -saved_pose[4] 
                    initialise = False

                print("current pose:",self.ee_pose)

                print(f"switching to ee_control input:{saved_pose}")

                self.send_position_command(saved_pose,None)

            if experiment == 3:
                serve = np.deg2rad([107, -47, -11, 100, -82, -82, -35]) # Serve
                drop = np.deg2rad([55, -40, -8, 82, 5, 20, 0]) # Drop
                self.send_position_command(drop,None)
                #print("self.q for drop:",self.q)
                self.move_towards_hand()
                

                # self.send_position_command(serve,None)
                # print("self.q for serve:",self.q)
                # self.move_towards_hand()
               
                #self.move_towards_hand()


            if experiment == 4:
                if initialise:
                    self.send_position_command(rot,None)

                    fixed_orientation = self.ee_pose[3:]
                    fixed_orientation[1] = -fixed_orientation[1] #BUG
                    
                    initialise = False

                 
                target_positions= {0: [-0.32212266, -0.53631857,  0.42099988],
                                    1: [-0.36131503, -0.5264838,   0.49599993],
                                    2: [-0.33462706, -0.65330973,  0.26599991],
                                    3: [-0.49206611, -0.63994768,  0.14599978],
                                    4: [-0.28143831, -0.39959577,  0.33499991],
                                    5: [-0.14201932, -0.28211292,  0.1739999 ],
                                    6: [-0.21154168, -0.2970249,   0.08799993]}
                 
                goal = target_positions[count] + fixed_orientation

                print(target_positions[count])
                 
                self.send_position_command(goal,None,mode='ee_cartesian')

                count += 1

                if count == 7:
                    count = 0


if __name__=='__main__':
    rospy.init_node("test")
    controller = RoboticArmController()
    controller.send_position_command([0,0,0,0,0,0,0],None)
    #controller.test(0)
    #while True:
        #controller.send_position_command([0,0,0,0,0,0,0],None)
        #controller.send_position_command(np.deg2rad([107, -47, -11, 100, -82, -82, -35]),None)
    # controller.move_towards_hand(update=True)
    # while True:
    # #     controller.send_position_command(np.deg2rad([107, -47, -11, 100, -82, -82, -35]),None)
    #     controller.move_towards_hand()
    
    #controller.test(3)
    #controller.test(1)
    #print(controller.frame_transform(np.array([0.8,0.2,2.6])))