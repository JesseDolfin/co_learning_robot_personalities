#!/usr/bin/env python3

import sys
import csv  
import os

import webbrowser
import time
import signal
from typing import Literal
import subprocess

import gspread
from google.oauth2.service_account import Credentials
import pandas as pd

import numpy as np
import rospy
from std_msgs.msg import String
import rospkg

from co_learning_controllers.robot_controller import RobotArmController
from co_learning_controllers.hand_controller import SoftHandController
from co_learning_controllers.questionaire_controller import GoogleFormHandler
from co_learning_messages.msg import secondary_task_message, hand_pose
from q_learning.QLearnAgent import QLearningAgent
from q_learning.CoLearnEnvironment import CoLearn


HOME_POSITION = [np.pi / 2, np.pi / 4, 0, -np.pi / 4, 0, np.pi / 4, 0]
INTERMEDIATE_POSITION = [np.pi/2, 0, 0, 0, 0, 0, 0]


class RoboticArmControllerNode:
    def __init__(self,):
        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        self.fake = rospy.get_param('/fake', False)
        rospy.loginfo(f"Running in fake mode:{self.fake}")
        self.num_test_runs = rospy.get_param('/num_test_runs', 10)

        allowed_personality_types = {'baseline', 'leader', 'follower', 'impatient', 'patient'}
        self.type = rospy.get_param('/personality_type', 'baseline')
        if not self.fake and self.type not in allowed_personality_types:
            raise ValueError(f"Invalid personality type '{self.type}'. Allowed values are: {', '.join(allowed_personality_types)}")

        self.participant_number = rospy.get_param('/participant_number', 1)
        rospack = rospkg.RosPack()
        controller_path = rospack.get_path('co_learning_controllers')
        project_root = os.path.dirname(controller_path)
        self.base_dir = os.path.join(project_root, 'data_collection')
        print(self.base_dir)

        self.participant_dir = os.path.join(self.base_dir, f'participant_{self.participant_number}')
        self.personality_dir = os.path.join(self.participant_dir, f'personality_type_{self.type}')

        if not self.fake and os.path.exists(self.personality_dir):
            raise ValueError(f"Directory already exists: {self.personality_dir}")

        os.makedirs(self.personality_dir, exist_ok=True)

        # Set the exploration factor based on the personality type
        if self.type == 'leader':
            self.exploration_factor = 0.8
        elif self.type == 'follower':
            self.exploration_factor = 0.6
        else:
            self.exploration_factor = 0.25


        self.phase = 0
        self.terminated = False
        self.episode = 0
        self.task_status = 0
        self.run = True
        self.action = 0
        self.msg = secondary_task_message()
        self.draining_start = 0
        self.q = None
        self.hand_pose = [0, 0, 0]
        self.orientation = 'None'
        self.rosbag_process = None


        rospy.Subscriber('Task_status', secondary_task_message, self.status_callback)
        rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)

        self.pub = rospy.Publisher('Task_status', secondary_task_message, queue_size=1)

        self.env = CoLearn()
        if self.type == 'leader':
            self.env.type = 'leader'

        self.rl_agent = QLearningAgent(env=self.env)
        if self.type == 'follower':
            self.rl_agent.type = 'follower'

        self.hand_controller = SoftHandController(self.fake)
        self.robot_arm_controller = RobotArmController()

        self.alpha = 0.15
        self.gamma = 0.8
        self.Lamda = 0.3

        form_url = "https://forms.gle/xGV3pWaNoPVrXjHXA"
        sheet_url = "https://docs.google.com/spreadsheets/d/1iVvVxfakw5Un8Wk9xu2ObyB40vr6SW-ENc43ewN9g54/edit"
        key_path = "/home/worker-20/jesse/ws/psyched-loader-422713-u4-0fbb54ca49b0.json"

        self.form_handler = GoogleFormHandler(form_url, sheet_url, key_path)

        self.start_rosbag_recording()

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        rospy.signal_shutdown("Shutdown signal received.")
        self.stop_rosbag_recording()
        sys.exit(0)

    def status_callback(self, msg):
        self.msg = msg
        self.task_status = msg.handover_successful
        self.draining_start = msg.draining_starts
        self.draining_done = msg.draining_successful

    def hand_pose_callback(self, msg):
        self.hand_pose = [msg.x, msg.y, msg.z]
        self.orientation = msg.orientation

    def phase_0(self):
        """
        Go to the home position and grab the object
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: Home")
        self.robot_arm_controller.send_trajectory_goal(INTERMEDIATE_POSITION, 'joint')
        self.robot_arm_controller.send_trajectory_goal(HOME_POSITION, 'joint')
        self.hand_controller.send_goal('open', self.hand_time)
        time.sleep(2)
        self.hand_controller.send_goal('close', self.hand_time)

    def phase_1(self):
        """
        Wait until the human starts draining then; start handover directly (action 1),
        wait for the human to ask for item (action 2); break if human fails draining process.
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        rate = rospy.Rate(10)

        if self.action == 1:
            while self.draining_start == 0:
                self.msg.reset = True
                rate.sleep()
                if self.task_status == -1:
                    break
        if self.action == 2:
            while self.draining_start == 0:
                self.msg.reset = True
                rate.sleep()
                if self.task_status == -1:
                    break

            self.msg.reset = False
            self.original_orientation = self.orientation

            while (
                self.original_orientation == self.orientation
                and self.draining_done == 0
                and self.task_status != -1
            ):
                rate.sleep()

    def phase_2(self):
        """
        Go to intermediate position and then to either serve orientation (action 3)
        or drop orientation (action 4). Then move the item towards the hand of the human.
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        position = self.convert_action_to_orientation(self.action)
        self.robot_arm_controller.send_trajectory_goal(INTERMEDIATE_POSITION, 'joint')
        self.robot_arm_controller.send_trajectory_goal(position, 'joint')
        self.robot_arm_controller.move_towards_hand(update=True)

    def phase_3(self):
        """
        Decide to open or close the hand based on the action.
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        self.robot_arm_controller.move_towards_hand()

        if self.action == 5:
            self.hand_controller.send_goal('open', self.hand_time)
        elif self.action == 6:
            self.hand_controller.send_goal('partial', self.hand_time)
        elif self.action == 7:
            self.hand_controller.send_goal('close', self.hand_time)

    def update_q_table(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: 4, Action: Experience replay")
        self.rl_agent.experience_replay(self.alpha, self.gamma, self.Lamda)

    def check_end_condition(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: 4, Action: Resume_experiment = {self.num_test_runs > self.episode}")
        if self.num_test_runs > self.episode:
            self.episode += 1
            self.reset()
        else:
            _ = self.robot_arm_controller.send_trajectory_goal(INTERMEDIATE_POSITION, 'joint')
            self.run = False

            # Run the form workflow for the current personality type
            self.form_handler.run_workflow(personality_dir=self.personality_dir)



    def start_rosbag_recording(self):
        rospy.loginfo("Starting rosbag ...")

        topics_to_record = ['/Task_status', '/hand_pose','/CartesianImpedanceController/joint_states','/JointImpedanceController/joint_states']

        bag_files_dir = os.path.join(self.personality_dir, 'bag_files')
        os.makedirs(bag_files_dir, exist_ok=True)
        bag_filename = f'robot_state_episode_{self.episode}.bag'
        bag_filepath = os.path.join(bag_files_dir, bag_filename)

        # Construct the rosbag record command
        command = ['rosbag', 'record', '-O', bag_filepath] + topics_to_record
    
        self.rosbag_process = subprocess.Popen(command)
        time.sleep(2) # Allow rosbag to startup
 
    def stop_rosbag_recording(self):
        if self.rosbag_process:
            rospy.loginfo("Stopping rosbag ...")
            # Send SIGINT to rosbag process to ensure it writes the bag file correctly
            self.rosbag_process.send_signal(signal.SIGINT)
            self.rosbag_process.wait()
            self.rosbag_process = None

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Phase 0: Home
        - Phase 1: Decide on handover moment
        - Phase 2: Decide on a handover orientation & move end-effector to coordinates of human hand
        - Phase 3: Decide on when to open the hand
          After termination:
        - Update q-table with experience replay
        - Save all information related to the experiment
        - If n_run < runs: Phase_0 else: end
        """
        while self.run:
            if not self.terminated:
               
                if self.phase == 0:
                    self.phase_0()
                if self.phase == 1:
                    self.action = 2
                    self.phase_1()
                if self.phase == 2:
                    self.phase_2()
                if self.phase == 3:
                    self.phase_3()

                self.action, self.phase, self.terminated = self.rl_agent.train(
                    learning_rate=self.alpha,
                    discount_factor=self.gamma,
                    trace_decay=self.Lamda,
                    exploration_factor=self.exploration_factor,
                    real_time=True,
                )

                if self.task_status in [-1,1]:
                    self.terminated = True
                    self.stop_rosbag_recording()
                    self.update_q_table()
                    self.save_information()
                    self.check_end_condition()

                if self.terminated:
                    self.phase_3()

            else:
                self.stop_rosbag_recording()
                self.update_q_table()
                self.save_information()
                self.check_end_condition()

    def convert_action_to_orientation(self, action: int):
        positions = {
            3: np.deg2rad([107, -47, -11, 100, -82, -82, -35]).tolist(),  # Serve
            4: np.deg2rad([55, -40, -8, 82, 5, 50, 0]).tolist(),  # Drop
        }
        return positions.get(action, INTERMEDIATE_POSITION)
    
    def save_information(self):
        """Saves the Q-table and logs total_reward and task_status to a CSV file."""

        total_reward = self.rl_agent.total_reward
        task_status = self.task_status

        # Q_tables
        q_tables_dir = os.path.join(self.personality_dir, 'Q_tables')
        os.makedirs(q_tables_dir, exist_ok=True)

        q_table_filename = f'Q_table_{self.episode}.npy'
        q_table_filepath = os.path.join(q_tables_dir, q_table_filename)
        
        self.rl_agent.save_q_table(filepath=q_table_filepath)
        
        # Log files
        logs_dir = os.path.join(self.personality_dir, 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        
        log_filename = 'episode_logs.csv'
        log_filepath = os.path.join(logs_dir, log_filename)
        
        file_exists = os.path.isfile(log_filepath)
        
        # Open the file in append mode
        with open(log_filepath, 'a', newline='') as csvfile:
            fieldnames = ['episode', 'total_reward', 'task_status']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            if not file_exists:
                writer.writeheader()
            
            writer.writerow({'episode': self.episode, 
                            'total_reward': total_reward, 
                            'task_status': task_status})

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.reset_msg()
        self.robot_arm_controller.hand_pose = None
        self.rl_agent.print_q_table()
        self.start_rosbag_recording()

        if self.type == 'leader':
            self.exploration_factor = max(self.exploration_factor * 0.9, 0.20)
        elif self.type == 'follower':
            self.exploration_factor = max(self.exploration_factor * 0.80, 0.05)
        else:
            self.exploration_factor = max(self.exploration_factor * 0.95, 0.10)

    def reset_msg(self):
        """
        Signals the secondary task that it may also get ready for another attempt.
        """
        self.msg = secondary_task_message()
        self.msg.draining_starts = 0
        self.msg.draining_successful = 0
        self.msg.reset = False
        self.msg.phase = 0
        self.draining_start = 0
        self.orientation = 'None'
        self.original_orientation = None
        self.task_status = 0


if __name__ == '__main__':
    try:
        node = RoboticArmControllerNode()
        node.start_episode()
    except rospy.ROSInterruptException:
        pass