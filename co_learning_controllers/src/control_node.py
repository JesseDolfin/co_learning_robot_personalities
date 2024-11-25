#!/usr/bin/env python3

import sys
import os

import argparse
import time
import random
import signal
from typing import Literal

import numpy as np
import rospy
from std_msgs.msg import String

from co_learning_messages.msg import secondary_task_message, hand_pose
from co_learning_controllers.src.hand_controller import SoftHandController
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn
from co_learning_controllers.src.robot_controller import RoboticArmController



HOME_POSITION = [np.pi / 2, np.pi / 4, 0, -np.pi / 4, 0, np.pi / 4, 0]
INTERMEDIATE_POSITION = [np.pi / 2, 0, 0, 0, 0, 0, 0]


class RoboticArmControllerNode:
    def __init__(
        self,
        num_test_runs: int,
        exploration_factor: float = 0.9,
        personality_type: Literal[
            'leader', 'follower', 'cautious', 'impatient', 'baseline'
        ] = 'baseline',
        fake=False,
    ):
        self.num_test_runs = num_test_runs

        if personality_type == 'leader':
            self.exploration_factor = 0.8
        elif personality_type == 'follower':
            self.exploration_factor = 0.6
        else:
            self.exploration_factor = exploration_factor

        self.phase = 0
        self.terminated = False
        self.episode = 0
        self.successful_handover = 0
        self.run = True
        self.action = 0
        self.msg = secondary_task_message()
        self.draining_start = 0
        self.q = None
        self.hand_pose = [0, 0, 0]
        self.orientation = 'None'
        self.type = personality_type

        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        rospy.Subscriber('Task_status', secondary_task_message, self.status_callback)
        rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)

        self.pub = rospy.Publisher('Task_status', secondary_task_message, queue_size=1)

        self.env = CoLearn()
        if personality_type == 'leader':
            self.env.type = 'leader'

        self.rl_agent = QLearningAgent(env=self.env)
        if personality_type == 'follower':
            self.rl_agent.type = 'follower'

        self.hand_controller = SoftHandController(fake)
        self.robot_arm_controller = RoboticArmController()
        if personality_type == 'impatient':
            self.robot_arm_controller.type = 'fast'
            self.hand_time = 1
        elif personality_type == 'cautious':
            self.robot_arm_controller.type = 'slow'
            self.hand_time = 3
        else:
            self.hand_time = 2

        self.alpha = 0.15
        self.gamma = 0.8
        self.Lamda = 0.3

        self.messages = {
            'impatient': [
                'Please hurry up!',
                'We need to move faster!',
                'Speed it up, please!',
            ],
            'leader': [
                'Good job, please present your arm to me when you are ready',
                'Looking forward to your next move!',
                'Excellent work, keep it up!',
            ],
            'cautious': [
                'Take your time!',
                'No rush, proceed at your pace',
                'Make sure everything is ready before proceeding',
            ],
            'follower': [
                'What should we do now?',
                'I am ready to follow your lead',
                'Let me know the next step',
            ],
            'baseline': [''],
        }

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        rospy.signal_shutdown("Shutdown signal received.")
        sys.exit(0)

    def status_callback(self, msg):
        self.msg = msg
        self.successful_handover = msg.handover_successful
        self.draining_start = msg.draining_starts
        self.draining_done = msg.draining_successful

    def hand_pose_callback(self, msg):
        self.hand_pose = [msg.x, msg.y, msg.z]
        self.orientation = msg.orientation

    def get_random_message(self, personality_type):
        if personality_type in self.messages:
            messages = self.messages[personality_type]
            if random.random() < 0.5:
                return random.choice(messages)
        return None

    def phase_0(self):
        """
        Go to the home position and grab the object
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: Home")
        _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION, None)
        _ = self.robot_arm_controller.send_position_command(HOME_POSITION, None)
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
                self.send_message()
                rate.sleep()
                if self.successful_handover == -1:
                    break
        if self.action == 2:
            while self.draining_start == 0:
                self.msg.reset = True
                self.send_message()
                rate.sleep()
                if self.successful_handover == -1:
                    break

            self.msg.reset = False
            self.send_message()
            self.original_orientation = self.orientation

            first_message = True
            while (
                self.original_orientation == self.orientation
                and self.draining_done == 0
                and self.successful_handover != -1
            ):
                if first_message:
                    first_message = False
                    message_text = self.get_random_message(self.type)
                    if message_text:
                        message = String()
                        message.data = message_text
                rate.sleep()

    def phase_2(self):
        """
        Go to intermediate position and then to either serve orientation (action 3)
        or drop orientation (action 4). Then move the item towards the hand of the human.
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        position = self.convert_action_to_orientation(self.action)
        _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION, None)
        _ = self.robot_arm_controller.send_position_command(position, None)
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
        """
        Checks if the terminal condition is reached.
        Sends a message at the end to embed more of the personality.
        """
        rospy.loginfo(f"Episode: {self.episode}, Phase: 4, Action: Resume_experiment = {self.num_test_runs > self.episode}")
        if self.num_test_runs > self.episode:
            self.episode += 1
            self.reset()
            message_text = self.get_random_message(self.type)
            if message_text:
                message = String()
                message.data = message_text
        else:
            _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION, None)
            self.run = False

    def send_message(self, phase=None):
        if self.msg is not None:
            msg = self.msg
        else:
            msg = secondary_task_message()
        if phase is not None:
            msg.phase = phase
        self.pub.publish(msg)

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Phase 0: Home
        - Phase 1: Decide on handover moment
        - Phase 2: Decide on a handover orientation & move end-effector to coordinates of human hand
        - Phase 3: Decide on when to open the hand
          After termination:
        - Update q-table with experience replay
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

                if self.terminated:
                    self.phase_3()

            else:
                self.update_q_table()
                self.check_end_condition()

    def convert_action_to_orientation(self, action: int):
        positions = {
            3: np.deg2rad([107, -47, -11, 100, -82, -82, -35]),  # Serve
            4: np.deg2rad([55, -40, -8, 82, 5, 50, 0]),  # Drop
        }
        return positions.get(action, INTERMEDIATE_POSITION)

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.reset_msg()
        self.robot_arm_controller.hand_pose = None
        self.rl_agent.print_q_table()

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
        self.successful_handover = 0
        self.send_message()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description="Robotic Arm Controller Node")
        parser.add_argument('--fake', action='store_true', help="Run in fake mode")
        args = parser.parse_args()
        fake = args.fake

        node = RoboticArmControllerNode(
            num_test_runs=10, exploration_factor=0.25, personality_type='follower', fake=fake
        )
        node.start_episode()

    except rospy.ROSInterruptException:
        pass
