#!/usr/bin/env python3

from gymnasium import Env
from gymnasium.spaces import Discrete
import rospy
import rosgraph
from co_learning_messages.msg import secondary_task_message, hand_pose
from std_msgs.msg import Bool


class CoLearn(Env):
    """
    Defines an environment for reinforcement learning with the following:

    - **Action Space** (8 actions):
        - Move to home: 0
        - Initiate handover immediately: 1
        - Wait for state change: 2
        - Go to Serve: 3
        - Go to Drop: 4
        - Open hand: 5
        - Open hand partially: 6
        - Close hand: 7

    - **Observation Space** (17 states):
        - Home: 0
        - (Immediately, hand in workspace): 1
        - (Immediately, hand not in workspace): 2
        - (Wait for state change, hand in workspace): 3
        - (Wait for state change, hand not in workspace): 4
        - (Hand_h: serve, robot: serve): 5
        - (Hand_h: serve, robot: drop): 6
        - (Hand_h: drop, robot: serve): 7
        - (Hand_h: drop, robot: drop): 8
        - (Hand_h: unknown, robot: serve): 9
        - (Hand_h: unknown, robot: drop): 10
        - (Human input, hand open): 11
        - (Human input, hand partial): 12
        - (Human input, hand close): 13
        - (No human input, hand open): 14
        - (No human input, hand partial): 15
        - (No human input, hand close): 16
    """

    def __init__(self):
        self.action_size = 8
        self.action_space = Discrete(self.action_size)

        self.observation_size = 17
        self.observation_space = Discrete(self.observation_size)

        self.phase = 0
        self.phase_size = 4

        self.state_size = 17
        self.state = 0

        self.reward = 0

        self.max_episode_length = 100
        self.episode_length = self.max_episode_length

        self.info = {'valid': None, 'phase': 0}

        self.handover_successful = 0
        self.human_input = False
        self.orientation = 'None'

        self.terminated = False

        self.type = 'none'

        self.initialize_ros()

    def status_callback(self, msg):
        self.handover_successful = msg.handover_successful
        self.time_left = msg.time_left

    def hand_pose_callback(self, msg):
        self.orientation = msg.orientation

    def human_input_callback(self, msg):
        self.human_input = msg.data

    def initialize_ros(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('Environment', anonymous=True)
            except rospy.exceptions.ROSException:
                rospy.logwarn(
                    "Cannot initialize node 'Environment' as it has already been initialized."
                )
            rospy.Subscriber('Task_status', secondary_task_message, self.status_callback)
            rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)
            rospy.Subscriber('human_input', Bool, self.human_input_callback)
        else:
            print("ROS is offline! Environment proceeds in offline mode")

    def check_phase_transition(self, action):
        if self.phase == 0:
            return action in [1, 2]
        elif self.phase == 1:
            return action in [3, 4]
        elif self.phase == 2:
            return action in [5, 6, 7]
        elif self.phase == 3:
            return action == 5
        else:
            return False

    def update_phase(self):
        if self.state == 0:
            self.phase = 0
        elif self.state in [1, 2, 3, 4]:
            self.phase = 1
        elif self.state in [5, 6, 7, 8, 9, 10]:
            self.phase = 2
        elif self.state in [11, 12, 13, 14, 15, 16]:
            self.phase = 3
        else:
            rospy.logwarn(f"No valid phase found, phase is: {self.phase}")

    def update_state(self, action):
        if self.state == 0:
            if action == 1:
                return 2 if self.orientation == 'None' else 1
            elif action == 2:
                return 4 if self.orientation == 'None' else 2

        elif self.state in [1, 2, 3, 4]:
            if action == 3:
                if self.orientation == 'Serve':
                    return 5
                elif self.orientation == 'Drop':
                    return 7
                else:
                    return 9
            elif action == 4:
                if self.orientation == 'Serve':
                    return 6
                elif self.orientation == 'Drop':
                    return 8
                else:
                    return 10

        elif self.state in [5, 6, 7, 8, 9, 10]:
            if self.human_input:
                if action == 5:
                    return 11
                elif action == 6:
                    return 12
                elif action == 7:
                    return 13
            else:
                if action == 5:
                    return 14
                elif action == 6:
                    return 15
                elif action == 7:
                    return 16

        elif self.state in [11, 12, 13, 14, 15, 16]:
            if self.human_input:
                if action == 5:
                    return 11
                elif action == 6:
                    return 12
                elif action == 7:
                    return 13
            else:
                if action == 5:
                    return 14
                elif action == 6:
                    return 15
                elif action == 7:
                    return 16

        # If no valid transition is found, return the current state
        return self.state

    def step(self, action=None):
        action = self.action_space.sample() if action is None else action
        valid_transition = self.check_phase_transition(action)

        self.episode_length -= 1
        self.terminated = self.episode_length <= 0 or (action == 5 and self.phase == 3)

        self.previous_state = self.state
        self.state = self.update_state(action)

        self.previous_phase = self.phase
        self.update_phase()

        self.obtain_reward(action)

        self.info['valid'] = valid_transition
        self.info['phase'] = self.phase

        return self.state, self.reward, self.terminated, self.info

    def obtain_reward(self, action):
        self.reward = 0
        if self.ros_running:
            if self.previous_phase == 3 and action == 5:
                # This means that the hand is now open
                rate = rospy.Rate(5)
                while self.handover_successful == 0:
                    # Wait for confirmation that the handover was successful or failed
                    rate.sleep()
                    rospy.loginfo_once("Waiting for handover status")
                if self.handover_successful == 1:
                    # Reward for successful handover
                    self.reward += self.observation_size * 10
                    self.reward += self.observation_size * self.time_left * 2
                elif self.handover_successful == -1:
                    # Penalty for failed handover
                    self.reward -= self.observation_size * 10

            if self.type == 'leader' and self.state in [5, 7]:
                # Robot prefers the state 'serve'
                self.reward += 10

            if self.previous_state in [12, 15] and action in [6, 7]:
                # Incentive not to close the hand
                self.reward -= 2

            if self.previous_state in [13, 16] and action == 7:
                # Incentive not to keep the hand closed
                self.reward -= 2
        else:
            self.reward = 0  # Offline reward function can be implemented here

    def reset(self):
        self.state = 0
        self.phase = 0
        self.reward = 0
        self.episode_length = self.max_episode_length
        return self.state, self.phase

    def close(self):
        pass

