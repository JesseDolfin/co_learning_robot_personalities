#!/usr/bin/env python3

import os
import numpy as np
import random
from tqdm import tqdm
import rosgraph
import rospy
from q_learning.CoLearnEnvironment import CoLearn


class QLearningAgent:
    def __init__(self, env):
        self.env = env

        # Random initialization of Q-table
        self.q_table = np.random.rand(self.env.state_size, self.env.action_size) * 0.01

        self.reset_experience()
        self.state, self.phase = self.env.reset()
        self.e_trace = np.zeros((self.env.state_size, self.env.action_size))
        self.total_reward = 0

        self.initialize = True
        self.type = 'none'

        self.initialize_ros()

    def initialize_ros(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('Q_agent', anonymous=True)
            except rospy.exceptions.ROSException:
                rospy.logwarn(
                    "Cannot initialize node 'Q_agent' as it has already been initialized."
                )
        else:
            print("ROS is offline! Agent proceeds in offline mode")

    def train(
        self,
        n_steps=100000,
        learning_rate=0.8,
        discount_factor=0.8,
        exploration_factor=0.25,
        trace_decay=0.3,
        real_time=False,
    ):
        """
        Trains the Q-learning agent.

        Args:
            n_steps (int): Number of training steps.
            learning_rate (float): Learning rate for Q-learning.
            discount_factor (float): Discount factor for future rewards.
            exploration_factor (float): Exploration rate for epsilon-greedy policy.
            trace_decay (float): Decay rate for eligibility traces.
            real_time (bool): If True, train in real-time mode.
        """
        if self.type == 'follower':
            alpha = 0.5
        else:
            alpha = learning_rate

        gamma = discount_factor
        epsilon = exploration_factor
        lamda = trace_decay

        if real_time:
            if self.initialize:
                self.total_reward = 0
                self.state, self.phase = self.env.reset()
                self.reset_experience()
                terminated = False
                self.e_trace = np.zeros_like(self.q_table)
                self.initialize = False

            phase = self.phase
            valid = False
            while phase == self.phase and not valid:
                action = self.epsilon_greedy(epsilon)
                next_state, reward, terminated, info = self.env.step(action)

                phase = info.get('phase', phase)
                valid = info.get('valid', False)

                self.experience_update(
                    self.state, action, next_state, reward, valid
                )

                if reward != 0 and valid:
                    self.total_reward += reward
                    # To prevent the agent from getting stuck in a loop
                    self.update_q_table(
                        self.state, action, reward, next_state, alpha, gamma, lamda
                    )

                self.state = next_state

            self.phase = phase

            if terminated:
                self.initialize = True

            return action, self.phase, terminated

        else:
            for _ in tqdm(range(1, n_steps + 1)):
                self.state, self.phase = self.env.reset()
                self.reset_experience()
                terminated = False
                self.e_trace = np.zeros_like(self.q_table)

                while not terminated:
                    action = self.epsilon_greedy(epsilon)
                    # Perform a step
                    next_state, reward, terminated, info = self.env.step(action)
                    valid = info.get('valid', False)
                    self.phase = info.get('phase', self.phase)
                    self.experience_update(
                        self.state, action, next_state, reward, valid
                    )
                    self.state = next_state

                self.experience_replay(alpha, gamma, lamda)

    def epsilon_greedy(self, epsilon):
        """Selects an action using epsilon-greedy policy."""
        if random.uniform(0, 1) < epsilon:
            action = self.env.action_space.sample()  # Explore action space
        else:
            action = np.argmax(self.q_table[self.state, :])  # Exploit learned values
        return action

    def experience_update(self, state, action, next_state, reward, valid):
        """Updates the experience buffer."""
        self.experience["state"].append(state)
        self.experience["action"].append(action)
        self.experience["next_state"].append(next_state)
        self.experience["reward"].append(reward)
        self.experience["valid"].append(valid)

    def experience_replay(self, alpha, gamma, lamda):
        """Updates the Q-table using experience replay."""
        if self.ros_running:
            rospy.loginfo("Updating Q-table with experience")

        last_reward = self.experience["reward"][-1]
        for i in range(len(self.experience["state"])):
            state = self.experience["state"][i]
            action = self.experience["action"][i]
            next_state = self.experience["next_state"][i]
            valid = self.experience["valid"][i]
            if valid:
                # Increase the reward for this transition by the final reward per phase
                reward = last_reward / self.env.phase_size
                self.total_reward  += reward
                self.update_q_table(state, action, reward, next_state, alpha, gamma, lamda)

    def reset_experience(self):
        """Resets the experience buffer."""
        self.experience = {
            "state": [],
            "action": [],
            "next_state": [],
            "reward": [],
            "valid": [],
        }

    def reset(self):
        """Resets the agent's state."""
        self.reset_experience()
        self.state, self.phase = self.env.reset()
        return self.state, self.phase

    def update_q_table(self, state, action, reward, next_state, alpha, gamma, lamda):
        """Updates the Q-table using the Q-learning update rule with eligibility traces."""
        old_value = self.q_table[state, action]
        next_max = np.max(self.q_table[next_state, :])
        td_error = reward + gamma * next_max - old_value

        self.e_trace[state, action] += 1

        self.q_table += alpha * td_error * self.e_trace

        self.e_trace *= gamma * lamda

    def save_q_table(self, filepath=None):
        """Saves the Q-table to a file."""
        if filepath is None:
            directory = "co_learning_robot_personalities/src/q_learning/Q_tables"
            os.makedirs(directory, exist_ok=True)

            index = 1
            while True:
                filename = f"q_table_{index}.npy"
                filepath = os.path.join(directory, filename)
                if not os.path.exists(filepath):
                    break
                index += 1
        else:
            directory = os.path.dirname(filepath)
            os.makedirs(directory, exist_ok=True)

        np.save(filepath, self.q_table)

        if self.ros_running:
            rospy.loginfo(f"Saved Q-table as: {filepath}")
        else:
            print(f"Saved Q-table as: {filepath}")

    def load_q_table(
        self, filepath="co_learning_robot_personalities/src/q_learning/Q_tables/q_table.npy"
    ):
        """Loads the Q-table from a file."""
        try:
            self.q_table = np.load(filepath)
            if self.ros_running:
                rospy.loginfo(f"Q-table loaded from: {filepath}")
            else:
                print(f"Q-table loaded from: {filepath}")
        except Exception as e:
            if self.ros_running:
                rospy.logwarn(f"Failed to load Q-table: {e}")
            else:
                print(f"Failed to load Q-table: {e}")

    def print_q_table(self, margin=12):
        """Prints the Q-table."""
        header = "".join(
            [f"{'Action ' + str(i):<{margin}}" for i in range(self.env.action_size)]
        )
        print(f"{'':<{margin}}" + header)
        for state in range(self.env.state_size):
            row = f"State {state:<{margin - 6}}"
            row += "".join(
                [
                    f"{self.q_table[state, action]:<{margin}.2f}"
                    for action in range(self.env.action_size)
                ]
            )
            print(row)


if __name__ == '__main__':
    try:
        n_steps = 10000
        agent = QLearningAgent(env=CoLearn())
        agent.train(n_steps=n_steps)
        agent.save_q_table(prefix=f"q_table_solved_{n_steps}_")
    except Exception as e:
        print(e)
