#!/usr/bin/env python3

from ast import Lambda
import os
import numpy as np
import random
from tqdm import tqdm
import matplotlib.pyplot as plt

import rosgraph
import rospy

from CoLearnEnvironment import CoLearn

from co_learning_messages.msg import secondary_task_message

class QLearningAgent():
    def __init__(self, env):
        """
        Q-learning agent class, contains the functions:

            - train:            Used to train the agent in simulation mode (for setting up a Q-table)
            - evaluate:         Looks up the action in the Q-table to test the agent's performance
            - train_real_time:  Unravels the training loop so that physical action commands can be 
                                sent to the robot, allows for re-training of Q-table in real-time

        Args:
            action_size:    Sets the size of the action space
            state_size:     Sets the size of the state space
            env:            Specifies the environment the agent operates on
            phases:         Phases are used to reduce the size of the Q-table, see CoLearn() for more information
        """
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('Q_agent', anonymous=True)
            except:
                rospy.logwarn("Cannot initialize node 'Q_agent' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline! Agent proceeds in offline mode")

        self.env = env
        self.q_table = np.random.rand(self.env.observation_size, self.env.action_size) * 0.01  # Random initialization of Q-table
        self.reset_experience()
        self.state, self.phase = self.env.reset()
        self.e_trace = np.zeros((self.env.observation_size, self.env.action_size))
        self.initialise = True
        self.alpha = 0.15
        self.gamma = 0.8 
        self.Lambda = 0.3

    def train(self, n_steps = 100000, learning_rate=0.15, discount_factor=0.8, exploration_factor=0.25, trace_decay=0.3,real_time=False):
        # Hyperparameters
        alpha = learning_rate
        gamma = discount_factor
        epsilon = exploration_factor
        Lambda = trace_decay

        if real_time:
            if self.phase == 0 and self.initialise:
                self.state, self.phase = self.env.reset()
                self.reset_experience()
                terminated = False
                self.e_trace = np.zeros_like(self.q_table)
                self.initialise = False

            phase = self.phase
            while phase == self.phase:
                action = self.epsilon_greedy(epsilon)
                next_state, reward, terminated, info = self.env.step(action)
                phase = info.get('phase', phase)

                self.experience_update(self.state,action,next_state,reward)
                self.state = next_state

            self.phase = phase

            if terminated:
                self.initialise = True

            return action, self.phase, terminated

        else:
            for i in tqdm(range(1, n_steps + 1)):
                self.state, self.phase = self.env.reset()
                self.reset_experience()
                terminated = False
                self.e_trace = np.zeros_like(self.q_table)

                while not terminated:
                    action = self.epsilon_greedy(epsilon)

                    # Perform a step
                    next_state, reward, terminated, info = self.env.step(action)
                    self.experience_update(self.state, action, next_state, reward)
                    self.state = next_state
                    self.phase = info.get('phase', self.phase)

                if i > 1:
                    self.experience_replay(alpha, gamma, Lambda)

    def epsilon_greedy(self,epsilon):
        # Exploration-exploitation trade-off
        if random.uniform(0, 1) < epsilon:
            action = self.env.action_space.sample()  # Explore action space
        else:
            action = np.argmax(self.q_table[self.state, :])  # Exploit learned values

        return action

    def experience_update(self, state, action, next_state, reward):
        self.experience["state"].append(state)
        self.experience["action"].append(action)
        self.experience["next_state"].append(next_state)
        self.experience["reward"].append(reward)

    def experience_replay(self, alpha, gamma, Lambda):
        for i in range(len(self.experience["state"])):
            state = self.experience["state"][i]
            action = self.experience["action"][i]
            next_state = self.experience["next_state"][i]
            reward = self.experience["reward"][i]

            self.update_q_table(state, action, reward, next_state, alpha, gamma, Lambda)

    def reset_experience(self):
        self.experience = {"state": [],
                           "action": [],
                           "next_state": [],
                           "reward": []}

    def reset(self):
        self.reset_experience()
        self.state, self.phase = self.env.reset()
        return self.state, self.phase

    def update_q_table(self, state, action, reward, next_state, alpha, gamma, Lambda):
        old_value = self.q_table[state, action]
        next_max = np.max(self.q_table[next_state, :])
        td_error = reward + gamma * next_max - old_value

        self.e_trace[state, action] += 1

        self.q_table += alpha * td_error * self.e_trace

        self.e_trace *= gamma * Lambda

    
    def save_q_table(self, directory="co_learning_robot_personalities/src/q_learning/Q_tables", prefix="q_table_"):
        # Create the directory if it doesn't exist
        os.makedirs(directory, exist_ok=True)

        # Find the next available filename
        index = 1
        while True:
            filename = f"{prefix}{index}.npy"
            filepath = os.path.join(directory, filename)
            if not os.path.exists(filepath):
                break
            index += 1

        # Save the data to the file
        with open(filepath, "wb") as file:
            np.save(file, self.q_table)

        if self.ros_running:
            rospy.loginfo(f"Saved Q-table as: {filepath}")
        else:
            print(f"Saved Q-table as: {filepath}")

    def load_q_table(self, directory="co_learning_robot_personalities/src/q_learning/Q_tables/q_table_solved_1000000_1"):
        try:
            self.q_table = np.load(directory)
            rospy.loginfo(f"Q_table loaded from directory: {directory}")
        except Exception as e:
            rospy.logwarn(e)
        
if __name__ == '__main__':
    try:
        n_steps = 100000
        Agent = QLearningAgent(env=CoLearn())
        Agent.train(n_steps=n_steps)
        Agent.save_q_table(prefix=f"q_table_solved_{n_steps}_")
        #Agent.load_q_table(directory="code_jesse/Q_tables/q_table_5.npy")
        #Agent.evaluate()
    except Exception as e:
        print(e)


