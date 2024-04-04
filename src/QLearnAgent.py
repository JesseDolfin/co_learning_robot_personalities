import gymnasium as gym
from gymnasium import Env
from gymnasium.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete

import os
import numpy as np
import random
from tqdm import tqdm
import matplotlib.pyplot as plt

import rospy

from CoLearnEnvironment import CoLearn

class QLearningAgent():
    def __init__(self,env):
        try:
            rospy.init_node('Q_agent', anonymous=True)
        except:
            rospy.logwarn("Cannot initialize node 'Q_agent' as it has already been initialized at the top level as a ROS node.")
        """
        Q-learning agent class, contains the functions:

            - train:            Used to train the agent in simulation mode (for setting up a Q-table)
            - evaluate:         Looks up the action in the Q-table to test the agents performance
            - train_real_time:  Unravels the training loop so that physical action commands can be 
                                send to the robot, allows for re-training of Q-table in real-time

        Args:
            action_size:    Sets the size of the action space
            state_size:     Sets the size of the sttate space
            env:            Specifies the environment the agent operates on
            phases:         Phases are used to reduce the size of the Q-table, see CoLearn() for more information
        """
        self.env = env
        self.q_table = np.random.rand(self.env.state_size, self.env.action_size,self.env.phase_size) * 0.01 # Random initiation of q-table
        
        self.terminated = False
        self.state,self.phase = self.env.reset()
        
    def train(self,n_steps,learning_rate=0.1,discount_factor=0.6,exploration_factor=0.1):
        # Hyperparameters
        alpha = learning_rate
        gamma = discount_factor
        epsilon = exploration_factor
        
        terminated = True

        for i in tqdm(range(1, n_steps+1)):
            state, phase = self.env.reset()
        
            epochs, reward, = 0, 0
            terminated = False
            
            while not terminated:
    
                if random.uniform(0, 1) < epsilon:
                    action = self.env.action_space.sample() # Explore action space
                else:
                    action = np.argmax(self.q_table[state, :, phase]) # Exploit learned values

                next_state, phase, reward, terminated, info = self.env.step(action) 

                old_value = self.q_table[state, action, phase]
                next_max = np.max(self.q_table[next_state, :, phase])
                
                new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
                self.q_table[state, action, phase] = new_value
        
                state = next_state
                epochs += 1
        
        print("Training finished.\n")

    def reset(self):
        self.terminated = False
        self.state, self.phase = self.env.reset()
        return self.state, self.phase, self.terminated
        

    def evaluate(self):
        """Evaluate agent's performance after Q-learning"""

        episodes = 10
        avg_reward = 0
        
        for _ in range(episodes):
            state, phase = self.env.reset()
            epochs, reward,tot_reward = 0, 0,0
            
            terminated = False
            
            while not terminated:
                if np.any(self.q_table):
                    action = np.argmax(self.q_table[state, :, phase])
                else:
                    action = None
              
                state, phase, reward, terminated, info = self.env.step(action)
                tot_reward += reward
                epochs += 1
            avg_reward += tot_reward
        avg_reward = avg_reward/episodes
        rospy.loginfo(f"average reward over {episodes} episodes is: {avg_reward}")

    def train_real_time(self,learning_rate=0.1,discount_factor=0.6,exploration_factor=0.8):
        # Hyperparameters
        alpha = learning_rate
        gamma = discount_factor
        epsilon = exploration_factor

        if random.uniform(0, 1) < epsilon:
            self.action = self.env.action_space.sample() # Explore action space
            #print(f"random action:{self.action}")
        else:
            self.action = np.argmax(self.q_table[self.state, :, self.phase]) # Exploit learned values

        next_state, self.phase, reward, self.terminated, info = self.env.step(self.action) 

        old_value = self.q_table[self.state, self.action, self.phase]
        next_max = np.max(self.q_table[next_state, :, self.phase])
        
        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        self.q_table[self.state, self.action, self.phase] = new_value

        self.state = next_state
            
        return self.action,self.phase,self.terminated
   
    def save_q_table(self, directory="code_jesse/Q_tables", prefix="q_table_"):
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

        rospy.loginfo(f"Saved Q-table as: {filepath}")
        
        
    def load_q_table(self,directory="/code_jesse/Q_tables/q_table_1"):
        try:
            self.q_table = np.load(directory)
            rospy.loginfo(f"Q_table loaded from directory: {directory}")
        except Exception as e:
            rospy.logwarn(e)
            rospy.logwarn("No Q_table loaded, agent operates on random initialisation of Q_table")
        
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


  