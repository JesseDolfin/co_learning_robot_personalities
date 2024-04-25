#!/usr/bin/env python3

import os
import numpy as np
import random
from tqdm import tqdm
import matplotlib.pyplot as plt

import rosgraph
import rospy

from CoLearnEnvironment import CoLearn

class QLearningAgent():
    def __init__(self,env):
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
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('Q_agent', anonymous=True)
            except:
                rospy.logwarn("Cannot initialize node 'Q_agent' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline!")



        self.env = env
        self.q_table = np.random.rand(self.env.state_size, self.env.action_size,self.env.phase_size) * 0.01 # Random initiation of q-table
        self.eligibility_trace = np.zeros((self.env.state_size, self.env.action_size, self.env.phase_size))
        self.action_phase_space = np.ones((self.env.action_size,self.env.phase_size))
        self.sampled_actions = [list(range(self.env.action_size)) for _ in range(self.env.phase_size)]

        
        self.terminated = False
        self.state,self.phase = self.env.reset()
        
    def train(self,n_steps,learning_rate=0.1,discount_factor=0.6,exploration_factor=0.1,trace_decay=0.9,replacement = True):
        # Hyperparameters
        alpha = learning_rate
        gamma = discount_factor
        epsilon = exploration_factor
        Lambda = trace_decay
        replacement = replacement
        
        terminated = True

        for _ in tqdm(range(1, n_steps+1)):
            state, phase = self.env.reset()
        
            reward = 0
            terminated = False
            
            while not terminated:
                # Get next action
                action = self.next_action(epsilon,replacement,state,phase)

                # Perform a step
                next_state, phase, reward, terminated, info = self.env.step(action) 
              
                # Apply the update rule to the q_table
                self.update_Q_table(next_state,reward,gamma,Lambda,alpha,action,phase,state)
        
                state = next_state
        
        #print("Training finished.\n")
  
    def train_real_time(self,learning_rate=0.8,discount_factor=0.6,exploration_factor=0.8,trace_decay = 0.6,replacement = True):
        """
        Unravels the training loop and saves the phase, action, state, and terminated values. 
        """
        # Hyperparameters
        alpha = learning_rate
        gamma = discount_factor
        epsilon = exploration_factor
        Lambda = trace_decay
        replacement = replacement

        current_phase = self.phase

        # Chose next action
        self.action = self.next_action(epsilon,replacement,self.state,self.phase)

        # Perform a step
        next_state, next_phase, reward, self.terminated, info = self.env.step(self.action) 

        # Apply the update rule to the q_table
        self.update_Q_table(next_state,reward,gamma,Lambda,alpha,self.action,next_phase,self.state)

        self.state = next_state
        
        self.phase = next_phase
            
        return self.action,current_phase,self.terminated
    
    def reset(self):
        self.terminated = False
        self.eligibility_trace = np.zeros((self.env.state_size, self.env.action_size, self.env.phase_size))
        self.state, self.phase = self.env.reset()
        return self.state, self.phase, self.terminated
    
    def update_Q_table(self,next_state,reward,gamma,Lambda,alpha,action,phase,state):
        # Calculate TD error
        old_value = self.q_table[state, action, phase]
        next_max = np.max(self.q_table[next_state, :, phase])
        delta = reward + gamma * next_max - old_value

        # Update eligibility trace
        self.eligibility_trace *= gamma * Lambda
        self.eligibility_trace[state, action, phase] += 1.0

        # Update Q-values using eligibility traces
        self.q_table += alpha * delta * self.eligibility_trace
    
    def next_action(self,epsilon,replacement,state,phase):
        # With replacement
        if replacement:
            if random.uniform(0, 1) < epsilon:
                action = self.env.action_space.sample() # Explore action space
            else:
                action = np.argmax(self.q_table[state, :, phase]) # Exploit learned values

        # Without replacement
        else:       
            if random.uniform(0, 1) < epsilon:
                if len(self.sampled_actions[phase]) == 0:
                    self.sampled_actions[phase] = list(range(self.env.action_size))
                
                action = random.choice(self.sampled_actions[phase])  
                self.sampled_actions[phase].remove(action)  
            else:
                action = np.argmax(self.q_table[state, :, phase])  

        return action
    
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
        
        
    def load_q_table(self,directory="co_learning_robot_personalities/src/q_learning/Q_tables/q_table_solved_1000000_1"):
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
        Agent.train(n_steps=n_steps,discount_factor=0.8)
        Agent.save_q_table(prefix=f"q_table_solved_{n_steps}_")
        #Agent.load_q_table(directory="code_jesse/Q_tables/q_table_5.npy")
        #Agent.evaluate()
    except Exception as e:
        print(e)


