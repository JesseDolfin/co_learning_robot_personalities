from email.policy import Policy
import gymnasium as gym
from gymnasium import Env
from gymnasium.spaces import Discrete
import numpy as np
import random

from regex import R
import rospy

import rosgraph

from co_learning_messages.msg import secondary_task_message

class CoLearn(Env):
    def __init__(self):
        '''
        Defines action space containing 6 actions (0-5)

        - Wait (reachable from each phase)
        - Initiate handover at time T_1 : 0, 0
        - Initiate handover at time T_2 : 0, 1
        - Go to Location A              : 1, 0 
        - Go to Location B              : 1, 1
        - Go to Serve                   : 2, 0
        - Go to Drop                    : 2, 1

        Where the first number indicates the phase

        
        Defines an observation space containing 9 states (0-8)

        - Handover_time             = {T_1,T,2},                phase = 0
        - State_position            = {Location_A,Location_B},  phase = 1
        - State_handOrientation     = {Serve,Drop},             phase = 2
        - State_space = 2 states per phase
        '''

        self.states={'Phase_0':['Wait','T1','T2'],
                     'Phase_1':['Wait','A','B'],
                     'Phase_2':['Wait','C','D']}
        
        self.action_size = 3 # per phase
        self.action_space = Discrete(self.action_size)
        
        self.observation_size = len(set(value for sublist in self.states.values() for value in sublist))
        self.observation_space = Discrete(self.observation_size)

        self.phase_size = len(self.states)
        self.phase = 0

        self.state_size = len(self.states['Phase_0']) # per phase
        self.state = random.randint(0, self.state_size-1) # initialize state here

        self.max_episode_length = 10
        self.episode_length = self.max_episode_length 

        self.info = {}

        self.initialise_ros()

    def status_callback(self, msg):
        self.successfull_handover = msg.handover_successfull
        self.time_left = msg.time_left

    def initialise_ros(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            rospy.Subscriber('Task_status',secondary_task_message,self.status_callback)
            self.rate=rospy.Rate(1)
            try:
                rospy.init_node('Environment', anonymous=True)
            except:
                rospy.logwarn("Cannot initialize node 'Environment' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline!")

        self.time_left = 0
        self.successfull_handover = 0

    def step(self, action=None):
        # Sample if no action is specified
        if action is None:
            action = self.action_space.sample()

        # Actions directly match states so direct update works (no unreachable states due to phase mechanism)
        self.state = action

        if self.phase < self.phase_size-1:
            self.phase += 1  
        else:
            self.phase = 0

        reward = self.obtain_reward()

        self.previous_state = self.state

        # Decrease remaining episode length at the end of all phases
        if self.phase == self.phase_size-1:
            self.episode_length -= 1

        terminated = self.episode_length <= 0

        return self.state, self.phase, reward, terminated, self.info 

    def obtain_reward(self):
        reward = 0
        if self.ros_running:
            if self.state != 0: # Incentive to not wait around
                reward += 1

            if self.phase == 2: # At the end of the phase check if handover succeeded
                if self.successfull_handover == 0:
                    self.rate.sleep()
                    self.obtain_reward()
                elif self.successfull_handover == 1:
                    reward += ( 10 + self.time_left ) 
                else:
                    reward -= 1 
            
        else:
            if self.state == 0:
                reward -= 1
            else:
                reward += 1

            if self.phase == 2 and self.previous_state == 1 and self.state == 2:
                reward += 10

        return reward
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.episode_length = self.max_episode_length 
        return self.state, self.phase

    def close(self):
        pass



