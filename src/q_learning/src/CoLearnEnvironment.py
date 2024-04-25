import gymnasium as gym
from gymnasium import Env
from gymnasium.spaces import Discrete
import numpy as np
import random

class CoLearn(Env):
    def __init__(self):
        '''
        Defines action space containing 6 actions (0-5)

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

        self.action_size = 2 # Per phase
        self.action_space = Discrete(self.action_size)
        self.observation_space = Discrete(6) # Same as state space

        self.state_size = 2 # Per phase
        self.state = random.randint(0, self.state_size - 1) # initialize state here

        self.phase_size = 3

        self.max_episode_length = 10
        self.episode_length = self.max_episode_length 
        self.info = {}
        
        self.phase = 0

    def step(self, action=None):
        # If no action is given, sample action space
        if action is None:
            action = self.action_space.sample()

        # Update the state based on the action taken
        self.state = action

        # Update the phase based on the previous phase
        if self.phase < self.phase_size-1:
            self.phase += 1  
        else:
            self.phase = 0

        # Obtain the reward for task completion
        reward = self.obtain_reward()

        # Decrease remaining episode length at the end of all phases
        if self.phase == self.phase_size:
            self.episode_length -= 1

        terminated = self.episode_length <= 0

        return self.state, self.phase, reward, terminated, self.info 

    def obtain_reward(self):
    
        reward_preference = 5 if self.phase == 2 and self.state == 0 else 1 if self.phase == 2 and self.state == 1 else 0
        reward = 16 if (self.phase == 0 and self.state == 0) or (self.phase == 0 and self.state == 1) else 0
     
        return reward_preference + reward
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.episode_length = self.max_episode_length 
        return self.state, self.phase

    def close(self):
        pass



