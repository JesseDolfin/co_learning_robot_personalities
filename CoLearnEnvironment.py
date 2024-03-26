import gymnasium as gym
from gymnasium import Env
from gymnasium.spaces import Discrete
import numpy as np
import random

class CoLearn(Env):
    def __init__(self):
        '''
        Defines action space containing 6 actions (0-5)

        - Go to Location A    : 0, 0 
        - Go to Location B    : 0, 1
        - Go to Location C    : 0, 2
        - Go to Serve         : 1, 0
        - Go to Drop          : 1, 1
        - Go to Place         : 1, 2

        Where the first number indicates the phase

        
        Defines an observation space containing 9 states (0-8)

        - State_position          = {Location_A,Location_B,Location_C} , phase = 0
        - State_handOrientation   = {Serve,Drop,Place} , phase = 1
        - State_space = 3 per phase
        '''
        self.action_size = 3
        self.action_space = Discrete(self.action_size)
        self.observation_space = Discrete(9)  # Updated observation space

        self.state_size = 3
        self.state = random.randint(0, self.state_size - 1) # initialize state here

        self.phase_size = 2

        self.max_episode_length = 10
        self.episode_length = self.max_episode_length 
        self.info = {}
        

        # Phase is used to break up action states into two distinct categories
        # Phase 1: move to position A or B or C, Phase 2: Move hand to orientation Serve, Drop, Place
        self.phase = 0

    def step(self, action=None):
        # If no action is given, sample action space
        if action is None:
            action = self.action_space.sample()

        # Lookup the next state based on the action taken
        next_state = action
        if self.phase < self.phase_size-1: # When max phase is reached reset phase count
            self.phase += 1  
        else:
            self.phase = 0
        self.state = next_state

        # Obtain the reward for task completion
        reward = self.obtain_reward()

        # Decrease remaining episode length  
        self.episode_length -= 1

        terminated = self.episode_length <= 0

        return self.state, self.phase, reward, terminated, self.info 

    def obtain_reward(self):
        # Agent will get rewarded more if the state transitions to location C in phase 1
        # and to location B in phase 2, so it has a 'preference' for these states
        reward_preference = 0 #10 if (self.phase == 0 and self.state == 0) or (self.phase == 1 and self.state == 1) else 0

        # Simulate a person who prefers everything but position C and the item placed on them (state 2)
        if (self.phase == 0 and self.state == 2) or (self.phase == 1 and self.state == 2):
            reward = 16
        else:
            reward = 0
        return reward_preference + reward
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.episode_length = self.max_episode_length 
        return self.state, self.phase

    def close(self):
        pass



