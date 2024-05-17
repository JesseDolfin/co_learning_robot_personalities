from email.policy import Policy
import gymnasium as gym
from gymnasium import Env
from gymnasium.spaces import Discrete
import numpy as np
import random
import threading

from regex import R
import rospy

import rosgraph

from co_learning_messages.msg import secondary_task_message

class CoLearn(Env):
    def __init__(self):
        '''
        Defines action space containing 8 actions (0-7)

        - Home
        - Initiate handover at time T_1 : 0, 0
        - Initiate handover at time T_2 : 0, 1
        - Go to Location A              : 1, 0 
        - Go to Location B              : 1, 1
        - Go to Serve                   : 2, 0
        - Go to Drop                    : 2, 1
        - Open hand

        Where the first number indicates the phase

        
        Defines an observation space containing 8 states (0-7)
        - Home                      = Home                      phase = 0
        - Handover_time             = {T_1,T,2},                phase = 1
        - State_position            = {Location_A,Location_B},  phase = 2
        - State_handOrientation     = {Serve,Drop},             phase = 3
        - Open hand                 = Open_hand                 phase = 4
        '''

        
        self.action_size = 8
        self.action_space = Discrete(self.action_size)
        
        self.observation_size = 8
        self.observation_space = Discrete(self.observation_size)

        self.phase_size = 5
        self.phase = 0

        self.state_size = 8
        self.state = 0 

        self.max_episode_length = 100
        self.episode_length = self.max_episode_length 

        self.info = {'valid':None,
                     'phase':0}
        
        self.phase_0_state = 0
        self.phase_1_state = 0
        self.phase_2_state = 0
        self.phase_3_state = 0

        self.condition = threading.Condition()
        self.relevant_message = None

        self.initialise_ros()

    def status_callback(self, msg):
        with self.condition:
            if msg.handover_successfull != 0:  # Check if handover_successfull is set to either -1 or 1
                self.relevant_part = {
                    'handover_successfull': msg.handover_successfull,
                    'time_left': msg.time_left
                }
                self.condition.notify()

   

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
            print("ROS is offline! Environment proceeds in offline mode")

        self.relevant_part = {'handover_successfull': 0, 'time_left': 0}

    def check_valid_action(self,action):
        if self.state == 0:                             # 0
            valid = True if action in [1,2] else False
        elif self.state in [1,2]:                       # 1
            valid = True if action in [3,4] else False
        elif self.state in [3,4]:                       # 2
            valid = True if action in [5,6] else False
        elif self.state in [5,6]:                       # 3
            valid = True if action == 7 else False
        elif self.state == 7:                           # 4
            valid = True if action == 0 else False
        else:
            valid = False
        return valid

    def step(self, action=None):
        # Sample if no action is specified
        if action is None:
            action = self.action_space.sample()

        self.action = action

        self.valid_transition = self.check_valid_action(action)

        self.previous_state = self.state

        if self.phase == 3:
            self.handover_event.clear()

        if self.valid_transition:
            self.state = action
            if self.previous_state == 7 and self.state ==0:
                self.phase = 0
            else:
                self.phase += 1
            self.save_previous_state(self.phase,self.state)

        reward = self.obtain_reward()

        self.info['valid'] = self.valid_transition
        self.info['phase'] = self.phase

        self.episode_length -= 1

        terminated = self.episode_length <= 0 or (self.previous_state == 7 and self.state == 0)

        return self.state, reward, terminated, self.info 

    def obtain_reward(self):
        reward = 0
        if self.ros_running:
            if not self.valid_transition: # Only valid transitions are rewarded
                reward = -10
            else:
                reward += 10

            if self.phase == 3: 
                with self.condition:
                    while self.relevant_part is None:
                        self.condition.wait()  # Ensures we obtain the reward only as soon as the handover is successful (or failed)
                    if self.relevant_part['handover_successfull'] == 1:
                        reward += self.relevant_part['time_left']
                    else:
                        reward = 0  # -5
            
        else:
            if not self.valid_transition:
                reward = -5
            else:
                reward += 5

            # if self.previous_state == self.action:
            #     reward -= 1

            if self.phase_1_state == 1 and self.phase_2_state == 3 and self.phase_3_state == 6:
                reward += 1

            if self.phase_1_state == 2 and self.phase_2_state == 4 and self.phase_3_state == 6:
                reward += 3

        return reward
    
    def save_previous_state(self,phase,state):
        if phase == 0:
            self.phase_0_state = state
        if phase == 1:
            self.phase_1_state = state
        if phase == 2:
            self.phase_2_state = state
        if phase == 3:
            self.phase_3_state = state
        
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.episode_length = self.max_episode_length 
        return self.state, self.phase

    def close(self):
        pass



