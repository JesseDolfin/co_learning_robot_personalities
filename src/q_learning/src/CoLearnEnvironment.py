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

        self.initialise_ros()

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

        self.old_phase_0 = 0
        self.old_phase_1 = 0

    def status_callback(self, msg):
        # Log update the variable that tells the system to proceed when True is received on the secondary task callback
        #rospy.loginfo("Draining_starts: %s, Draining_successfull: %s, Handover_successfull: %s", msg.draining_starts, msg.draining_successfull,msg.handover_successfull)
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
        if self.phase == self.phase_size-1:
            self.episode_length -= 1

        terminated = self.episode_length <= 0

        return self.state, self.phase, reward, terminated, self.info 

    def obtain_reward(self):
        if self.ros_running:
            reward = 0
            if self.old_phase < self.phase_size - 1: #at the end of the phases wait untill the handover has been successfull or not
                reward += 0
            else:
                if self.successfull_handover == 0:
                    self.rate.sleep()
                    self.obtain_reward()
                elif self.successfull_handover == 1:
                    reward += ( 10 + self.time_left )
                else:
                    reward -= 2
        else:
            reward = 0
            if self.phase == 0:
                self.old_phase_0 = self.state
            if self.phase == 1:
                self.old_phase_1 = self.state

            # #Trajectory 1: 0,0,1 = 10 R
            # if self.old_phase_0 == 0 and self.old_phase_1 == 0 and self.phase == 2 and self.state == 1:
            #     reward += 10
            # #Trajectory 2: 1,0,1 = 5 R
            # elif self.old_phase_0 == 1 and self.old_phase_1 == 0 and self.phase == 2 and self.state == 1:
            #     reward += 5
            # else:
            #     reward += 0

            if self.phase == 2 and self.state == 1:
                reward += 10

       
    
        return reward
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.episode_length = self.max_episode_length 
        return self.state, self.phase

    def close(self):
        pass



