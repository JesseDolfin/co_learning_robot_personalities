#!/usr/bin/env python3
from gymnasium import Env
from gymnasium.spaces import Discrete
import threading
import rospy
import rosgraph
from co_learning_messages.msg import secondary_task_message,hand_pose
from std_msgs.msg import Bool
import time

class CoLearn(Env):
    def __init__(self):
        '''
        Defines action space containing 8 actions (0-7)
        - Move to home                      : 0, 0
        - Initiate handover immediately     : 1, 1
        - Wait for state_change             : 1, 2
        - Go to Serve                       : 2, 3
        - Go to Drop                        : 2, 4
        - Open hand                         : 3, 5
        - Open hand partially               : 3, 6
        - Close hand                        : 3, 7
        Where the first number indicates the phase

        
        Defines an observation space containing 17 states (0-16)
        - Home                                          : 0, 0
        - (Immediately, hand in workspace)              : 1, 1
        - (Immediately, hand not in workspace)          : 1, 2
        - (wait for state change, hand in workspace)    : 1, 3
        - (wait for state change, hand not in workspace): 1, 4
        - (Hand_h: serve, robot: serve)                 : 2, 5
        - (Hand_h: serve, robot: drop)                  : 2, 6
        - (Hand_h: drop, robot: serve)                  : 2, 7
        - (hand_h: drop, robot: drop)                   : 2, 8
        - (Hand_h: unknown, robot: serve)               : 2, 9
        - (Hand_h: unknown, robot: drop)                : 2, 10
        - (Human_input, Hand open)                      : 3, 11
        - (Human_input, Hand partial)                   : 3, 12
        - (Human_input, Hand close)                     : 3, 13
        - (No human_input, Hand open)                   : 3, 14
        - (No human_input, Hand partial)                : 3, 15
        - (No human_input, Hand close)                  : 3, 16
        '''
 
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

        self.info = {'valid':None,
                     'phase':0,
                     'break':False}
        
     
        self.condition = threading.Condition()
        self.relevant_message = None

        self.handover_successful = 0
        self.human_input = False
        self.orientation = 'None'

        self.terminated = False

        self.hand_open = False

        self.type = 'none'

        self.initialise_ros()

    def status_callback(self, msg):
        self.handover_successful= msg.handover_successful
        self.time_left = msg.time_left

    def hand_pose_callback(self,msg):
        self.orientation = msg.orientation

    def human_input_callback(self, msg):
        self.human_input = msg.data

    def initialise_ros(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            rospy.Subscriber('Task_status',secondary_task_message,self.status_callback)
            rospy.Subscriber('hand_pose',hand_pose,self.hand_pose_callback)
            rospy.Subscriber('human_input',Bool,self.human_input_callback)
            self.rate=rospy.Rate(1)
            try:
                rospy.init_node('Environment', anonymous=True)
            except:
                rospy.logwarn("Cannot initialize node 'Environment' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline! Environment proceeds in offline mode")

    def check_phase_transition(self, action):
        if self.phase == 0: return action in [1, 2]
        elif self.phase == 1: return action in [3, 4]
        elif self.phase == 2: return action in [5, 6, 7]
        elif self.phase == 3: return action == 5
        else: return False
        
    def update_phase(self):
        if self.state == 0: self.phase = 0
        elif self.state in [1,2,3,4]: self.phase = 1
        elif self.state in [5,6,7,8,9,10]: self.phase = 2
        elif self.state in [11,12,13,14,15,16]: self.phase = 3
        else: rospy.logwarn(f"No valid phase found, phase is:{self.phase}")

    def update_state(self, action):
        if self.state == 0:
            if action == 1: return 2 if self.orientation == 'None' else 1  # self.orientation == 'None' means that the hand is not in the workspace
            elif action == 2: return 4 if self.orientation == 'None' else 2
            
        elif self.state in [1,2,3,4]:
            if action == 3:
                if self.orientation == 'Serve': return 5 
                elif self.orientation == 'Drop': return 7
                else: return 9
            elif action == 4:
                if self.orientation == 'Serve': return 6
                elif self.orientation == 'Drop': return 8
                else: return 10

        elif self.state in [5,6,7,8,9,10]:
            if self.human_input:
                if action == 5: return 11
                elif action == 6: return 12
                elif action == 7: return 13
            else:
                if action == 5: return 14
                if action == 6: return 15
                if action == 7: return 16

        elif self.state in [11,12,13,14,15,16]:
            #if action == 0: return 0
            if self.human_input:
                if action == 5: return 11
                if action == 6: return 12
                if action == 7: return 13
            else:
                if action == 5: return 14
                if action == 6: return 15
                if action == 7: return 16

        # If no valid transition is found return the current state
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

    def obtain_reward(self,action):
        self.reward = 0
        if self.ros_running:
            if self.previous_phase == 3 and action == 5:
                # This means that the hand is now open
                while self.handover_successful == 0:
                    # Before continuing wait for confirmation that the handover was either successful or failed
                    rospy.Rate(5).sleep()
                    rospy.loginfo_once("Waiting for handover status")
                if self.handover_successful == 1:
                    self.reward += self.observation_size * 10 # Reward for finishing is 10 (for each state)
                    self.reward += self.observation_size * self.time_left * 2 # Dynamic reward based on performance

                elif self.handover_successful == -1:
                    self.reward -= self.observation_size * 10 

            if self.type == 'leader' and self.state in [5,7]:
                self.reward += 10 # robot prefers the state 'serve'

            if self.previous_state in [12,15] and action in [6,7]:
                self.reward -= 2 # Incentive not to close the hand

            if self.previous_state in [13,16] and action == 7:
                self.reward -= 2 # Incentive not to keep the hand closed

        else:
            self.reward = 0 # OFFLINE REWARD FUNCTION GOES HERE

     
    
    def reset(self):
        self.state = 0
        self.phase = 0
        self.reward = 0
        self.episode_length = self.max_episode_length 
        self.hand_open = False
        return self.state, self.phase

    def close(self):
        pass



