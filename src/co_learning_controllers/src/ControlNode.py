#!/usr/bin/env python3

import sys
import os
from pathlib import Path

# Get the absolute path of the workspace's 'src' directory
workspace_src = os.path.join(os.path.dirname(__file__), '../../..', 'src')

# Append the workspace's 'src' directory to the Python path
sys.path.append(workspace_src)


import rospy
import actionlib
import numpy as np
import threading

from std_msgs.msg import Float32MultiArray
from cor_tud_msgs.msg import ControllerAction, ControllerGoal

from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn
from co_learning_messages.msg import secondary_task_message
from HandController import SoftHandController

# Constants
HOME_POSITION = [-0.7, -0.7, 0.5, 0.0, 0.0, 0.0]
RATE_HZ = 2
GOAL_MODE = 'ee_cartesian_ds'
GOAL_TIME = 3
GOAL_PRECISION = 1e-1
GOAL_RATE = 100
GOAL_STIFFNESS = 10 * np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
GOAL_DAMPING = 2 * np.sqrt(GOAL_STIFFNESS)
GOAL_NULLSPACE_GAIN = [0] * 7
GOAL_NULLSPACE_REFERENCE = [0] * 7




class RoboticArmControllerNode:
    def __init__(self, num_test_runs: int, exploration_factor: float = 0.8):
        """
        Initializes the RoboticArmControllerNode, handles the sending of position commands and 
        uses the QLearnAgent() and CoLearn() classes for reinforcement learning purposes

        Args:
            num_test_runs (int): The number of test runs to perform.
            exploration_factor (float): The exploration factor for the Q-learning agent.
        """
        self.num_test_runs = num_test_runs
        self.exploration_factor = exploration_factor
        self.phase = 0
        self.terminated = False
        self.episode = 0
        self.save_position = [0,0,0]
        self.max_exploration_factor = exploration_factor
        self.secondary_task_proceed = False
        self.successfull_handover = 0
        self.run = True

        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        rospy.Subscriber('Task_status',secondary_task_message,self.status_callback)

        self.pub = rospy.Publisher('Task_status',secondary_task_message,queue_size=1)

        ns = rospy.get_param('/namespaces')
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)
        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        self.env = CoLearn()
        self.rl_agent = QLearningAgent(env=self.env)
        self.hand_controller = SoftHandController()

        self.condition = threading.Condition()
        self.relevant_part = None

        self.rate = rospy.Rate(1)

    def status_callback(self, msg):
        """
        Callback function to update secondary task states.
        Args:
            msg (secondary_task_message): The message containing task status updates.
        """
        self.successfull_handover = msg.handover_successfull
        with self.condition:
            if self.phase == 1 and msg.draining_starts != 0:
                self.relevant_part = {'draining_starts': msg.draining_starts}
            elif self.phase == 1 and msg.draining_successfull != 0:
                self.relevant_part = {'draining_successfull': msg.draining_successfull}
            elif self.phase == 4 and msg.handover_successfull != 0:
                self.relevant_part = {'handover_successfull': msg.handover_successfull}
            if self.relevant_part is not None:
                self.condition.notify()

    def create_goal(self, position):
        goal = ControllerGoal()
        goal.mode = GOAL_MODE
        goal.time = GOAL_TIME
        goal.precision = GOAL_PRECISION
        goal.rate = GOAL_RATE
        goal.stiffness = GOAL_STIFFNESS
        goal.damping = GOAL_DAMPING
        goal.nullspace_gain = GOAL_NULLSPACE_GAIN
        goal.nullspace_reference = GOAL_NULLSPACE_REFERENCE

        if self.phase == 2:
            self.save_position = position[0:3]
            goal.reference[0:3] = self.save_position
            goal.reference[3:6] = [np.pi, np.pi, np.pi]
        elif self.phase == 3:
            goal.reference[0:3] = self.save_position
            goal.reference[3:6] = position[3:6]
        else:
            goal.reference = position
        goal.velocity_reference = np.zeros(6)
        return goal


    def send_position_command(self, position: list) -> bool:
        goal = self.create_goal(position)
        try:
            self.client.wait_for_server()
            self.client.send_goal(goal)
            self.client.wait_for_result()
        except actionlib.ActionException as e:
            rospy.logerr(f"Action client error: {e}")
            return False
        return True
    
    def home_position(self):
        _=self.send_position_command(HOME_POSITION)
        return
       
    def phase_1(self):
        with self.condition:
            while not (self.relevant_part and (self.relevant_part.get('draining_starts') == 1 or self.relevant_part.get('draining_successfull') == 1)):
                self.condition.wait()
        
    def phase_2(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        position = self.convert_action_to_position(self.action)
        _= self.send_position_command(position)
        return

    def phase_3(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        position = self.convert_action_to_position(self.action)
        _= self.send_position_command(position)
        return

    def phase_4(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        hand_open = self.hand_controller.set_percentage(100) 
        
        if self.successfull_handover == -1:
            return
        else:
            with self.condition:
                if self.relevant_part and self.relevant_part.get('handover_successfull') == -1:
                    return
                while not (self.relevant_part and self.relevant_part.get('handover_successfull') in [-1, 1]):
                    self.condition.wait()
    


    def phase_5(self):
        self.rl_agent.experience_replay(0.15,0.8,0.3) # TODO: dynaically link these values
        return

    def phase_6(self):
        if self.num_test_runs > self.episode:
            self.episode += 1
            self.reset()
            return
        else:
            self.run = False
            return

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Phase 0: Home
        - Phase 1: Decide on a starting time
        - Phase 2: Decide on a handover location
        - Phase 3: Decide on a handover location
        - Phase 4: Open the robotic hand
          After termination: 
        - Phase 5: Update q-table with experience replay
        - Phase 6: If n_run < runs: Phase_0 else: end
        """
        self.relevant_part = None
    
        if not self.terminated:
            if self.phase == 0:
                self.home_position()
                
            if self.phase == 1:
                self.phase_1()

            if self.phase == 2:
                self.phase_2()

            if self.phase == 3:
                self.phase_3()

            if self.phase == 4:
                self.phase_4()

            self.action, self.phase, self.terminated = self.rl_agent.train(exploration_factor = self.exploration_factor,real_time = True)
        else:
            self.phase_5()
            self.phase_6()

        if self.run:
            self.start_episode()


             
    def convert_action_to_position(self, action):
        # TODO: Find correct values
        positions = {
            3: [-0.7,  0.7, 0.5, 0, 0, 0],                 # Location A
            4: [-0.7,  0.7, 0.7, 0, 0, 0],                 # Location B
            5: [   0,    0,   0,     0, np.pi/2,       0], # Hand orientation A
            6: [   0,    0,   0, np.pi,       0, np.pi/2], # Hand orientation B
        }
        return positions.get(action, HOME_POSITION)

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.exploration_factor = self.max_exploration_factor
        return
            
        
if __name__ == '__main__':
    try:
        num_test_runs = 2  # Specify the number of test runs
        persistence_factor = 0.5
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=0.9)
        q_table_path = Path('co_learning_robot_personalities/src/q_learning/Q_tables/q_table_solved_100000_38.npy')
        
        if q_table_path.exists():
            node.rl_agent.load_q_table(str(q_table_path))
            node.rl_agent.q_table *= persistence_factor
            print("Q-Table:")
            header = "".join([f"{'Action ' + str(i):<{12}}" for i in range(8)])
            print(f"{'':<{12}}" + header)
            for state in range(8):
                row = f"State {state:<{12 - 6}}"  # Adjust for 'State ' prefix length
                row += "".join([f"{node.rl_agent.q_table[state, action]:<{12}.2f}" for action in range(8)])
                print(row)
            node.start_episode()
        else:
            rospy.logerr(f"Q-table file not found: {q_table_path}")

    except rospy.ROSInterruptException:
        pass
