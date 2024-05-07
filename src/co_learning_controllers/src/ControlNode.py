#!/usr/bin/env python3

import sys
import os

# Get the absolute path of the workspace's 'src' directory
workspace_src = os.path.join(os.path.dirname(__file__), '../../..', 'src')

# Append the workspace's 'src' directory to the Python path
sys.path.append(workspace_src)


import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float32MultiArray
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn
from co_learning_messages.msg import secondary_task_message
from HandController import SoftHandController




class RoboticArmControllerNode:
    def __init__(self, num_test_runs, exploration_factor=0.8):
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
        self.state = 0
        self.terminated = False
        self.episode = 0
        self.home_pos = [-0.7, -0.7, 0.5, 0.0, 0.0, 0.0]
        self.save_position = [0,0,0]
        self.i = 0
        self.initialise = True
        self.max_exploration_factor = exploration_factor
        self.secondary_task_proceed = False
        self.update_phase = True
        self.successfull_handover = 0

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

        self.rate = rospy.Rate(2)

    def status_callback(self, msg):
        # Log update the variable that tells the system to proceed when True is received on the secondary task callback
        #rospy.loginfo("Draining_starts: %s, Draining_successfull: %s, Handover_successfull: %s", msg.draining_starts, msg.draining_successfull,msg.handover_successfull)
        self.secondary_task_proceed = msg.secondary_task_start
        self.draining_starts = msg.draining_starts
        self.draining_success = msg.draining_successfull
        self.successfull_handover = msg.handover_successfull


    def send_position_command(self, position):
        """
        Sends a position command to the robotic arm.

        Args:
            position (list): The desired position coordinates.
        """
        goal = ControllerGoal()
        goal.mode = 'ee_cartesian_ds'
        goal.time = 3
        goal.precision = 1e-1
        goal.rate = 100
        goal.stiffness = 10 * np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
        goal.damping = 2 * np.sqrt(goal.stiffness)
        goal.nullspace_gain = [0] * 7
        goal.nullspace_reference = [0] * 7
        if self.state == 1:
            self.save_position = position[0:2]
            goal.reference[0:2] = self.save_position
            goal.reference[3:5] = [np.pi,np.pi,np.pi]
        if self.state == 2:
            goal.reference[0:2] = self.save_position
            goal.reference[3:5] = position[3:5]
        else:
            goal.reference = position
        goal.velocity_reference = np.zeros(6)

        self.client.wait_for_server()
        self.client.send_goal(goal)
        self.client.wait_for_result()

        return True
        
    def phase_0(self):
        if self.update_phase:
            self.action, self.phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)
            rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
            self.update_phase = False

        if self.action == 0 and self.draining_starts == 1:
            self.state = 1
            self.start_episode()
        elif self.action == 1 and self.draining_success == 1:
            self.state = 1
            self.start_episode()
        else:
            self.rate.sleep()
            self.phase_0()

    def phase_1(self):
        action, phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)

        rospy.loginfo(f"Episode:{self.episode}, Phase:{phase}, Action:{action}")

        position = self.convert_action_to_position(self.phase, action)
        _= self.send_position_command(position)
        self.state = 2
        self.start_episode()

    def phase_2(self):
        self.exploration_factor = max(self.exploration_factor * 0.90, 0.1)  # Decay exploration factor
        action, phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)

        rospy.loginfo(f"Episode:{self.episode}, Phase:{phase}, Action:{action}")
        self.episode += 1

        position = self.convert_action_to_position(self.phase, action)
        _= self.send_position_command(position)
        self.state = 3
        self.start_episode()


    def open_hand(self): #TODO: Implement functionality
        hand_open = self.hand_controller.set_percentage(100) #TODO: Functionality not yet implemented

        if self.terminated:
            self.state = 5
        else:
            self.state = 4
        if self.successfull_handover == 1 or self.successfull_handover == -1:
            self.rl_agent.save_q_table(prefix="intermittend_q_table_")
            self.start_episode()
        else:
            self.rate.sleep()
            self.open_hand()

    def home_position(self):
        if self.update_phase:
            rospy.loginfo(f"Episode:{self.episode}, Phase:/, Action:home_pos")
            _=self.send_position_command(self.home_pos)
            self.update_phase = False
        self.state = 0

        if self.secondary_task_proceed:
            self.start_episode()
        else:
            self.rate.sleep()
            self.home_position()

    def stop(self):
        if self.num_test_runs > 0:
            self.reset()
        else:
            return
        

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Start: Initialise the robot by homing it
        - State 0: Go to the state: 'phase_0'. Perform a step of the q_learning agent, go to state 1
        - State 1: Go to the state: 'phase_1'. Perform a step of the q_learning agent, go to state 2
        - State 2: Go to the state: 'phase_2'. Perform a step of the q_learning agent, go to state 3
        - State 3: Open the robotic hand, if simulation continues go to state 4, otherwise go to state 5
        - State 4: Home the robotic arm, go to state 0
        - State 5: End simulation, reset state machine
        """
        print(f"State:{self.state}, Successfull handover:{self.successfull_handover}")
        self.update_phase = True
        if self.initialise:
            self.initialise = False
            self.home_position()
            
        if self.state == 0:
            self.phase_0()

        if self.state == 1:
            self.phase_1()

        if self.state == 2:
            self.phase_2()

        if self.state == 3:
            self.open_hand()

        if self.state == 4:
            self.home_position()

        if self.state == 5:
            self.stop()

             
    def convert_action_to_position(self, phase, action):
        # TODO: Find correct values
        positions = {
            (1, 0): [-0.7,  0.7, 0.5, 0, 0, 0],                 # Location A
            (1, 1): [-0.7,  0.7, 0.7, 0, 0, 0],                 # Location B
            (2, 0): [   0,    0,   0,     0, np.pi/2,       0], # Hand orientation A
            (2, 1): [   0,    0,   0, np.pi,       0, np.pi/2], # Hand orientation B
        }
        return positions.get((phase, action), [0] * 6)

    def reset(self):
        self.state, self.phase, self.terminated = 0,0,False
        _,_,_=self.rl_agent.reset()
        self.episode = 0
        self.exploration_factor = self.max_exploration_factor
        self.start_episode()
            
        
if __name__ == '__main__':
    try:
        num_test_runs = 2  # Specify the number of test runs
        persistance_factor = 0.5
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=0.9)
        node.rl_agent.load_q_table('co_learning_robot_personalities/src/q_learning/Q_tables/q_table_solved_100000_1.npy')
        node.rl_agent.q_table*persistance_factor

        for phase in range(node.rl_agent.env.phase_size):
            print(f"Q_table for phase {phase}:\n{node.rl_agent.q_table[:,:,phase]}")

        node.reset()

    except rospy.ROSInterruptException:
        pass
