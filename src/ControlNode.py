#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float32MultiArray
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn
from co_learning_messages.msg import secondary_task_message




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
        self.i = 0
        self.initialise = True
        self.max_exploration_factor = exploration_factor
        self.secondary_task_proceed = False

        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        rospy.Subscriber('Task_status',secondary_task_message,self.status_callback)

        ns = rospy.get_param('/namespaces')
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)
        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        self.env = CoLearn()
        self.rl_agent = QLearningAgent(env=self.env)

        self.rate = rospy.Rate(2)

    def status_callback(self, msg):
        # Log update the variable that tells the system to proceed when True is received on the secondary task callback
        rospy.loginfo("Tries: %s, Success: %s", msg.tries, msg.success)
        self.secondary_task_proceed = msg.success


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
        goal.reference = position
        goal.velocity_reference = np.zeros(6)

        self.client.wait_for_server()
        self.client.send_goal(goal)
        self.client.wait_for_result()

        return True
        
    def phase_0(self):
        action, phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)

        rospy.loginfo(f"Episode:{self.episode}, Phase:{phase}, Action:{action}")

        position = self.convert_action_to_position(self.phase, action)
        _= self.send_position_command(position)
        self.state = 1
        self.start_episode()

    def phase_1(self):
        self.exploration_factor = max(self.exploration_factor * 0.90, 0.1)  # Decay exploration factor
        action, phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)

        rospy.loginfo(f"Episode:{self.episode}, Phase:{phase}, Action:{action}")
        self.episode += 1

        position = self.convert_action_to_position(self.phase, action)
        _= self.send_position_command(position)
        self.state = 2
        self.start_episode()

    def open_hand(self): #TODO: Implement functionality
        if self.terminated:
            self.state = 4
        else:
            self.state = 3
        self.start_episode()

    def home_position(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:/, Action:home_pos")
      
        self.state = 0
        if self.secondary_task_proceed:
            _=self.send_position_command(self.home_pos)
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
        - State 0: Go to the state: 'phase_0'. Perform a step of the q_learning agent and send position command to the robot, go to state 1
        - State 1: Go to the state: 'phase_1'. Perform a step of the q_learning agent and send position command to the robot, go to state 2
        - State 2: Open the robotic hand, if simulation continues go to state 3, otherwise go to state 4
        - State 3: Home the robotic arm, go to state 0
        - State 4: End simulation, reset state machine
        """

        if self.initialise:
            self.initialise = False
            self.home_position()
            
        if self.state == 0:
            self.phase_0()

        if self.state == 1:
            self.phase_1()

        if self.state == 2:
            self.open_hand()

        if self.state == 3:
            self.home_position()

        if self.state == 4:
            self.stop()

             
    def convert_action_to_position(self, phase, action):
        # TODO: Find correct values
        positions = {
            (0, 0): [-0.5, 0.3, 0.5, 0, 0, 0],
            (0, 1): [-0.5, -0.3, 0.7, 0, 0, 0],
            (0, 2): [0.5, 0.3, 0.7, 0, 0, 0],
            (1, 0): [-0.5, 0.3, 0.5, 0, 0, 0],
            (1, 1): [-0.5, -0.3, 0.7, 0, 0, 0],
            (1, 2): [0.5, 0.3, 0.7, 0, 0, 0],
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
        print(f"Q_table for phase 0:\n{node.rl_agent.q_table[:,:,0]}\nQ_table for phase 1:\n{node.rl_agent.q_table[:,:,1]}")
        node.reset()

    except rospy.ROSInterruptException:
        pass
