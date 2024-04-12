#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float32MultiArray
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from QLearnAgent import QLearningAgent
from CoLearnEnvironment import CoLearn
from PA3_MAIN import secondary_task


class RoboticArmControllerNode:
    def __init__(self, num_test_runs, exploration_factor=0.8, simulate_mode=False):
        """
        Initializes the RoboticArmControllerNode, handles the sending of position commands and 
        uses the QLearnAgent() and CoLearn() classes for reinforcement learning purposes

        Args:
            num_test_runs (int): The number of test runs to perform.
            simulate_mode (bool): Whether to run in simulation mode.
            exploration_factor (float): The exploration factor for the Q-learning agent.
        """
        self.num_test_runs = num_test_runs
        self.exploration_factor = exploration_factor
        self.simulate_mode = simulate_mode
        self.phase = 0
        self.state = 0
        self.terminated = False
        self.prev_phase = 1
        self.episode = 0
        self.flag = True
        self.home_pos = [-0.7, -0.7, 0.5, 0.0, 0.0, 0.0]
        self.i = 0
        self.count = 0
        self.max_exploration_factor = exploration_factor

        rospy.init_node('robotic_arm_controller_node', anonymous=True)

        ns = rospy.get_param('/namespaces')
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)
        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        self.env = CoLearn()
        self.rl_agent = QLearningAgent(env=self.env)


    def result_callback(self, msg):
        # TODO: Figure out how to obtain information once movement is done.
        rospy.loginfo(msg)

    def send_position_command(self, position):
        """
        Sends a position command to the robotic arm.

        Args:
            position (list): The desired position coordinates.
        """
        goal = ControllerGoal()
        goal.mode = 'ee_cartesian'
        goal.time = 3
        goal.precision = 1e-1
        goal.rate = 100
        goal.stiffness = 10 * np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
        goal.damping = 2 * np.sqrt(goal.stiffness)
        goal.nullspace_gain = [0] * 7
        goal.nullspace_reference = [0] * 7
        goal.reference = position
        goal.velocity_reference = np.zeros(6)

        #self.client.wait_for_server()
        self.client.send_goal(goal)
        self.client.wait_for_result()

        rospy.sleep(goal.time)  # FOR SIMULATION PURPOSE. TODO: THIS NEEDS TO BE REPLACED BY PROPER 'WAIT FOR CONFIRMATION OF GOOD MOVEMENT'!

        self.start_episode()

    def start_episode(self):
        """
        Starts an episode of the robotic arm controller.
        Contains functionality for simulation mode, triggered by the self.simulation_mode flag and real-time training mode.
        - In simulation mode an existing q_table is used and the agent just repeats its highest action value.
        - In real-time training mode the agent uses an existing q_table (trained to incorporate a certain preference) but this time
        the agent re-trains this q_table to enable co learning
        """
        if not self.simulate_mode:
            if self.phase == 1 and self.flag:
                self.flag = False
                rospy.loginfo(f"Episode:   Action: Home_Pos")
                self.send_position_command(self.home_pos)

            self.episode += 1
            self.exploration_factor = max(self.exploration_factor * 0.90, 0.1)  # Decay exploration factor
            self.action, self.phase, self.terminated = self.rl_agent.train_real_time(exploration_factor=self.exploration_factor)
            position = self.convert_action_to_position(self.phase, self.action)

            rospy.loginfo(f"Episode:{self.episode}, Action:{self.action}")

            if self.phase == 1:
                self.flag = True

            if not self.terminated:  # Check if episode is not terminated
                self.send_position_command(position)
            else:
                self.reset()
        else:
            if self.count <= self.num_test_runs - 1:

                if self.phase == 1 and self.flag:
                    self.flag = False
                    rospy.loginfo(f"Episode:   Action: Home_Pos")
                    self.send_position_command(self.home_pos)

                self.episode += 1
                self.action = np.argmax(self.rl_agent.q_table[self.state,:,self.phase])
                position = self.convert_action_to_position(self.phase,self.action)
                
                # Manual step operation
                self.state = self.action
                self.phase += 1
                if self.phase >= 2:
                    self.phase = 0

                rospy.loginfo(f"Episode:{self.episode}, Action:{self.action}")

                if self.phase == 1:
                    self.flag = True

                self.count += 1
                self.send_position_command(position)
            else:
                self.i = self.num_test_runs + 1 # Hacky way to do this
                self.run()

            
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

    def run(self):
        self.i += 1
        if self.i < self.num_test_runs + 1:
            rospy.loginfo(f"Starting Run:{self.i}")
            self.start_episode()
        else:
            if not self.simulate_mode:
                self.rl_agent.save_q_table()
                rospy.signal_shutdown("Maximum number of test runs reached")
            else:
                rospy.signal_shutdown("Maximum number of test runs reached")

    def reset(self):
        self.state, self.phase, self.terminated = self.rl_agent.reset()
        self.episode = 0
        self.exploration_factor = self.max_exploration_factor
        self.run()


if __name__ == '__main__':
    try:
        num_test_runs = 10  # Specify the number of test runs
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=0.9, simulate_mode=False)
        node.rl_agent.load_q_table('Co-Learning-KUKA-RL/Q_tables/q_table_solved_100000_1.npy')
        print(f"Q_table for phase 0:\n{node.rl_agent.q_table[:,:,0]}\nQ_table for phase 1:\n{node.rl_agent.q_table[:,:,1]}")
        node.run()

    except rospy.ROSInterruptException:
        pass
