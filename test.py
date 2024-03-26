# Imports the environment
from CoLearnEnvironment import CoLearn
from QLearnAgent import QLearningAgent
from ControlNode import RoboticArmControllerNode
import rospy
import numpy as np

import Control_in_HRI
from Control_in_HRI import PA3_MAIN
from Control_in_HRI.pantograph import Pantograph

PA3_MAIN



#Agent_t = Q_learning_agent(action_size=3,state_size=3,env=CoLearn())

# Agent_t.train(n_steps=100000)
# Agent_t.save_q_table()


# Agent_t.load_q_table()
# avg = Agent_t.evaluate()

# print("Old q_table:",Agent_t.q_table)
# print("Old performance:",avg)

# Agent_t.train(n_steps=100000)
# avg = Agent_t.evaluate()

# print("New q_table",Agent_t.q_table)
# print("Average reward",avg)

# q_table_previous = np.load('code_jesse/Q_tables/q_table_solved_1000_1.npy')
# q_table = np.load('code_jesse/Q_tables/q_table_11.npy')
# q_table_solved = np.load('code_jesse/Q_tables/q_table_solved_100000_3.npy')
# print(f"\033[1m Solved Q table\n \033[0m Q_table for phase 0:\n{q_table_solved[:,:,0]}\nQ_table for phase 1:\n{q_table_solved[:,:,1]}")
# print(f"\033[1m Initial Q table\n \033[0m Q_table for phase 0:\n{q_table_previous[:,:,0]}\nQ_table for phase 1:\n{q_table_previous[:,:,1]}")
# print(f"\033[1m Re trained Q table\n \033[0m Q_table for phase 0:\n{q_table[:,:,0]}\nQ_table for phase 1:\n{q_table[:,:,1]}")

