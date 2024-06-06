
#!/usr/bin/env python3
import signal
import sys
import os
from pathlib import Path
import rospy
import actionlib
import numpy as np
import threading
from cor_tud_msgs.msg import ControllerAction, ControllerGoal

# Add the root directory to sys.path
sys.path.append('/home/jesse/Thesis/co_learning_robot_personalities/src')
from co_learning_messages.msg import secondary_task_message,hand_pose
from co_learning_controllers.src.HandController import SoftHandController
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn

# Constants
ROT = np.pi/4
HOME_POSITION =         [np.pi/2, ROT, 0, -ROT, 0, ROT,0]
INTERMEDIATE_POSITION = [np.pi/2,    0, 0, 0  , 0, 0 ,0]
GOAL_MODE = 'joint_ds'
GOAL_TIME = 5
GOAL_PRECISION = 1e-1
GOAL_RATE = 10
GOAL_STIFFNESS =   [100.0, 100.0, 50.0, 50.0, 25.0, 10.0, 10.0]
GOAL_DAMPING = 2 * np.sqrt(GOAL_STIFFNESS)
GOAL_NULLSPACE_GAIN =  [0,0,0,0,0,0,0]
GOAL_NULLSPACE_REFERENCE = [0,0,0,0,0,0,0]

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
        self.successful_handover = 0
        self.run = True
        self.update = False
        self.stop = False
        self.action = 0
        self.msg = None

        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        rospy.Subscriber('Task_status',secondary_task_message,self.status_callback)
        rospy.Subscriber('hand_pose',hand_pose,self.hand_pose_callback)

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

        self.alpha = 0.15 # Can change dependend on desired learning speed
        self.gamma = 0.8 # Needs to be the same as initial training
        self.Lamda = 0.3 # Needs to be the same as initial training

        self.rate = rospy.Rate(5)

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.stop = True
        sys.exit(0)

    def status_callback(self, msg):
        self.msg = msg
        self.successful_handover = msg.handover_successful

    def hand_pose_callback(self,msg):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_z = msg.z
        self.orientation = msg.orientation

    def create_goal(self, position):
        goal = ControllerGoal()
        goal.mode = GOAL_MODE
        goal.time = GOAL_TIME
        goal.precision = GOAL_PRECISION
        goal.rate = GOAL_RATE
        goal.stiffness = GOAL_STIFFNESS
        goal.damping = GOAL_DAMPING
        # goal.nullspace_gain = GOAL_NULLSPACE_GAIN
        # goal.nullspace_reference = GOAL_NULLSPACE_REFERENCE
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
    
    def phase_0(self):
        _= self.send_position_command(INTERMEDIATE_POSITION)
        _= self.send_position_command(HOME_POSITION)
        return
       
    def phase_1(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        pass

        
    def phase_2(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        pass
        return
    
    def move_towards_hand(self):
        pass

    def phase_3(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        pass
        return

    def update_q_table(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:5, Action:Experience replay")
        self.rl_agent.experience_replay(self.alpha,self.gamma,self.Lamda) 
        self.rate.sleep()
        return

    def check_end_condition(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:6, Action:Resume_experiment={self.num_test_runs > self.episode}")
        if self.num_test_runs > self.episode:
            self.episode += 1
            self.reset()
            self.rl_agent.print_q_table()
            return
        else:
            _= self.send_position_command(INTERMEDIATE_POSITION)
            self.run = False
            return
        
    def send_message(self,phase=None):
            if self.msg != None:
                msg = self.msg
            else:
                msg = secondary_task_message()
            if phase != None:
                msg.phase = phase
            self.pub.publish(msg)

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Phase 0: Home
        - Phase 1: Decide on handover moment
        - Phase 2: Decide on a handover orientation
        - Move end-effector to coordinates of human-hand
        - Phase 3: Decide on when to open the hand
          After termination: 
        - Update q-table with experience replay
        - If n_run < runs: Phase_0 else: end
        """

        self.relevant_part = None
        self.send_message(self.phase)
        if not self.terminated:
            if self.phase == 0:
                self.phase_0()
                
            if self.phase == 1:
                self.phase_1()

            if self.phase == 2:
                self.phase_2()
                self.move_towards_hand()

            if self.phase == 3:
                self.phase_3()
               
            self.action, self.phase, self.terminated = self.rl_agent.train(learning_rate=self.alpha,discount_factor=self.gamma,
                                                                        trace_decay=self.Lamda,exploration_factor = self.exploration_factor,
                                                                        real_time = True)
            
        elif self.run:
            self.update_q_table()
            self.check_end_condition()

        if self.run:
            self.start_episode()

    def convert_action_to_orientation(self, action):
        positions = {
            3: [], # Orientation A
            4: [], # Orientation B
        }
        return positions.get(action, HOME_POSITION)

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.msg.draining_starts = 0
        self.msg.draining_successful = 0
        #self.exploration_factor *= 0.8
        self.update = False
        return
            
        
if __name__ == '__main__':
    try:
        num_test_runs = 10  # Specify the number of test runs
        persistence_factor = 0.5
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=1)
        
        base_dir = Path(__file__).resolve().parent.parent.parent
        print(base_dir)
        q_table_path = base_dir / 'q_learning/Q_tables/q_table_solved_100000_38.npy'
        
        if os.path.isfile(q_table_path):
            node.rl_agent.load_q_table(str(q_table_path))
            node.rl_agent.q_table *= persistence_factor
            node.rl_agent.print_q_table()
            node.start_episode()
        else:
            rospy.logerr(f"Q-table file not found: {q_table_path}")

    except rospy.ROSInterruptException:
        pass


