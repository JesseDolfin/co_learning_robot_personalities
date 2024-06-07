
#!/usr/bin/env python3
import signal
import sys
import os
from pathlib import Path
import rospy
import actionlib
import numpy as np
import time
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from sensor_msgs.msg import JointState

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
NULLSPACE_GAIN = [1,1,1,1,1,1,1]

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
        rospy.Subscriber("/iiwa7/joint_states", JointState, self.joint_callback, queue_size=10)

        self.pub = rospy.Publisher('Task_status',secondary_task_message,queue_size=1)

        ns = rospy.get_param('/namespaces')
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)

        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")
 
        self.env = CoLearn()
        self.rl_agent = QLearningAgent(env=self.env)
        self.hand_controller = SoftHandController()

        self.relevant_part = None
        self.orientation = 'None'

        self.alpha = 0.15 # Can change dependend on desired learning speed
        self.gamma = 0.8 # Needs to be the same as initial training
        self.Lamda = 0.3 # Needs to be the same as initial training

        self.rate = rospy.Rate(5)

        signal.signal(signal.SIGINT, self.signal_handler)

    def joint_callback(self,msg):
        self.q = msg.position
        self.q_dot =msg.velocity

    def signal_handler(self, sig, frame):
        self.stop = True
        sys.exit(0)

    def status_callback(self, msg):
        self.msg = msg
        self.successful_handover = msg.handover_successful
        self.start = msg.start

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
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        _= self.send_position_command(INTERMEDIATE_POSITION)
        _= self.send_position_command(HOME_POSITION)
        self.hand_controller.open(0)

        while self.start == 0:
            self.wait()
        return
    
    def wait(self):
        self.rate.sleep()
        return
       
    def phase_1(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        if self.action == 1:
            return
        if self.action == 2:
            self.original_orientation = self.orientation 
            while self.original_orientation == self.orientation:
                self.rate.sleep() # wait for a state change
            return

    def phase_2(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        position = self.convert_action_to_orientation(self.action)
        _ = self.send_position_command(INTERMEDIATE_POSITION)
        _ = self.send_position_command(position)
        return
    
    def move_towards_hand(self):
        rospy.Rate(5).sleep()

    def phase_3(self):
        rospy.loginfo(f"Episode:{self.episode}, Phase:{self.phase}, Action:{self.action}")
        if self.action == 5:
            self.hand_controller.open(100)
        elif self.action == 6:
            self.hand_controller.open(30)
        elif self.action == 7:
            pass
        self.rate.sleep()
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
            #self.rl_agent.print_q_table()
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
            3: np.deg2rad([107,-47,-11,100,-82,-82,-35]), # Serve
            4: np.deg2rad([55 ,-40,-8 ,82 , 5 , 20, 0 ])  # Drop
        }
        return positions.get(action, HOME_POSITION)

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.msg.draining_starts = 0
        self.msg.draining_successful = 0
        self.orientation = 'None'
        #self.exploration_factor *= 0.8
        self.update = False
        return
            
        
if __name__ == '__main__':
    try:
        num_test_runs = 10  # Specify the number of test runs
        persistence_factor = 0.5
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=100)
        
        base_dir = Path(__file__).resolve().parent.parent.parent
        print(base_dir)
        q_table_path = base_dir / 'q_learning/Q_tables/q_table_solved_100000_38.npy'
        
        if os.path.isfile(q_table_path):
            #node.rl_agent.load_q_table(str(q_table_path))
            #node.rl_agent.q_table *= persistence_factor
            node.rl_agent.print_q_table()
            node.start_episode()
        else:
            rospy.logerr(f"Q-table file not found: {q_table_path}")

    except rospy.ROSInterruptException:
        pass


