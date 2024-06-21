#!/usr/bin/env python3
import signal
import string
import sys
import rospy
import numpy as np
import os
from pathlib import Path

# Add the root directory to sys.path
sys.path.append('/home/jesse/Thesis/co_learning_robot_personalities/src')

from co_learning_messages.msg import secondary_task_message, hand_pose
from co_learning_controllers.src.hand_controller import SoftHandController
from q_learning.src.QLearnAgent import QLearningAgent
from q_learning.src.CoLearnEnvironment import CoLearn
from co_learning_controllers.src.robot_controller import RoboticArmController 

HOME_POSITION = [np.pi/2, np.pi/4, 0, -np.pi/4, 0, np.pi/4, 0]
INTERMEDIATE_POSITION = [np.pi/2, 0, 0, 0, 0, 0, 0]

class RoboticArmControllerNode:
    def __init__(self, num_test_runs: int, exploration_factor: float = 0.25, personality_type: string = 'baseline'):
        self.num_test_runs = num_test_runs
        if personality_type == 'independent':
            self.exploration_factor = 0.8
        else:
            self.exploration_factor = exploration_factor

        self.phase = 0
        self.terminated = False
        self.episode = 0
        self.successful_handover = 0
        self.run = True
        self.update = False
        self.stop = False
        self.action = 0
        self.msg = secondary_task_message()
        self.start = 0
        self.q = None
        self.hand_pose = [0, 0, 0]
        self.orientation = 'None'
        self.type = personality_type

        rospy.init_node('robotic_arm_controller_node', anonymous=True)
        rospy.Subscriber('Task_status', secondary_task_message, self.status_callback)
        rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)

        self.pub = rospy.Publisher('Task_status', secondary_task_message, queue_size=1)

        self.env = CoLearn()
        if personality_type == 'independent':
            self.env.type = 'independent'
        self.rl_agent = QLearningAgent(env=self.env)
        #self.hand_controller = SoftHandController()
        self.robot_arm_controller = RoboticArmController()  

        self.alpha = 0.15  
        self.gamma = 0.8  
        self.Lamda = 0.3  

        self.rate = rospy.Rate(.5)

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.stop = True
        sys.exit(0)

    def status_callback(self, msg):
        self.msg = msg
        self.successful_handover = msg.handover_successful
        self.start = msg.draining_starts

    def hand_pose_callback(self, msg):
        self.hand_pose = [msg.x, msg.y, msg.z]
        self.orientation = msg.orientation

    def phase_0(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION)
        _ = self.robot_arm_controller.send_position_command(HOME_POSITION)
        #self.hand_controller.send_goal('close',2)

        if self.type == 'impatient':
            # Implement: Reduce delay between commands, flash LED, send reminder to human
            pass
        elif self.type == 'leader':
            # Implement: Send preparatory message, give instructions on next steps
            pass

    def phase_1(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        count = 0
        if self.action == 1:
            while self.start == 0: # Alsways wait untill the human has at least started thed draining process
                self.msg.reset = True
                self.send_message()
                self.rate.sleep()
                if self.successful_handover == -1:
                    break
        if self.action == 2:
            while self.start == 0:
                self.msg.reset = True
                self.send_message()
                self.rate.sleep()
                if self.successful_handover == -1:
                    break

            self.msg.reset = False
            self.send_message()
            self.original_orientation = self.orientation
            while self.original_orientation == self.orientation and self.successful_handover != -1:

                if self.type == 'impatient':
                    count += 1
                    if count > 10:
                        #TODO: implement: shake arm, send message (hurry up), make beeping sound, pulse light
                        count = 0
                elif self.type == 'leader':
                    if count > 10:
                        count = 0
                        #TODO: implement: send instructions like: start handover now or 'remember to maximise score', have led be green
                elif self.type == 'cautious':
                    if count > 10:
                        count = 0
                        # Implement: Wait for confirmation signal, slow movement, provide continuous feedback
                        pass

                self.rate.sleep()
            return

    def phase_2(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
        position = self.convert_action_to_orientation(self.action)
        _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION)
        _ = self.robot_arm_controller.send_position_command(position)
        self.robot_arm_controller.move_towards_hand()

        if self.type == 'impatient':
            # Implement: Move quickly, skip non-essential checks
            pass
        elif self.type == 'leader':
            # Implement: Send updates to human, provide guidance on positioning
            pass
        elif self.type == 'cautious':
            # Implement: Move slowly and steadily, provide continuous feedback
            pass

        return

    def phase_3(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: {self.phase}, Action: {self.action}")
    
        if self.action == 5:
            self.hand_controller.send_goal('open',2)
        elif self.action == 6:
            self.hand_controller.send_goal('partial',2)
        elif self.action == 7:
            pass

        self.rate.sleep()
        self.robot_arm_controller.move_towards_hand()
        return

    def update_q_table(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: 4, Action: Experience replay")
        self.rl_agent.experience_replay(self.alpha, self.gamma, self.Lamda)
        self.rate.sleep()
        return

    def check_end_condition(self):
        rospy.loginfo(f"Episode: {self.episode}, Phase: 4, Action: Resume_experiment = {self.num_test_runs > self.episode}")
        if self.num_test_runs > self.episode:
            self.episode += 1
            self.reset()
            if self.type == 'leader':
                pass
                #TODO: implement; send message to secondary task: 'well done!', 'nice score', '
            return
        else:
            _ = self.robot_arm_controller.send_position_command(INTERMEDIATE_POSITION)
            self.run = False
            return

    def send_message(self, phase=None):
        if self.msg is not None:
            msg = self.msg
        else:
            msg = secondary_task_message()
        if phase is not None:
            msg.phase = phase
        self.pub.publish(msg)

    def start_episode(self):
        """
        Implements the finite state machine of the actions the robot has to take
        - Phase 0: Home
        - Phase 1: Decide on handover moment
        - Phase 2: Decide on a handover orientation & Move end-effector to coordinates of human-hand
        - Phase 3: Decide on when to open the hand
          After termination: 
        - Update q-table with experience replay
        - If n_run < runs: Phase_0 else: end
        """
        self.relevant_part = None
        if not self.terminated:
            if self.phase == 0:
                self.phase_0()
            if self.phase == 1:
                self.phase_1()
            if self.phase == 2:
                self.phase_2()
            if self.phase == 3:
                self.phase_3()
                
            self.action, self.phase, self.terminated = self.rl_agent.train(
                learning_rate=self.alpha,
                discount_factor=self.gamma,
                trace_decay=self.Lamda,
                exploration_factor=self.exploration_factor,
                real_time=True
            )
        elif self.run:
            self.update_q_table()
            self.check_end_condition()

        if self.run:
            self.start_episode()

    def convert_action_to_orientation(self, action):
        positions = {
            3: np.deg2rad([107, -47, -11, 100, -82, -82, -35]), # Serve
            4: np.deg2rad([55, -40, -8, 82, 5, 20, 0]) # Drop
        }
        return positions.get(action, HOME_POSITION)

    def reset(self):
        _, self.phase = self.rl_agent.reset()
        self.terminated = False
        self.reset_msg()
        self.rl_agent.print_q_table()
        if self.type == 'independent':
            self.exploration_factor = max(self.exploration_factor *.9, 0.20)
        return

    def reset_msg(self):
        self.msg = secondary_task_message()
        self.msg.draining_starts = 0
        self.msg.draining_successful = 0
        self.msg.reset = False
        self.msg.phase = 0
        self.start = 0
        self.orientation = 'None'
        self.successful_handover = 0
        self.send_message()
        return

if __name__ == '__main__':
    try:
        num_test_runs = 10
        persistence_factor = 0.5
        node = RoboticArmControllerNode(num_test_runs, exploration_factor=100)

        base_dir = Path(__file__).resolve().parent.parent.parent
        print(base_dir)
        q_table_path = base_dir / 'q_learning/Q_tables/q_table_solved_100000_38.npy'

        if os.path.isfile(q_table_path):
            # node.rl_agent.load_q_table(str(q_table_path))
            # node.rl_agent.q_table *= persistence_factor
            node.rl_agent.print_q_table()
            node.start_episode()
        else:
            rospy.logerr(f"Q-table file not found: {q_table_path}")

    except rospy.ROSInterruptException:
        pass



