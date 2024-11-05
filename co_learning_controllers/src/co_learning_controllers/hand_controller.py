#!/usr/bin/python3
import rosgraph
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
import sys
import numpy as np
import time


class SoftHandController:
    def __init__(self, fake=False):
        self.fake = fake
        self.position = None

        if not self.fake:
            if not rosgraph.is_master_online():
                print("ROS is offline! Shutting down")
                sys.exit(0)

            try:
                rospy.init_node('hand_controller')
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(f"Cannot initialize node 'hand_controller' as it has already been initialized")

            # Initialize publisher and subscriber
            self.pub = rospy.Publisher(
                '/qbhand1/control/qbhand1_synergy_trajectory_controller/command',
                JointTrajectory,
                queue_size=10
            )
            rospy.Subscriber('/qbhand1/control/joint_states', JointState, self.callback)
        else:
            print("[INFO] Running qb_hand in mock mode. No connection to real hardware or ROS required.")
            self.position = 0.5  # Mock an initial position

    def callback(self, msg):
        """Callback function for joint_states subscriber."""
        if not self.fake:
            self.position = msg.position[0]

    def send_goal(self, mode: str, duration: int):
        """
        Sends a goal to the hand controller.

        Parameters:
            mode (str): Mode of operation ('open', 'close', 'partial').
            duration (int): Duration in seconds for the movement.
        """
        if mode not in ['open', 'close', 'partial']:
            raise ValueError("Invalid mode. Expected 'open', 'close', or 'partial'.")

        if not self.fake:
            rospy.loginfo(f"{mode} for {duration} s")
        
        goal = self.get_qbhand_goal(mode, duration)

        if not self.fake:
            self.pub.publish(goal)
        else:
            print(f"[MOCK] Published goal for mode: {mode}")

        time.sleep(duration)

    def get_qbhand_goal(self, mode='open', duration=5, n_interval=20):
        """
        Generates a JointTrajectory goal based on the mode and duration.

        Parameters:
            mode (str): Mode of operation ('open', 'close', 'partial').
            duration (int): Duration in seconds for the movement.
            n_interval (int): Number of intervals for trajectory points.

        Returns:
            JointTrajectory: The trajectory goal.
        """
        if mode not in ['open', 'close', 'partial']:
            raise ValueError("Invalid mode. Expected 'open', 'close', or 'partial'.")

        if not self.fake:
            while self.position is None and not rospy.is_shutdown():
                rospy.loginfo("Waiting for joint position...")
                rospy.sleep(1)
        else:
            print(f"[MOCK] Generating mock trajectory for mode: {mode}")
            if self.position is None:
                self.position = 0.5  # Default mock position

        goal = JointTrajectory()
        goal.header.stamp = rospy.Time.now() if not self.fake else rospy.Time(0)
        goal.joint_names = ['qbhand1_synergy_joint']

        start_position = self.position
        if mode == 'open':
            end_position = 0.0
        elif mode == 'close':
            end_position = 1.0
        elif mode == 'partial':
            end_position = 0.5

        tp = np.linspace(start_position, end_position, n_interval)
        tt = np.linspace(0.1, duration, n_interval)

        for i in range(n_interval):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(float(tt[i]))
            point.positions = [float(tp[i])]
            goal.points.append(point)

        return goal

if __name__ == '__main__':
    controller = SoftHandController(fake=False)
    controller.send_goal('open', 1)
