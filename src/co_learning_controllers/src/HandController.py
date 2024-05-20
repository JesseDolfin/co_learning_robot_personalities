#!/usr/bin/python3

import rospy
import trajectory_msgs.msg


class SoftHandController:
    def __init__(self):
        self.pub = rospy.Publisher('/qbhand1/control/qbhand1_synergy_trajectory_controller/command',
                                            trajectory_msgs.msg.JointTrajectory, queue_size=10)

        self.msg = trajectory_msgs.msg.JointTrajectory()
        self.msg.header.seq = 0
        self.msg.joint_names = ["qbhand1_synergy_joint"]

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0.0]
        point.time_from_start = rospy.Time.from_sec(0.2)

        self.msg.points = [point]
        self.open()

    def _publish(self):
        self.pub.publish(self.msg)

    @property
    def goal(self):  # 1 = closed, 0 = open!
        return self.msg.points[0].positions[0]

    @property
    def percentage_open(self):
        return round(100 - self.goal * 100)

    @property
    def percentage_closed(self):
        return round(100 * self.goal)

    def _set_goal(self, goal):
        self.msg.points[0].positions = [goal]

    def set_speed(self, speed):
        self.msg.points[0].time_from_start = rospy.Time.from_sec(speed)

    def open(self, percentage=100):
        goal = 1 - percentage/100
        self._set_goal(goal)
        self._publish()

    def close(self, percentage=100):
        goal = percentage/100
        self._set_goal(goal)
        self._publish()
