#!/usr/bin/python3
import rosgraph
import rospy
import trajectory_msgs.msg

class SoftHandController:
    def __init__(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('soft_hand_controller')
            except:
                rospy.logwarn("Cannot initialize node 'soft_hand_controller' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline!")

        # Initialize ROS node and publisher
        
        self.pub = rospy.Publisher('/qbhand1/control/qbhand1_synergy_trajectory_controller/command',
                                   trajectory_msgs.msg.JointTrajectory, queue_size=10)
        # Initialize JointTrajectory message
        self.msg = trajectory_msgs.msg.JointTrajectory()
        self.msg.header.seq = 0
        self.msg.joint_names = ["qbhand1_synergy_joint"]
        # Initialize a point in the trajectory message
        self.point = trajectory_msgs.msg.JointTrajectoryPoint()
        self.point.positions = [0.0]
        self.point.time_from_start = rospy.Time.from_sec(0.2)
        self.msg.points = [self.point]

    def set_percentage(self, percentage):
        # Set the goal position based on the percentage (0 = fully open, 100 = fully closed)
        goal = (100 - percentage) / 100 if percentage >= 0 else 0
        self.point.positions = [goal]
        self.pub.publish(self.msg)
        return True

if __name__ == "__main__":
    controller = SoftHandController()
    rospy.sleep(1)  # Wait for initialization
    controller.set_percentage(50)  # Set the hand to 50% closed
    rospy.sleep(1)  # Wait for the movement to complete
    controller.set_percentage(0)  # Open the hand fully
