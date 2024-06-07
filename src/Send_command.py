# Used in the main computations
import numpy as np

# ROS libraries
import rospy # CC
import actionlib

# Mesages to and from our KUKA baby
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from sensor_msgs.msg import JointState
from robot.robot import Robot

class arm_controller():
    def __init__(self):
        rospy.init_node('torque_controller_example')
        rospy.Subscriber("/iiwa7/joint_states", JointState, self.callback, queue_size=10)

        ns = rospy.get_param('/namespaces')
        self.client = actionlib.SimpleActionClient(ns + '/torque_controller', ControllerAction)
        self.robot = Robot(model=ns.replace('/', ''))
        rospy.loginfo("waiting for response from server")
        self.client.wait_for_server()
        rospy.loginfo("server initialised")
        self.q_i = [0,0,0,0,0,0,0]
        self.ref = [0,0,2,0,0,0]

    def callback(self,msg):
        self.q = msg.position
        print(self.q)
        self.q_dot =msg.velocity

    def run(self):
        while True:
            # Define message
            goal = ControllerGoal()
            goal.mode = 'joint_ds'

            # Fill in parameters
            goal.time = 5 #(seconds). This parameter defines the time in which required movement is executed
            goal.precision = 1e-1
            goal.rate = 10 #(Hz)
            goal.stiffness = 3*np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10, 10])
            goal.damping = 2*np.sqrt(goal.stiffness)
            goal.reference = self.ref
            goal.velocity_reference =np.zeros(6)

            self.client.send_goal(goal)
            result = self.client.wait_for_result()
       
if __name__ == '__main__':
    try:
        controller = arm_controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass