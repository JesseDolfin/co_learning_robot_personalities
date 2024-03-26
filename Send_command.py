# Used in the main computations
import numpy as np

# ROS libraries
import rospy # CC
import actionlib

# Mesages to and from our KUKA baby
from cor_tud_msgs.msg import ControllerAction, ControllerGoal


# Imports the environment
from CoLearnEnvironment import CoLearn
from QLearnAgent import Q_learning_agent

if __name__ == "__main__":
    global bag
    try:
        rospy.init_node('torque_controller_example')
        client = actionlib.SimpleActionClient('/iiwa7/torque_controller', ControllerAction)
        client.wait_for_server()

        agent = Q_learning_agent(action_size=3,state_size=3,env=CoLearn())

            
        

        
        # Define message
        goal = ControllerGoal()
        goal.mode = 'ee_cartesian'

        # Fill in parameters
        goal.time = 5 #(seconds). This parameter defines the time in which required movement is executed
        goal.precision = 1e-1
        goal.rate = 10 #(Hz)
        goal.stiffness = 10*np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
        goal.damping = 2*np.sqrt(goal.stiffness)
        goal.nullspace_gain = [0,0,0,0,0,0,0]
        goal.nullspace_reference = [0,0,0.0,0,0,0,0]
        goal.reference = [0.4, 0.5, 0.4, 0, np.pi, 0] #[x y z rx ry rz] (meters, rad)
        goal.velocity_reference =np.zeros(6)
    
        client.send_goal(goal)
        result = client.wait_for_result()
        

    except rospy.ROSInterruptException:
        pass

