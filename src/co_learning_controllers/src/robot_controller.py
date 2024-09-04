# robotic_arm_control.py

from os import wait
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from cor_tud_msgs.msg import ControllerAction, ControllerGoal
from scipy.spatial.transform import Rotation as R
from robot.robot import Robot
from co_learning_messages.msg import hand_pose
from std_msgs.msg import Bool
import matplotlib.pyplot as plt

# Constants
HOME_POSITION = [np.pi/2, np.pi/4, 0, -np.pi/4, 0, np.pi/4, 0]
INTERMEDIATE_POSITION = [np.pi/2, 0, 0, 0, 0, 0, 0]

class RoboticArmController:
    def __init__(self):
        self.q = None
        self.goal_time = 5.0
        self.ee_pose = [0, 0, 0, 0, 0, 0]
        ns = rospy.get_param('/namespaces')

        self.robot = Robot(model=ns.replace('/',''))
        self.client = actionlib.SimpleActionClient(ns+'/torque_controller', ControllerAction)
        rospy.Subscriber(ns+'/joint_states', JointState, self.joint_callback, queue_size=10)
        rospy.Subscriber('hand_pose', hand_pose, self.hand_pose_callback)

        self.publish_human_input = rospy.Publisher('human_input', Bool, queue_size=1)

        rospy.loginfo("Initializing client: Waiting for server")
        self.client.wait_for_server()
        rospy.loginfo("Server initialized")

        rospy.Rate(0.5).sleep() # Give enough time for the FRIoverlay app to start up

        self.goal_time = 5.0
        self.hand_pose = None

        self.type = 'none'

        self.initialise = True

    def joint_callback(self, msg):
        self.q = msg.position
        self.q_dot = msg.velocity
        if self.robot is not None:
            ee_T = np.array(self.robot.fkine(self.q, end='iiwa_link_7', start='iiwa_link_0'))
            # ee_T_dot = np.array(self.robot.fkine(self.q_dot, end='iiwa_link_7', start='iiwa_link_0'))

            # ee_vel = [ee_T_dot[0, 3], ee_T_dot[1, 3], ee_T_dot[2, 3]]

            
            # if np.linalg.norm(ee_vel) > 10:
            #     self.publish_human_input.publish(True)
            # else:
            #     self.publish_human_input.publish(False)

            translation = ee_T[:3,3]
            rot_mat = ee_T[0:3, 0:3]
            r = R.from_matrix(rot_mat)
            euler_angles = r.as_euler('xyz', degrees=False)
            self.ee_pose = np.hstack((translation,np.flip(euler_angles)))
            


    def hand_pose_callback(self, msg):
        self.hand_pose = [msg.x, msg.y, msg.z]

    def create_goal(self, position, nullspace=None, goal_time = None, mode = None):
        goal = ControllerGoal()

        if goal_time == None:
            if self.type == 'fast':
                goal.time = 5 - 2.0
            elif self.type == 'slow':
                goal.time = 5 + 2.0
            else:
                goal.time = 5
        else:
            goal.time = goal_time

        # Auto select correct goal-mode based on input arguments
        if len(position) == 7:
            goal.mode = 'joint_ds'
            stiffness = [100.0, 100.0, 50.0, 50.0, 25.0, 10.0, 10.0]
            goal.stiffness = stiffness
            goal.damping = (2 * np.sqrt(stiffness)).tolist()
        elif len(position) == 6:
            if mode == None:
                goal.mode = 'ee_cartesian_ds'
                stiffness = [80.0, 80.0, 80.0, 5.0, 5.0, 5.0]
            elif mode == 'ee_cartesian':
                goal.mode = mode
                stiffness = [80.0, 80.0, 80.0, 5.0, 3.0, 3.0]
                rospy.Rate(1).sleep()
            goal.stiffness = stiffness
            goal.damping = (2 * np.sqrt(stiffness)).tolist()
        elif len(position) not in [6,7]:
            raise ValueError("Requested position MUST be either all joint angles or the full cartesian position [x,y,z,rx,ry,rz]")

        goal.precision = 1e-1
        goal.rate = 20

        if nullspace == None:
            goal.nullspace_reference = [0,0,0,0,0,0,0]
            goal.nullspace_gain = [0,0,0,0,0,0,0]
        else:
            goal.nullspace_reference = nullspace
            goal.nullspace_gain = np.array([10,10,10,10,10,10,10])

        goal.reference = position
        goal.velocity_reference = np.zeros(6)

        return goal

    def send_position_command(self, position, nullspace,goal_time = None, mode=None):
        if not isinstance(position,ControllerGoal):
            goal = self.create_goal(position, nullspace,goal_time,mode)
        else:
            goal = position
        try:
            self.client.wait_for_server()
            self.client.send_goal(goal)
            self.client.wait_for_result()
        except actionlib.ActionException as e:
            rospy.logerr(f"Action client error: {e}")
            return False
        return True

    def move_towards_hand(self):
        rospy.loginfo("Moving towards hand")
        fixed_orientation = self.ee_pose[3:]
        if self.q[4] < -1:
            fixed_orientation[1] = -fixed_orientation[1] # euler angles have 2 solutions causing flipping of axis. This is not a fix, specific patch for 2 predetermined locations
        fixed_position = self.ee_pose[0:3]

        if self.hand_pose == None:
            target_position_arm = fixed_position
        else:
            target_position_cam = np.array(self.hand_pose)
            target_position_arm = self.frame_transform(target_position_cam)

        current_position = np.array(self.ee_pose[:3])

        position_threshold = 0.3

        if self.hand_pose == None:
            wait_for_hand = True
        else:
            wait_for_hand = False

        while (np.linalg.norm(target_position_arm - current_position) > position_threshold) or wait_for_hand:
            if self.hand_pose != None:
                wait_for_hand = False

            if self.hand_pose is None:
                rospy.loginfo(f"hand_pose=:{self.hand_pose}")
            else:
                transformed_pose = self.frame_transform(np.array(self.hand_pose))
                transformed_pose[2] = 0.1 if transformed_pose[2] < 0.1 else transformed_pose[2]
                rospy.loginfo(f"hand_pose=:{[f'{x:.3f}' for x in transformed_pose]}")


            if self.hand_pose == None:
                target_position_arm = np.array(fixed_position) 
            else:
                target_position_arm = self.frame_transform(np.array(self.hand_pose))


            current_position = np.array(self.ee_pose[:3])
            target_pose = np.hstack((target_position_arm,fixed_orientation))
            goal_time = 2
            self.send_position_command(target_pose, None, goal_time)

        rospy.loginfo("Reached the hand position")
        self.hand_pose = None
       
        return
    
    def frame_transform(self,target):
        """
        Manual calibration camera kuka iiwa7:

        translation: x=0.276, y=0.146, z=2.643

        180 deg rotation about x then 90 deg rotation about z
        """
        
        offset = np.array([0.356, 0.256, 2.643]) # experimental values obtained around operating region

        rot_x_180 = np.array([[1,0,0],[0,-1,0],[0,0,-1]]) 
        rot_z_90 = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        rot_tot = np.dot(rot_x_180,rot_z_90)
        
        transform = np.column_stack((rot_tot,offset))
        transform = np.vstack((transform,np.array([0,0,0,1])))

        target_hom = np.append(target[0:3],1)

        return np.dot(transform,target_hom)[:3]
    
         
    
    def test(self,experiment,positions = None):
        rot= np.deg2rad([107, -47, -11, 100, -82, -82, -35])
        count = 0

        initialise = True

        #random set of x,y,z coordinates from simulated camera:
        if positions == None:
            positions= np.array([[-0.6,0.5,2.1],
                                [-0.7,0.4,2.0],
                                [-0.6,0.3,2.1],
                                [-0.5,0.4,2.0]])
                           

        while not rospy.is_shutdown():
            if experiment == 0:
                if count == 0:
                    rospy.loginfo("Sending pre-defined position command")
                    self.send_position_command(rot)

                target = self.ee_pose
        
                # for some reason this axis is inverted
                target[4] = -target[4] 
            
                # Modify values to test if arm orientation changes
                target[0] += 0.1
                target[1] -= 0.1
                target[2] += 0.1

                rospy.loginfo("sending dynamic position command, forward")
                self.send_position_command(target)

                target = self.ee_pose
                target[4] = -target[4] 

                # Move back to original position
                target[0] -= 0.1
                target[1] += 0.1
                target[2] -= 0.1

                rospy.loginfo("sending dynamic position command, backwards")
                self.send_position_command(target)

                count += 1

            if experiment == 1:
                if initialise:
                    self.send_position_command(rot,None)
                    count = 0
                    orientation = self.ee_pose[3:6]
                    orientation[1] = -orientation[1] #BUG
                    full_pose = self.q
                    initialise = False
                    
                print("ee_pose at handover orientation:",orientation)

                xyz = self.frame_transform()

                target = np.hstack((xyz,orientation))

                print("target:",target)

                self.send_position_command(target,None)

                count+=1
                if count == 4:
                    count = 0

            if experiment == 2:
                if initialise:
                    self.send_position_command(rot,None)
                    initialise = False
                    saved_pose = self.ee_pose
                    saved_pose[4] = -saved_pose[4] 
                    initialise = False

                print("current pose:",self.ee_pose)

                print(f"switching to ee_control input:{saved_pose}")

                self.send_position_command(saved_pose,None)

            if experiment == 3:
                serve = np.deg2rad([107, -47, -11, 100, -82, -82, -35]) # Serve
                drop = np.deg2rad([55, -40, -8, 82, 5, 20, 0]) # Drop
                self.send_position_command(drop,None)
                print("self.q for drop:",self.q)
                self.move_towards_hand()
                

                self.send_position_command(serve,None)
                print("self.q for serve:",self.q)
                self.move_towards_hand()
               
                #self.move_towards_hand()


            if experiment == 4:
                 if initialise:
                     self.send_position_command(rot,None)

                     fixed_orientation = self.ee_pose[3:]
                     fixed_orientation[1] = -fixed_orientation[1] #BUG
                     
                     initialise = False

                 
                 target_positions= {0: [-0.32212266, -0.53631857,  0.42099988],
                                    1: [-0.36131503, -0.5264838,   0.49599993],
                                    2: [-0.33462706, -0.65330973,  0.26599991],
                                    3: [-0.49206611, -0.63994768,  0.14599978],
                                    4: [-0.28143831, -0.39959577,  0.33499991],
                                    5: [-0.14201932, -0.28211292,  0.1739999 ],
                                    6: [-0.21154168, -0.2970249,   0.08799993]}
                 
                 goal = target_positions[count] + fixed_orientation

                 print(target_positions[count])
                 
                 self.send_position_command(goal,None,mode='ee_cartesian')

                 count += 1

                 if count == 7:
                     count = 0



            
if __name__=='__main__':
    rospy.init_node("test")
    controller = RoboticArmController()
   
    while True:
        controller.test(3)
    #controller.test(1)
    #print(controller.frame_transform(np.array([0.8,0.2,2.6])))