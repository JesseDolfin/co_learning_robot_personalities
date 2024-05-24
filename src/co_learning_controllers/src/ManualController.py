import rospy
import rosgraph
from co_learning_messages.msg import secondary_task_message
import signal
import sys

class ManualControl():
    def __init__(self) -> None:
        self.initialise_ros()

        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.stop = True
        sys.exit(0)

    def initialise_ros(self):
        self.ros_running = rosgraph.is_master_online()
        if self.ros_running:
            try:
                rospy.init_node('ManualControl', anonymous=True)
                self.pub = rospy.Publisher('Task_status', secondary_task_message, queue_size=1)
                self.rate = rospy.Rate(5)
            except rospy.ROSException:
                rospy.logwarn("Cannot initialize node 'ManualControl' as it has already been initialized at the top level as a ROS node.")
        else:
            print("ROS is offline! Environment proceeds in offline mode")

    def send_message(self, successful):
        msg = secondary_task_message()
        msg.handover_successful = successful
        self.pub.publish(msg)
        rospy.loginfo(f"Message sent with handover_successful={successful}")

    def run(self):
        while not rospy.is_shutdown():
            print("Enter 's' for successful and 'f' for unsuccessful:")
            goal = input().strip()
            if goal == 's':
                self.send_message(1)
            elif goal == 'f':
                self.send_message(-1)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        control = ManualControl()
        control.run()
    except rospy.ROSInterruptException:
        pass
