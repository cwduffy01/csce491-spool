#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32


class TetherModel():
    prev_dist = 0
    curr_dist = 0

    prev_length = 0
    curr_length = 0

    prev_time = 0

    def __init__(self):
        rospy.init_node("tether_model")

        rospy.Subscriber("/duckie_pose", Pose2D, self.pose_callback)
        self.pub = rospy.Publisher("/target_tether_length", Float32, queue_size=10)

        rospy.spin()

    def pose_callback(self, msg):
        dist = (msg.x**2 + msg.y**2) ** 0.5
        tether_len = 1.2 * dist + 0.2
        # print(tether_len)
        self.pub.publish(tether_len)
        
    
if __name__ == "__main__":
    TetherModel()